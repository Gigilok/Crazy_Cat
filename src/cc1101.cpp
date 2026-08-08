#include <SPI.h>
#include "config.h"

// ============================================================
// BARRAMENTO SPI DEDICADO PARA CC1101 (HSPI)
// Separado do VSPI que o NRF24 usa - SEM CONFLITO
// ============================================================
SPIClass spiCC1101(HSPI);

// ============================================================
// REGISTRADORES CC1101
// ============================================================
#define CC1101_IOCFG2   0x00
#define CC1101_IOCFG0   0x02
#define CC1101_FIFOTHR  0x03
#define CC1101_PKTCTRL0 0x08
#define CC1101_FREQ2    0x0D
#define CC1101_FREQ1    0x0E
#define CC1101_FREQ0    0x0F
#define CC1101_MDMCFG4  0x10
#define CC1101_MDMCFG3  0x11
#define CC1101_MDMCFG2  0x12
#define CC1101_MDMCFG1  0x13
#define CC1101_MDMCFG0  0x14
#define CC1101_DEVIATN  0x15
#define CC1101_MCSM0    0x18
#define CC1101_FOCCFG   0x19
#define CC1101_BSCFG    0x1A
#define CC1101_AGCCTRL2 0x1B
#define CC1101_AGCCTRL1 0x1C
#define CC1101_AGCCTRL0 0x1D
#define CC1101_FREND0   0x22
#define CC1101_FSCAL3   0x23
#define CC1101_FSCAL2   0x24
#define CC1101_FSCAL1   0x25
#define CC1101_FSCAL0   0x26
#define CC1101_TEST2    0x2C
#define CC1101_TEST1    0x2D
#define CC1101_TEST0    0x2E
#define CC1101_PATABLE  0x3E

#define CC1101_PARTNUM  0x30
#define CC1101_VERSION  0x31
#define CC1101_RSSI     0x34
#define CC1101_MARCSTATE 0x35

#define CC1101_SRES     0x30
#define CC1101_SCAL     0x33
#define CC1101_SRX      0x34
#define CC1101_STX      0x35
#define CC1101_SIDLE    0x36

#define CC1101_READ_SINGLE  0x80
#define CC1101_READ_BURST   0xC0
#define CC1101_WRITE_BURST  0x40

// ============================================================
// ESTADO GLOBAL
// ============================================================
bool cc1101Initialized = false;
bool cc1101RollJamActive = false;
uint8_t rj_state = 0;
unsigned long rj_timer = 0;

extern unsigned long captureStartTime;

// ============================================================
// CAPTURA - frequencias e estado
// ============================================================
static const uint32_t captureFreqs[] = {433920000, 315000000, 868000000, 915000000};
static const uint8_t  captureFreqCount = 4;
static uint8_t currentFreqIndex = 0;
static unsigned long lastFreqSwitch = 0;

enum CaptureState {
    CAP_STATE_HOPPING,
    CAP_STATE_LOCKED,
    CAP_STATE_CAPTURING
};
static uint8_t capture_state = CAP_STATE_HOPPING;

struct SignalCapture {
    uint16_t timings[200];
    uint8_t count;
    uint32_t frequency;
    bool active;
    bool scanning;
    unsigned long startTime;
    uint8_t lastValue;
    unsigned long lastChangeTime;
};
SignalCapture currentCapture;

// Buffer ISR - captura timings brutos do GDO0
volatile uint16_t isr_timings[200];
volatile uint8_t isr_count = 0;
volatile unsigned long isr_last_change = 0;
volatile uint8_t isr_last_val = 0;
volatile bool capture_started = false;
volatile bool isr_enabled = false;

// ============================================================
// ANALISADOR DE ESPECTRO
// ============================================================
uint16_t spec_an_values[64];
uint32_t spec_an_freqs[64];
uint8_t spec_an_idx = 0;
bool spec_an_running = false;

// ============================================================
// PROTOTIPOS
// ============================================================
static void tuneToFreq(uint32_t freqHz);
static void cc1101ISR();

// ============================================================
// ISR MANUAL - captura timings brutos do GDO0
// Captura QUALQUER sinal: rolling codes, Keeloq, HCS301, etc.
// ============================================================
void IRAM_ATTR cc1101ISR() {
    if (!isr_enabled) return;
    unsigned long now = micros();
    uint8_t val = digitalRead(CC1101_GDO0);
    if (val != isr_last_val) {
        unsigned long duration = now - isr_last_change;
        // Filtro de ruido: pulsos entre 50us e 100ms
        if (duration > 50 && duration < 100000) {
            if (isr_count < 200) {
                isr_timings[isr_count] = (uint16_t)duration;
                isr_count++;
            }
            capture_started = true;
        }
        isr_last_val = val;
        isr_last_change = now;
    }
}

// ============================================================
// HELPERS DE PINAGEM
// ============================================================
// CRITICO: apos CSN LOW, devemos esperar o chip ficar pronto.
// O CC1101 segura MISO em HIGH quando esta ocupado (wake-up, calibracao,
// transicao de estado). Se comecarmos a transferencia antes do chip
// estar pronto, ele responde com 0x00 (lixo).
//
// Bug historico: versoes anteriores NAO faziam essa espera, causando
// todos os registradores lerem 0x00 mesmo com chip funcionando.
void cc1101Select() {
    digitalWrite(CC1101_CSN, LOW);
    // Espera MISO ir para LOW (chip pronto) - timeout 1ms
    for (uint16_t i = 0; i < 100; i++) {
        if (digitalRead(CC1101_MISO) == LOW) break;
        delayMicroseconds(10);
    }
    // Pequeno delay de seguranca (alguns clones sao lentos)
    delayMicroseconds(30);
}
void cc1101Deselect() { digitalWrite(CC1101_CSN, HIGH); }

// ============================================================
// FUNCOES DE SPI MANUAL (usa HSPI dedicado, nao VSPI)
// SPI a 1MHz para maxima compatibilidade com clones chineses
// (4MHz pode causar leituras erradas em modulos baratos)
// ============================================================
uint8_t cc1101ReadReg(uint8_t reg) {
    spiCC1101.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    cc1101Select();
    spiCC1101.transfer(reg | CC1101_READ_SINGLE);
    uint8_t val = spiCC1101.transfer(0x00);
    cc1101Deselect();
    spiCC1101.endTransaction();
    return val;
}

uint8_t cc1101ReadStatus(uint8_t reg) {
    spiCC1101.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    cc1101Select();
    spiCC1101.transfer(reg | CC1101_READ_BURST);
    uint8_t val = spiCC1101.transfer(0x00);
    cc1101Deselect();
    spiCC1101.endTransaction();
    return val;
}

void cc1101WriteReg(uint8_t reg, uint8_t value) {
    spiCC1101.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    cc1101Select();
    spiCC1101.transfer(reg);
    spiCC1101.transfer(value);
    cc1101Deselect();
    spiCC1101.endTransaction();
}

void cc1101SendCommand(uint8_t cmd) {
    spiCC1101.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    cc1101Select();
    spiCC1101.transfer(cmd);
    cc1101Deselect();
    spiCC1101.endTransaction();
    delayMicroseconds(100);
}

void cc1101SetFrequency(uint32_t freqHz) {
    // Calcula palavra de frequencia: F = (freq / 26MHz) * 65536
    uint32_t freqWord = (uint32_t)((freqHz / 26000000.0) * 65536);
    cc1101WriteReg(CC1101_FREQ2, (freqWord >> 16) & 0xFF);
    cc1101WriteReg(CC1101_FREQ1, (freqWord >> 8) & 0xFF);
    cc1101WriteReg(CC1101_FREQ0, freqWord & 0xFF);
}

// ============================================================
// INICIALIZACAO
// ============================================================
bool cc1101Init() {
    Serial.println(F("[CC1101] Inicializando v3.6 (SPI timing fix)..."));
    Serial.flush();

    // 1. Inicia barramento SPI dedicado (HSPI) nos pinos do CC1101
    //    Isso NAO conflita com o VSPI do NRF24
    spiCC1101.begin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CSN);
    pinMode(CC1101_CSN, OUTPUT);
    digitalWrite(CC1101_CSN, HIGH);
    pinMode(CC1101_GDO0, INPUT_PULLUP);
    pinMode(CC1101_GDO2, INPUT);

    // 2. Reset do CC1101 (pulse CSN)
    digitalWrite(CC1101_CSN, LOW);
    delayMicroseconds(10);
    digitalWrite(CC1101_CSN, HIGH);
    delay(100);

    // 3. Reset via comando SRES
    uint8_t partnum = 0xFF;
    for (int i = 0; i < 3; i++) {
        cc1101SendCommand(CC1101_SRES);
        delay(10);
        partnum = cc1101ReadStatus(CC1101_PARTNUM);
        // PARTNUM=0x00 e o valor CORRETO do CC1101 (datasheet TI)
        // So 0xFF significa que o modulo nao responde
        if (partnum != 0xFF) break;
        delay(50);
    }

    Serial.printf("[CC1101] PARTNUM lido = 0x%02X\n", partnum);
    if (partnum == 0xFF) {
        Serial.println(F("[CC1101] FAIL: modulo nao responde (PARTNUM=0xFF)"));
        Serial.println(F("[CC1101] Verifique alimentacao 3.3V e pinos SPI"));
        return false;
    }

    // DETECCAO DE MISO PRESO EM LOW:
    // PARTNUM=0x00 pode ser valor correto OU MISO preso em LOW.
    // Para diferenciar, lemos VERSION - se tambem for 0x00, MISO esta preso.
    uint8_t version_check = cc1101ReadStatus(CC1101_VERSION);
    Serial.printf("[CC1101] VERSION lido = 0x%02X\n", version_check);
    if (version_check == 0x00) {
        Serial.println(F(""));
        Serial.println(F("[CC1101] ========================================"));
        Serial.println(F("[CC1101]  ERRO CRITICO: MISO PRESO EM LOW!"));
        Serial.println(F("[CC1101] ========================================"));
        Serial.println(F("[CC1101] Sintoma: todos registradores leem 0x00"));
        Serial.println(F("[CC1101] Causa mais provavel (em ordem):"));
        Serial.println(F("[CC1101]   1. Resistor pull-down no GPIO 12 em CURTO"));
        Serial.println(F("[CC1101]      -> DESSOLDE o resistor e teste sem ele"));
        Serial.println(F("[CC1101]   2. Fio MISO do CC1101 desconectado/quebrado"));
        Serial.println(F("[CC1101]      -> Verifique continuidade do fio"));
        Serial.println(F("[CC1101]   3. Curto fisico entre GPIO 12 e GND"));
        Serial.println(F("[CC1101]      -> Multimetro: GPIO12<->GND nao deve apitar"));
        Serial.println(F("[CC1101]   4. Pino GPIO 12 queimado no ESP32"));
        Serial.println(F("[CC1101]      -> Mude CC1101_MISO no config.h para outro pino"));
        Serial.println(F("[CC1101] ========================================"));
        Serial.println(F("[CC1101] Continuando boot sem CC1101..."));
        Serial.println(F(""));
        Serial.flush();
        return false;
    }

    // 4. Escreve registradores de configuracao (OOK, 500kHz RxBW)
    //    Valores validados do datasheet TI e do projeto ESP32-DIV
    cc1101WriteReg(CC1101_IOCFG0,   0x0D);  // GDO0 = async serial data out
    cc1101WriteReg(CC1101_IOCFG2,   0x0D);  // GDO2 = async serial data out
    cc1101WriteReg(CC1101_FIFOTHR,  0x07);  // RX FIFO threshold
    cc1101WriteReg(CC1101_PKTCTRL0, 0x32);  // Async serial, infinite length
    cc1101WriteReg(CC1101_FREQ2,    0x10);  // 433.92 MHz
    cc1101WriteReg(CC1101_FREQ1,    0xB1);
    cc1101WriteReg(CC1101_FREQ0,    0x3B);
    cc1101WriteReg(CC1101_MDMCFG4,  0x17);  // RxBW = 325 kHz (DRATE_E=7)
    cc1101WriteReg(CC1101_MDMCFG3,  0x32);  // Data rate = 250 kBaud
    cc1101WriteReg(CC1101_MDMCFG2,  0x30);  // OOK, no Manchester, 30/32 sync
    cc1101WriteReg(CC1101_MDMCFG1,  0x00);
    cc1101WriteReg(CC1101_MDMCFG0,  0x00);
    cc1101WriteReg(CC1101_DEVIATN,  0x15);  // FSK deviation (nao importa em OOK)
    cc1101WriteReg(CC1101_MCSM0,    0x18);  // Auto-calibrate on IDLE->RX/TX
    cc1101WriteReg(CC1101_FOCCFG,   0x18);
    cc1101WriteReg(CC1101_BSCFG,    0x1C);
    cc1101WriteReg(CC1101_AGCCTRL2, 0x07);  // Max AGC target
    cc1101WriteReg(CC1101_AGCCTRL1, 0x00);
    cc1101WriteReg(CC1101_AGCCTRL0, 0x91);  // Worst-case, max LNA gain
    cc1101WriteReg(CC1101_FREND0,   0x11);  // PATABLE[0] for TX
    cc1101WriteReg(CC1101_FSCAL3,   0xE9);
    cc1101WriteReg(CC1101_FSCAL2,   0x2A);
    cc1101WriteReg(CC1101_FSCAL1,   0x00);
    cc1101WriteReg(CC1101_FSCAL0,   0x1F);
    cc1101WriteReg(CC1101_TEST2,    0x81);
    cc1101WriteReg(CC1101_TEST1,    0x35);
    cc1101WriteReg(CC1101_TEST0,    0x09);

    // 5. Configura PATABLE (potencia de TX: 0xC0 = ~+10dBm)
    spiCC1101.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    cc1101Select();
    spiCC1101.transfer(CC1101_PATABLE | CC1101_WRITE_BURST);
    for (int i = 0; i < 8; i++) spiCC1101.transfer(0xC0);
    cc1101Deselect();
    spiCC1101.endTransaction();

    // 6. Anexa ISR manual no GDO0
    attachInterrupt(digitalPinToInterrupt(CC1101_GDO0), cc1101ISR, CHANGE);
    isr_enabled = false;

    cc1101Initialized = true;
    Serial.println(F("[CC1101] Configurado com sucesso!"));

    // === DIAGNOSTICO COMPLETO ===
    Serial.println(F("[CC1101] === DIAGNOSTICO COMPLETO ==="));

    // TESTE DE WRITE-READBACK: prova irrefutavel de MISO preso
    cc1101WriteReg(CC1101_IOCFG0, 0xAB);
    delay(2);
    uint8_t readback = cc1101ReadReg(CC1101_IOCFG0);
    Serial.printf("  TESTE WRITE-READBACK: escreveu 0xAB, leu 0x%02X\n", readback);
    if (readback != 0xAB) {
        Serial.println(F("  >>> FALHA NO READBACK! MISO preso em LOW."));
        Serial.println(F("  >>> Causa: curto no GPIO 12 ou fio MISO desconectado."));
    }
    cc1101WriteReg(CC1101_IOCFG0, 0x0D);  // restaura valor correto

    uint8_t partnum_val = cc1101ReadStatus(CC1101_PARTNUM);
    uint8_t version_val = cc1101ReadStatus(CC1101_VERSION);
    uint8_t marcstate   = cc1101ReadStatus(CC1101_MARCSTATE) & 0x1F;
    uint8_t iocfg0_read = cc1101ReadReg(CC1101_IOCFG0);
    uint8_t iocfg2_read = cc1101ReadReg(CC1101_IOCFG2);
    uint8_t mdmcfg2     = cc1101ReadReg(CC1101_MDMCFG2);
    uint8_t mdmcfg4     = cc1101ReadReg(CC1101_MDMCFG4);
    uint8_t freq2       = cc1101ReadReg(CC1101_FREQ2);
    uint8_t freq1       = cc1101ReadReg(CC1101_FREQ1);
    uint8_t freq0       = cc1101ReadReg(CC1101_FREQ0);

    // Le RSSI (precisa estar em RX para ter valor real)
    uint8_t rssiDec = cc1101ReadStatus(CC1101_RSSI);
    int rssi = (rssiDec >= 128) ? ((int)rssiDec - 256) / 2 - 74
                                : (int)rssiDec / 2 - 74;

    Serial.printf("  PARTNUM  = 0x%02X (esperado 0x00)\n", partnum_val);
    Serial.printf("  VERSION  = 0x%02X (esperado 0x04 ou 0x14)\n", version_val);
    Serial.printf("  MARCSTATE = 0x%02X (0x0D=RX, 0x01=IDLE, 0x00=SLEEP)\n", marcstate);
    Serial.printf("  RSSI     = %d dBm\n", rssi);
    Serial.printf("  GDO0 pin = %d\n", digitalRead(CC1101_GDO0));
    Serial.printf("  GDO2 pin = %d\n", digitalRead(CC1101_GDO2));
    Serial.printf("  IOCFG0   = 0x%02X (esperado 0x0D)\n", iocfg0_read);
    Serial.printf("  IOCFG2   = 0x%02X (esperado 0x0D)\n", iocfg2_read);
    Serial.printf("  MDMCFG2  = 0x%02X (esperado 0x30 = OOK)\n", mdmcfg2);
    Serial.printf("  MDMCFG4  = 0x%02X (esperado 0x17)\n", mdmcfg4);
    Serial.printf("  FREQ2    = 0x%02X (esperado 0x10)\n", freq2);
    Serial.printf("  FREQ1    = 0x%02X (esperado 0xB1)\n", freq1);
    Serial.printf("  FREQ0    = 0x%02X (esperado 0x3B)\n", freq0);

    // Entra em RX apos o diagnostico
    cc1101SendCommand(CC1101_SIDLE);
    delay(2);
    cc1101SendCommand(CC1101_SCAL);
    delay(2);
    cc1101SendCommand(CC1101_SRX);
    delay(5);

    marcstate = cc1101ReadStatus(CC1101_MARCSTATE) & 0x1F;
    Serial.printf("  MARCSTATE apos SRX = 0x%02X (esperado 0x0D=RX)\n", marcstate);

    Serial.println(F("[CC1101] === FIM DO DIAGNOSTICO ==="));
    Serial.flush();

    return true;
}

// ============================================================
// CAPTURA - Copiar Sinal (RAW universal)
// ============================================================
static void tuneToFreq(uint32_t freqHz) {
    cc1101SendCommand(CC1101_SIDLE);
    delay(2);
    cc1101SetFrequency(freqHz);
    // Re-configura IOCFG apos mudar freq (alguns clones resetam)
    cc1101WriteReg(CC1101_IOCFG0, 0x0D);
    cc1101WriteReg(CC1101_IOCFG2, 0x0D);
    cc1101SendCommand(CC1101_SCAL);
    delay(2);
    cc1101SendCommand(CC1101_SRX);
    delay(5);
}

void cc1101StartCapture() {
    if (!cc1101Initialized) return;
    cc1101CopyActive = true;
    currentCapture.count = 0;
    currentCapture.startTime = millis();
    captureStartTime = currentCapture.startTime;
    currentFreqIndex = 0;
    currentCapture.frequency = captureFreqs[currentFreqIndex];
    lastFreqSwitch = millis();
    capture_state = CAP_STATE_HOPPING;
    isr_count = 0;
    capture_started = false;
    isr_enabled = false;

    // Configura GDO0/GDO2 como saida de dados assincrona
    cc1101WriteReg(CC1101_IOCFG0, 0x0D);
    cc1101WriteReg(CC1101_IOCFG2, 0x0D);
    pinMode(CC1101_GDO0, INPUT_PULLUP);

    // Tuning inicial
    tuneToFreq(currentCapture.frequency);

    // Ativa ISR desde o inicio
    isr_last_val = digitalRead(CC1101_GDO0);
    isr_last_change = micros();
    isr_enabled = true;

    Serial.printf("[CC1101] Capture iniciada - freq inicial %lu Hz\n",
                  currentCapture.frequency);
    Serial.flush();
}

void cc1101CaptureLoop() {
    if (!cc1101CopyActive) return;
    unsigned long now = micros();
    unsigned long nowMs = millis();

    // Condicoes de save
    bool silenceTimeout = (capture_started && (now - isr_last_change > 50000));
    bool bufferFull     = (isr_count >= 200);
    bool totalTimeout   = (nowMs - currentCapture.startTime > CAPTURE_DURATION);

    if (silenceTimeout || bufferFull) {
        isr_enabled = false;

        if (isr_count > 20 && savedSignalCount < MAX_SAVED_SIGNALS) {
            // Sinal valido capturado!
            SignalData* sig = &savedSignals[savedSignalCount];
            sig->length     = isr_count;
            sig->frequency  = currentCapture.frequency;
            sig->modulation = 0;
            sig->valid      = true;

            uint32_t totalDuration = 0;
            for (int i = 0; i < sig->length && i < 200; i++) {
                sig->timings[i] = isr_timings[i];
                totalDuration += sig->timings[i];
            }

            // Classificacao
            if (sig->length < 25 && totalDuration < 30000) {
                snprintf(sig->name, 16, "Sensor %luM", sig->frequency / 1000000);
            } else if (sig->length >= 24 && sig->length <= 50) {
                snprintf(sig->name, 16, "Portao %luM", sig->frequency / 1000000);
            } else if (sig->length > 50 || totalDuration > 70000) {
                snprintf(sig->name, 16, "Carro %luM", sig->frequency / 1000000);
            } else {
                snprintf(sig->name, 16, "Sinal %luM", sig->frequency / 1000000);
            }
            savedSignalCount++;

            Serial.printf("[CC1101] CAPTURADO! %s - %d pulsos, %lu us total\n",
                          sig->name, sig->length, totalDuration);
            Serial.flush();

            // Continua capturando na mesma freq
            capture_state = CAP_STATE_LOCKED;
            lastFreqSwitch = nowMs;
            isr_count = 0;
            capture_started = false;
            isr_last_val = digitalRead(CC1101_GDO0);
            isr_last_change = micros();
            isr_enabled = true;
            return;
        }

        // Sinal muito curto - reseta e continua
        isr_count = 0;
        capture_started = false;
        isr_last_val = digitalRead(CC1101_GDO0);
        isr_last_change = micros();
        isr_enabled = true;
    }

    // Maquina de hopping
    if (capture_state == CAP_STATE_HOPPING) {
        if (isr_count > 5) {
            capture_state = CAP_STATE_LOCKED;
            lastFreqSwitch = nowMs;
            Serial.printf("[CC1101] LOCK em %lu Hz (pulsos=%d)\n",
                          currentCapture.frequency, isr_count);
        } else if (nowMs - lastFreqSwitch > 1500) {
            // Sem sinal, troca de frequencia
            isr_enabled = false;
            isr_count = 0;
            capture_started = false;
            currentFreqIndex = (currentFreqIndex + 1) % captureFreqCount;
            currentCapture.frequency = captureFreqs[currentFreqIndex];
            tuneToFreq(currentCapture.frequency);
            isr_last_val = digitalRead(CC1101_GDO0);
            isr_last_change = micros();
            isr_enabled = true;
            lastFreqSwitch = nowMs;
        }
    } else if (capture_state == CAP_STATE_LOCKED) {
        if (nowMs - lastFreqSwitch > 2000 && isr_count < 20) {
            capture_state = CAP_STATE_HOPPING;
            lastFreqSwitch = nowMs;
            isr_count = 0;
            capture_started = false;
        }
    }

    // Timeout total
    if (totalTimeout) {
        currentCapture.startTime = millis();
        captureStartTime = millis();
        isr_count = 0;
        capture_started = false;
        capture_state = CAP_STATE_HOPPING;
        currentFreqIndex = 0;
        currentCapture.frequency = captureFreqs[currentFreqIndex];
        lastFreqSwitch = millis();
        tuneToFreq(currentCapture.frequency);
        isr_last_val = digitalRead(CC1101_GDO0);
        isr_last_change = micros();
        isr_enabled = true;
    }
}

void cc1101StopCapture() {
    isr_enabled = false;
    cc1101CopyActive = false;
    currentCapture.active = false;
    cc1101WriteReg(CC1101_IOCFG0, 0x0D);
    cc1101SendCommand(CC1101_SIDLE);
    cc1101SendCommand(CC1101_SRX);
}

uint8_t cc1101GetPulseCount() { return isr_count; }
uint32_t cc1101GetCurrentFreq() { return currentCapture.frequency / 1000000; }
uint8_t cc1101GetPinState() { return digitalRead(CC1101_GDO0); }

// ============================================================
// REPLAY - bit-bang manual dos timings brutos
// ============================================================
void cc1101ReplaySignal(uint8_t index) {
    if (index >= savedSignalCount || !savedSignals[index].valid) return;
    if (!cc1101Initialized) return;

    isr_enabled = false;
    SignalData* sig = &savedSignals[index];

    Serial.printf("[CC1101] Replay: %s @ %lu Hz, %d timings\n",
                  sig->name, sig->frequency, sig->length);

    cc1101SetFrequency(sig->frequency);
    cc1101WriteReg(CC1101_IOCFG0, 0x2E);  // GDO0 = TX data input
    cc1101SendCommand(CC1101_SIDLE);
    delay(1);
    cc1101SendCommand(CC1101_STX);
    delay(1);

    pinMode(CC1101_GDO0, OUTPUT);
    for (int i = 0; i < sig->length; i++) {
        digitalWrite(CC1101_GDO0, i % 2 == 0 ? HIGH : LOW);
        delayMicroseconds(sig->timings[i]);
    }
    digitalWrite(CC1101_GDO0, LOW);

    // Volta para RX
    delay(10);
    pinMode(CC1101_GDO0, INPUT_PULLUP);
    cc1101WriteReg(CC1101_IOCFG0, 0x0D);
    cc1101WriteReg(CC1101_IOCFG2, 0x0D);
    cc1101SendCommand(CC1101_SIDLE);
    cc1101SendCommand(CC1101_SRX);
}

// ============================================================
// BRUTEFORCE - codigo de 24 bits
// ============================================================
void cc1101SendBruteForceCode(uint32_t code, uint32_t freq) {
    if (!cc1101Initialized) return;
    isr_enabled = false;

    cc1101SetFrequency(freq);
    cc1101WriteReg(CC1101_IOCFG0, 0x2E);
    cc1101SendCommand(CC1101_SIDLE); delay(1);
    cc1101SendCommand(CC1101_STX); delay(1);

    pinMode(CC1101_GDO0, OUTPUT);

    for (int rep = 0; rep < 3; rep++) {
        for (int i = 23; i >= 0; i--) {
            bool bit = (code >> i) & 0x01;
            if (bit) {
                digitalWrite(CC1101_GDO0, HIGH); delayMicroseconds(900);
                digitalWrite(CC1101_GDO0, LOW);  delayMicroseconds(300);
                digitalWrite(CC1101_GDO0, HIGH); delayMicroseconds(900);
                digitalWrite(CC1101_GDO0, LOW);  delayMicroseconds(300);
            } else {
                digitalWrite(CC1101_GDO0, HIGH); delayMicroseconds(300);
                digitalWrite(CC1101_GDO0, LOW);  delayMicroseconds(900);
                digitalWrite(CC1101_GDO0, HIGH); delayMicroseconds(300);
                digitalWrite(CC1101_GDO0, LOW);  delayMicroseconds(900);
            }
        }
        digitalWrite(CC1101_GDO0, HIGH); delayMicroseconds(300);
        digitalWrite(CC1101_GDO0, LOW);  delayMicroseconds(9300);
    }
    digitalWrite(CC1101_GDO0, LOW);

    pinMode(CC1101_GDO0, INPUT_PULLUP);
    cc1101WriteReg(CC1101_IOCFG0, 0x0D);
    cc1101WriteReg(CC1101_IOCFG2, 0x0D);
    cc1101SendCommand(CC1101_SIDLE);
    cc1101SendCommand(CC1101_SRX);
}

// ============================================================
// JAMMER Sub-GHz
// ============================================================
void cc1101StartSubGHzJammer() {
    if (!cc1101Initialized) return;
    isr_enabled = false;
    cc1101SetFrequency(433920000);
    cc1101WriteReg(CC1101_IOCFG0, 0x2E);
    cc1101SendCommand(CC1101_SIDLE); delay(1);
    cc1101SendCommand(CC1101_STX); delay(2);
    pinMode(CC1101_GDO0, OUTPUT);
    digitalWrite(CC1101_GDO0, HIGH);
    Serial.println(F("[CC1101] SubGHz Jammer ativado em 433.92 MHz"));
}

void cc1101StopSubGHzJammer() {
    if (!cc1101Initialized) return;
    digitalWrite(CC1101_GDO0, LOW);
    pinMode(CC1101_GDO0, INPUT_PULLUP);
    cc1101WriteReg(CC1101_IOCFG0, 0x0D);
    cc1101WriteReg(CC1101_IOCFG2, 0x0D);
    cc1101SendCommand(CC1101_SIDLE);
    cc1101SendCommand(CC1101_SRX);
}

// ============================================================
// ROLLJAM AUTO
// ============================================================
void cc1101StartRollJam() {
    if (!cc1101Initialized) return;
    cc1101RollJamActive = true;
    rj_state = 0;
    rj_timer = millis();
    currentCapture.frequency = 433920000;
    isr_enabled = false;
    cc1101SetFrequency(433920000);
    cc1101WriteReg(CC1101_IOCFG0, 0x0D);
    cc1101WriteReg(CC1101_IOCFG2, 0x0D);
    pinMode(CC1101_GDO0, INPUT_PULLUP);
    cc1101SendCommand(CC1101_SIDLE); delay(1);
    cc1101SendCommand(CC1101_SRX); delay(5);
    isr_last_val = digitalRead(CC1101_GDO0);
    isr_last_change = micros();
    isr_enabled = true;
}

void cc1101RollJamLoop() {
    if (!cc1101RollJamActive) return;
    unsigned long now = millis();
    unsigned long nowUs = micros();

    if (rj_state == 0) {
        if (isr_count > 5 || digitalRead(CC1101_GDO0) == HIGH) {
            isr_count = 0;
            isr_last_val = HIGH;
            isr_last_change = nowUs;
            capture_started = true;
            isr_enabled = true;
            rj_state = 1;
            rj_timer = now;
            Serial.println(F("[CC1101] RollJam: sinal detectado, capturando..."));
        }
    } else if (rj_state == 1) {
        if (now - rj_timer > 200) {
            isr_enabled = false;
            cc1101WriteReg(CC1101_IOCFG0, 0x2E);
            cc1101SendCommand(CC1101_SIDLE); delay(1);
            cc1101SendCommand(CC1101_STX);
            pinMode(CC1101_GDO0, OUTPUT);
            digitalWrite(CC1101_GDO0, HIGH);
            rj_state = 2;
            rj_timer = now;
            Serial.println(F("[CC1101] RollJam: JAMMING..."));
        }
    } else if (rj_state == 2) {
        if (now - rj_timer > 200) {
            digitalWrite(CC1101_GDO0, LOW);
            pinMode(CC1101_GDO0, INPUT_PULLUP);
            cc1101WriteReg(CC1101_IOCFG0, 0x0D);
            cc1101WriteReg(CC1101_IOCFG2, 0x0D);
            cc1101SendCommand(CC1101_SIDLE);

            if (isr_count > 20 && savedSignalCount < MAX_SAVED_SIGNALS) {
                SignalData* sig = &savedSignals[savedSignalCount];
                sig->length = isr_count;
                sig->frequency = currentCapture.frequency;
                sig->modulation = 0;
                sig->valid = true;
                for (int i = 0; i < sig->length && i < 200; i++) {
                    sig->timings[i] = isr_timings[i];
                }
                snprintf(sig->name, 16, "Roubado %luM", sig->frequency / 1000000);
                savedSignalCount++;
                Serial.printf("[CC1101] RollJam: sinal salvo! %d pulsos\n", isr_count);
            }
            cc1101RollJamActive = false;
            cc1101SendCommand(CC1101_SRX);
        }
    }
}

void cc1101StopRollJam() {
    cc1101RollJamActive = false;
    isr_enabled = false;
    digitalWrite(CC1101_GDO0, LOW);
    pinMode(CC1101_GDO0, INPUT_PULLUP);
    cc1101WriteReg(CC1101_IOCFG0, 0x0D);
    cc1101WriteReg(CC1101_IOCFG2, 0x0D);
    cc1101SendCommand(CC1101_SIDLE);
    cc1101SendCommand(CC1101_SRX);
}

// ============================================================
// ANALISADOR DE ESPECTRO
// ============================================================
void cc1101StartAnalyzer() {
    if (!cc1101Initialized) return;
    spec_an_running = true;
    spec_an_idx = 0;
    for (int i = 0; i < 15; i++) spec_an_freqs[i] = 300000000 + (i * 3200000);
    for (int i = 0; i < 16; i++) spec_an_freqs[15 + i] = 387000000 + (i * 4800000);
    for (int i = 0; i < 33; i++) spec_an_freqs[31 + i] = 779000000 + (i * 4500000);
    for (int i = 0; i < 64; i++) spec_an_values[i] = 0;
}

void cc1101AnalyzerLoop() {
    if (!spec_an_running) return;
    for (int i = 0; i < 64; i++) {
        if (spec_an_values[i] > 0) spec_an_values[i]--;
    }
    uint32_t freq = spec_an_freqs[spec_an_idx];
    cc1101SetFrequency(freq);
    cc1101SendCommand(CC1101_SIDLE); delay(1);
    cc1101SendCommand(CC1101_SRX);
    delayMicroseconds(300);

    uint8_t rssiDec = cc1101ReadStatus(CC1101_RSSI);
    int rssi = (rssiDec >= 128) ? ((int)rssiDec - 256) / 2 - 74 : (int)rssiDec / 2 - 74;
    if (rssi < -90) rssi = -90;
    if (rssi > -50) rssi = -50;
    uint16_t target_h = map(rssi, -90, -50, 0, 40);
    if (target_h > spec_an_values[spec_an_idx]) {
        spec_an_values[spec_an_idx] = target_h;
    }
    spec_an_idx++;
    if (spec_an_idx >= 64) spec_an_idx = 0;
}

void cc1101StopAnalyzer() {
    spec_an_running = false;
    cc1101SendCommand(CC1101_SIDLE);
    cc1101SendCommand(CC1101_SRX);
}

bool cc1101AnalyzerIsRunning() { return spec_an_running; }
uint16_t cc1101GetAnalyzerValue(int idx) {
    if (idx < 0 || idx >= 64) return 0;
    return spec_an_values[idx];
}
uint32_t cc1101GetAnalyzerFreq(int idx) {
    if (idx < 0 || idx >= 64) return 0;
    return spec_an_freqs[idx] / 1000000;
}
uint8_t cc1101GetAnalyzerSelected() { return spec_an_idx; }

// ============================================================
// GERENCIAMENTO DE SINAIS
// ============================================================
void cc1101ClearSavedSignals() {
    savedSignalCount = 0;
    memset(savedSignals, 0, sizeof(savedSignals));
    Serial.println(F("[CC1101] Sinais apagados da memoria."));
}

void cc1101DeleteSignal(uint8_t index) {
    if (index >= savedSignalCount) return;
    for (int i = index; i < savedSignalCount - 1; i++) {
        savedSignals[i] = savedSignals[i + 1];
    }
    memset(&savedSignals[savedSignalCount - 1], 0, sizeof(SignalData));
    savedSignalCount--;
    Serial.println(F("[CC1101] Sinal individual excluido."));
}

// ============================================================
// TRANSMIT RAW
// ============================================================
void cc1101TransmitRaw(uint32_t frequency, uint16_t* timings, uint8_t length) {
    if (!cc1101Initialized || length == 0 || length > 200) return;
    isr_enabled = false;
    cc1101SetFrequency(frequency);
    cc1101WriteReg(CC1101_IOCFG0, 0x2E);
    cc1101SendCommand(CC1101_SIDLE); delay(1);
    cc1101SendCommand(CC1101_STX); delay(1);
    pinMode(CC1101_GDO0, OUTPUT);
    for (int i = 0; i < length; i++) {
        digitalWrite(CC1101_GDO0, i % 2 == 0 ? HIGH : LOW);
        delayMicroseconds(timings[i]);
    }
    digitalWrite(CC1101_GDO0, LOW);
    pinMode(CC1101_GDO0, INPUT_PULLUP);
    cc1101WriteReg(CC1101_IOCFG0, 0x0D);
    cc1101WriteReg(CC1101_IOCFG2, 0x0D);
    cc1101SendCommand(CC1101_SIDLE);
    cc1101SendCommand(CC1101_SRX);
}

// ============================================================
// HELPERS DE STATUS
// ============================================================
bool cc1101IsAvailable() { return cc1101Initialized; }
bool cc1101IsCapturing() { return cc1101CopyActive; }
uint8_t cc1101GetSavedCount() { return savedSignalCount; }
SignalData* cc1101GetSignal(uint8_t index) {
    if (index < savedSignalCount) return &savedSignals[index];
    return nullptr;
}

// ============================================================
// DRONE RSSI
// ============================================================
int8_t cc1101GetDroneRSSI() {
    if (!cc1101Initialized) return 0;
    int maxRssiDbm = -100;
    int persistentHits = 0;
    for (int freq = 0; freq < 2; freq++) {
        cc1101SendCommand(CC1101_SIDLE);
        if (freq == 0) cc1101SetFrequency(868000000);
        else           cc1101SetFrequency(915000000);
        cc1101SendCommand(CC1101_SRX);
        delayMicroseconds(500);
        for (int i = 0; i < 3; i++) {
            uint8_t rssiDec = cc1101ReadStatus(CC1101_RSSI);
            int rssi = (rssiDec >= 128) ? ((int)rssiDec - 256) / 2 - 74 : (int)rssiDec / 2 - 74;
            if (rssi > maxRssiDbm) maxRssiDbm = rssi;
            if (rssi > -70) persistentHits++;
            delay(5);
        }
    }
    cc1101SendCommand(CC1101_SIDLE);
    cc1101SendCommand(CC1101_SRX);
    if (persistentHits < 3) return 0;
    if (maxRssiDbm < -65) return 0;
    if (maxRssiDbm > -30) return 100;
    return map(maxRssiDbm, -65, -30, 1, 100);
}
