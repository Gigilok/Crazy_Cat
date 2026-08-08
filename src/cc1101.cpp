#include <SPI.h>
#include "config.h"

// ============================================================
// CC1101 - Driver custom (baseado no padrão ELECHOUSE/DIV/Flipper)
// 
// REFERÊNCIAS:
//   - ELECHOUSE_cc1101 (SmartRC-CC1101-Driver-Lib) V3.0.2
//   - Flipper Zero firmware (subghz_device_cc1101_preset_ook_270khz_async)
//   - DIV (Cifertech) — usa ELECHOUSE internamente
//
// PADRÃO SPI CORRETO (usado por TODOS os projetos funcionais):
//   1. beginTransaction()
//   2. CSN LOW
//   3. while(digitalRead(MISO)) { timeout }   <-- espera CHIP_RDYn
//   4. SPI.transfer(...)
//   5. CSN HIGH (mínimo 40µs = t_sp do datasheet TI)
//   6. endTransaction()
//
// MODO SPI: MODE 0 (CPOL=0, CPHA=0), MSBFIRST
// CLOCK:   4 MHz (ELECHOUSE/DIV) a 8 MHz (Flipper Zero)
//
// HISTÓRICO DE BUGS CORRIGIDOS:
//   - waitMisoReady() tinha timeout de 5ms (agora 200ms como ELECHOUSE)
//   - Retorno de waitMisoReady() era ignorado (agora verificado)
//   - SPI a 1MHz (agora 4MHz)
//   - FSCTRL1 não era escrito (IF freq errada)
//   - FREND1 não era escrito (front-end analógico errado)
//   - PATABLE todo 0xC0 (OOK sem modulação)
//   - AGCCTRL2/0 com valores subótimos
//   - MDMCFG4 banda muito larga (812kHz → 270kHz)
// ============================================================

SPIClass spiCC1101(HSPI);

// ============================================================
// Registradores do CC1101
// ============================================================
#define CC1101_IOCFG2   0x00
#define CC1101_IOCFG0   0x02
#define CC1101_FIFOTHR  0x03
#define CC1101_FSCTRL1  0x07
#define CC1101_PKTCTRL0 0x08
#define CC1101_PKTCTRL1 0x09
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
#define CC1101_FREND1   0x21
#define CC1101_FREND0   0x22
#define CC1101_FSCAL3   0x23
#define CC1101_FSCAL2   0x24
#define CC1101_FSCAL1   0x25
#define CC1101_FSCAL0   0x26
#define CC1101_FSTEST   0x2B
#define CC1101_TEST2    0x2C
#define CC1101_TEST1    0x2D
#define CC1101_TEST0    0x2E
#define CC1101_PARTNUM  0x30
#define CC1101_VERSION  0x31
#define CC1101_RSSI     0x34
#define CC1101_MARCSTATE 0x35
#define CC1101_WORCTRL  0x20

#define CC1101_SRES     0x30
#define CC1101_SCAL     0x33
#define CC1101_SRX      0x34
#define CC1101_STX      0x35
#define CC1101_SIDLE    0x36
#define CC1101_SFRX     0x3A
#define CC1101_PATABLE  0x3E

#define CC1101_READ_SINGLE  0x80
#define CC1101_READ_BURST   0xC0
#define CC1101_WRITE_BURST  0x40

// ============================================================
// SPI Settings — 4 MHz como ELECHOUSE/DIV
// ============================================================
#define CC1101_SPI_FREQ  4000000  // 4 MHz (ELECHOUSE padrão)
#define CC1101_SPI_SETTINGS  SPISettings(CC1101_SPI_FREQ, MSBFIRST, SPI_MODE0)

bool cc1101Initialized = false;
bool cc1101CopyActive = false;
uint8_t rj_state = 0;
unsigned long rj_timer = 0;

extern unsigned long captureStartTime;

uint32_t captureFreqs[] = {433920000, 315000000, 868000000, 915000000};
uint8_t currentFreqIndex = 0;
unsigned long lastFreqSwitch = 0;

#define STATE_HOPPING   0
#define STATE_LOCKED    1
#define STATE_CAPTURING 2
uint8_t capture_state = STATE_HOPPING;

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

volatile uint16_t isr_timings[200];
volatile uint8_t isr_count = 0;
volatile unsigned long isr_last_change = 0;
volatile uint8_t isr_last_val = 0;
volatile bool capture_started = false;
volatile bool isr_enabled = false;

uint16_t spec_an_values[64];
uint32_t spec_an_freqs[64];
uint8_t spec_an_idx = 0;
bool spec_an_running = false;

// ============================================================
// ISR - captura timestamps de transição do GDO0
// Filtra pulsos < 100us (ruído) e > 100ms (silêncio)
// ============================================================
void IRAM_ATTR cc1101ISR() {
    if (!isr_enabled) return;
    unsigned long now = micros();
    uint8_t val = digitalRead(CC1101_GDO0);
    if (val != isr_last_val) {
        unsigned long dt = now - isr_last_change;
        if (dt > 100 && dt < 100000) {
            if (isr_count < 200) {
                isr_timings[isr_count] = dt;
                isr_count++;
            }
        }
        isr_last_val = val;
        isr_last_change = now;
        capture_started = true;
    }
}

// ============================================================
// SPI HELPERS — padrão ELECHOUSE/Flipper Zero
// ============================================================

// Espera MISO ir para LOW (CHIP_RDYn ativo-LOW).
// Após CSN falling edge, MISO vai HIGH (chip waking), depois LOW (pronto).
// Timeout de 200ms — igual ELECHOUSE (Flipper usa 250ms).
// Retorna true se chip ficou pronto, false se timeout.
static bool waitMisoReady() {
    uint32_t start = millis();
    while (digitalRead(CC1101_MISO) != LOW) {
        if (millis() - start > 200) return false;  // 200ms como ELECHOUSE
    }
    return true;
}

// Select = CSN LOW + espera chip ready (CHIP_RDYn LOW)
// Retorna true se o chip ficou pronto para comunicação.
static bool cc1101Select() {
    digitalWrite(CC1101_CSN, LOW);
    bool ready = waitMisoReady();
    if (!ready) {
        // Chip não ficou pronto — desmarca CSN e reporta falha
        digitalWrite(CC1101_CSN, HIGH);
        Serial.println("[CC1101] WARN: MISO timeout no cc1101Select()");
    }
    return ready;
}

// Deselect = CSN HIGH com tempo mínimo de 40µs (t_sp datasheet TI)
static inline void cc1101Deselect() {
    digitalWrite(CC1101_CSN, HIGH);
    // t_sp mínimo = 40µs entre transações (datasheet SWRS061F)
    delayMicroseconds(50);
}

// ============================================================
// SPI PRIMITIVES — verificam chip ready antes de transferir
// ============================================================

static inline void cc1101SpiStart() {
    spiCC1101.beginTransaction(CC1101_SPI_SETTINGS);
}

static inline void cc1101SpiEnd() {
    spiCC1101.endTransaction();
}

uint8_t cc1101ReadReg(uint8_t reg) {
    uint8_t val = 0xFF;
    cc1101SpiStart();
    if (cc1101Select()) {
        spiCC1101.transfer(reg | CC1101_READ_SINGLE);
        val = spiCC1101.transfer(0x00);
        cc1101Deselect();
    }
    cc1101SpiEnd();
    return val;
}

uint8_t cc1101ReadStatus(uint8_t reg) {
    uint8_t val = 0xFF;
    cc1101SpiStart();
    if (cc1101Select()) {
        spiCC1101.transfer(reg | CC1101_READ_BURST);
        val = spiCC1101.transfer(0x00);
        cc1101Deselect();
    }
    cc1101SpiEnd();
    return val;
}

void cc1101WriteReg(uint8_t reg, uint8_t value) {
    cc1101SpiStart();
    if (cc1101Select()) {
        spiCC1101.transfer(reg);
        spiCC1101.transfer(value);
        cc1101Deselect();
    } else {
        Serial.printf("[CC1101] WARN: write reg 0x%02X falhou (MISO timeout)\n", reg);
    }
    cc1101SpiEnd();
}

static bool cc1101WriteRegBurst(uint8_t reg, uint8_t* data, uint8_t len) {
    cc1101SpiStart();
    if (cc1101Select()) {
        spiCC1101.transfer(reg | CC1101_WRITE_BURST);
        for (uint8_t i = 0; i < len; i++) {
            spiCC1101.transfer(data[i]);
        }
        cc1101Deselect();
        cc1101SpiEnd();
        return true;
    }
    cc1101SpiEnd();
    return false;
}

void cc1101SendCommand(uint8_t cmd) {
    cc1101SpiStart();
    if (cc1101Select()) {
        spiCC1101.transfer(cmd);
        cc1101Deselect();
    } else {
        Serial.printf("[CC1101] WARN: cmd 0x%02X falhou (MISO timeout)\n", cmd);
    }
    cc1101SpiEnd();
}

void cc1101SetFrequency(uint32_t freqHz) {
    uint32_t freqWord = (uint32_t)((freqHz / 26000000.0) * 65536);
    cc1101WriteReg(CC1101_FREQ2, (freqWord >> 16) & 0xFF);
    cc1101WriteReg(CC1101_FREQ1, (freqWord >> 8) & 0xFF);
    cc1101WriteReg(CC1101_FREQ0, freqWord & 0xFF);
}

// Seta frequência e recalibra o VCO (CC1101_SCAL)
// Sequência do datasheet TI: SIDLE → set freq → SCAL
static void cc1101SetFrequencyCalibrated(uint32_t freqHz) {
    cc1101SendCommand(CC1101_SIDLE);
    delay(1);
    cc1101SetFrequency(freqHz);
    cc1101SendCommand(CC1101_SCAL);
    delay(2);  // SCAL demora ~720us
}

// ============================================================
// RESET — sequência ELECHOUSE (testada em milhares de dispositivos)
// ============================================================
static bool cc1101Reset() {
    // Pulso CSN para reset do SPI bus do CC1101
    digitalWrite(CC1101_CSN, LOW);
    delayMicroseconds(10);
    digitalWrite(CC1101_CSN, HIGH);
    delayMicroseconds(40);   // t_sp = 40µs mínimo (datasheet)
    digitalWrite(CC1101_CSN, LOW);

    // Espera MISO LOW antes de enviar SRES
    if (!waitMisoReady()) {
        Serial.println("[CC1101] RESET: MISO timeout antes do SRES");
        return false;
    }

    // Envia comando SRES (Software Reset)
    spiCC1101.beginTransaction(CC1101_SPI_SETTINGS);
    spiCC1101.transfer(CC1101_SRES);
    spiCC1101.endTransaction();
    digitalWrite(CC1101_CSN, HIGH);

    // Espera MISO LOW novamente (reset em andamento)
    if (!waitMisoReady()) {
        Serial.println("[CC1101] RESET: MISO timeout apos SRES");
        return false;
    }

    digitalWrite(CC1101_CSN, HIGH);
    delay(1);  // Tempo extra para estabilização pós-reset
    return true;
}

// ============================================================
// INIT — configuração dos registradores
// Valores baseados em Flipper Zero (ook_270khz_async) + ELECHOUSE
// ============================================================
bool cc1101Init() {
    Serial.println("[CC1101] Inicializando...");
    Serial.flush();

    // Inicializa HSPI nos pinos do CC1101 (separado do VSPI do NRF24)
    spiCC1101.begin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CSN);
    pinMode(CC1101_CSN, OUTPUT);
    digitalWrite(CC1101_CSN, HIGH);
    pinMode(CC1101_GDO0, INPUT_PULLUP);
    pinMode(CC1101_GDO2, INPUT);
    delay(10);

    // Reset do CC1101 (sequência ELECHOUSE)
    if (!cc1101Reset()) {
        Serial.println("[CC1101] FAIL: reset falhou");
        return false;
    }

    // Lê PARTNUM e VERSION para confirmar que o chip responde
    uint8_t partnum = cc1101ReadStatus(CC1101_PARTNUM);
    uint8_t version = cc1101ReadStatus(CC1101_VERSION);
    Serial.printf("[CC1101] PARTNUM=0x%02X VERSION=0x%02X\n", partnum, version);
    Serial.flush();

    // CC1101 original TI: PARTNUM=0x00, VERSION=0x04
    // Clone chinês comum:   PARTNUM=0x00, VERSION=0x14 (ou 0x0F)
    if (partnum == 0xFF && version == 0xFF) {
        Serial.println("[CC1101] FAIL: modulo nao responde (PARTNUM=0xFF)");
        Serial.println("[CC1101] Verifique: 1) Fio MISO (GPIO 12) 2) Alimentacao 3.3V 3) Contato CC1101");
        return false;
    }

    // ============================================================
    // REGISTRADORES — OOK/Async para captura e reprodução
    // Valores alinhados com Flipper Zero + ELECHOUSE
    // ============================================================

    // --- GDO0: async serial data output (dados OOK saem direto no GDO0) ---
    cc1101WriteReg(CC1101_IOCFG0,   0x0D);  // GDO0 = async serial output (RX data)

    // --- FIFO: ADC retention + threshold adequado ---
    cc1101WriteReg(CC1101_FIFOTHR,  0x47);  // 0x47 (Flipper): ADC_RETENTION + FIFO threshold

    // --- IF Frequency: OBRIGATÓRIO (faltava no código original!) ---
    // Sem FSCTRL1, a frequência IF do mixer fica em valor padrão de reset (0x00)
    // que pode colocar o sinal fora da faixa do filtro IF, causando surdez total.
    // Valor 0x06 → IF = 152.34375 kHz (usado por ELECHOUSE e Flipper)
    cc1101WriteReg(CC1101_FSCTRL1,  0x06);  // FIX CRÍTICO: faltava no código original

    // --- Modo de pacote: async serial, sem CRC, sem whitening ---
    cc1101WriteReg(CC1101_PKTCTRL0, 0x32);  // Async serial, infinite packet len, no CRC
    cc1101WriteReg(CC1101_PKTCTRL1, 0x04);  // FIX: append status, CRC auto-flush, sem addr check

    // --- Configuração de modulação OOK/ASK ---
    // MDMCFG4: BW = 270.833 kHz (Flipper 0x67) em vez de 812.5kHz (0x17)
    // Banda mais estreita = menos ruído = melhor SNR para captura
    cc1101WriteReg(CC1101_MDMCFG4,  0x67);  // FIX: 270.833 kHz (Flipper) em vez de 812.5kHz
    cc1101WriteReg(CC1101_MDMCFG3,  0x32);  // Data rate ~3.79 kBaud (igual Flipper/ELECHOUSE)
    cc1101WriteReg(CC1101_MDMCFG2,  0x30);  // OOK/ASK, no Manchester, no preamble sync
    cc1101WriteReg(CC1101_MDMCFG1,  0x00);  // No FEC, 0 preamble bytes
    cc1101WriteReg(CC1101_MDMCFG0,  0x00);  // Channel spacing = 25kHz

    // --- Deviation (irrelevante para OOK, mas registrado) ---
    cc1101WriteReg(CC1101_DEVIATN,  0x15);

    // --- Controle de máquina de estados ---
    cc1101WriteReg(CC1101_MCSM0,    0x18);  // Auto-cal quando vai IDLE→RX/TX

    // --- Frequency offset compensation ---
    cc1101WriteReg(CC1101_FOCCFG,   0x16);  // Valor ELECHOUSE

    // --- Bit synchronization ---
    cc1101WriteReg(CC1101_BSCFG,    0x1C);  // FIX: faltava (ELECHOUSE padrão)

    // --- AGC: valores do Flipper Zero (testados em milhares de dispositivos) ---
    cc1101WriteReg(CC1101_AGCCTRL2, 0x03);  // FIX: DVGA all, MAX LNA+LNA2, target 24dB
    cc1101WriteReg(CC1101_AGCCTRL1, 0x00);  // LNA decision boundary 0
    cc1101WriteReg(CC1101_AGCCTRL0, 0x40);  // FIX: Low hysteresis, 8 samples, 4dB boundary (Flipper)

    // --- Front-end analógico RX/TX ---
    // FREND1: OBRIGATÓRIO (faltava no código original!)
    // Sem FREND1, o front-end analógico opera com configuração de reset (0x00)
    // que desabilita partes do LNA e mixer do receptor.
    cc1101WriteReg(CC1101_FREND1,   0xB6);  // FIX CRÍTICO: faltava (0xB6 = Flipper OOK)
    cc1101WriteReg(CC1101_FREND0,   0x11);  // PA index 1 para OOK high output

    // --- Calibração do sintetizador de frequência ---
    cc1101WriteReg(CC1101_FSCAL3,   0xE9);  // Valores do datasheet TI
    cc1101WriteReg(CC1101_FSCAL2,   0x2A);
    cc1101WriteReg(CC1101_FSCAL1,   0x00);
    cc1101WriteReg(CC1101_FSCAL0,   0x1F);

    // --- Teste de produção (ELECHOUSE sempre escreve) ---
    cc1101WriteReg(CC1101_FSTEST,   0x59);  // FIX: faltava (ELECHOUSE)

    // --- Test registers ---
    cc1101WriteReg(CC1101_TEST2,    0x81);
    cc1101WriteReg(CC1101_TEST1,    0x35);
    cc1101WriteReg(CC1101_TEST0,    0x09);

    // ============================================================
    // PATABLE — potência para TX
    // FIX CRÍTICO: Para OOK, PA[0]=0x00 (carrier OFF) e PA[1]=0xC0 (carrier ON)
    // Código original escrevia TODOS os 8 bytes como 0xC0, o que
    // eliminava a modulação OOK (0 e 1 produziam mesma potência).
    // Alinhado com Flipper Zero e ELECHOUSE.
    // ============================================================
    uint8_t paTable[8] = {0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    cc1101WriteRegBurst(CC1101_PATABLE, paTable, 8);

    // Anexa ISR no GDO0 (CHANGE para capturar ambas as bordas)
    attachInterrupt(digitalPinToInterrupt(CC1101_GDO0), cc1101ISR, CHANGE);

    cc1101Initialized = true;
    Serial.println("[CC1101] Configurado com sucesso!");
    Serial.flush();

    // === DIAGNÓSTICO: verifica se writes persistiram ===
    Serial.println("[CC1101] === DIAGNOSTICO ===");
    uint8_t fsctrl1_read  = cc1101ReadReg(CC1101_FSCTRL1);
    uint8_t frend1_read   = cc1101ReadReg(CC1101_FREND1);
    uint8_t iocfg0_read   = cc1101ReadReg(CC1101_IOCFG0);
    uint8_t pktctrl0_read = cc1101ReadReg(CC1101_PKTCTRL0);
    uint8_t mdmcfg4_read  = cc1101ReadReg(CC1101_MDMCFG4);
    uint8_t mdmcfg2_read  = cc1101ReadReg(CC1101_MDMCFG2);
    uint8_t agcctrl2_read = cc1101ReadReg(CC1101_AGCCTRL2);
    uint8_t marcstate_read= cc1101ReadStatus(CC1101_MARCSTATE) & 0x1F;
    Serial.printf("  FSCTRL1   = 0x%02X (esperado 0x06) %s\n",
        fsctrl1_read, fsctrl1_read == 0x06 ? "OK" : "FAIL");
    Serial.printf("  FREND1    = 0x%02X (esperado 0xB6) %s\n",
        frend1_read, frend1_read == 0xB6 ? "OK" : "FAIL");
    Serial.printf("  IOCFG0    = 0x%02X (esperado 0x0D) %s\n",
        iocfg0_read, iocfg0_read == 0x0D ? "OK" : "FAIL");
    Serial.printf("  PKTCTRL0  = 0x%02X (esperado 0x32) %s\n",
        pktctrl0_read, pktctrl0_read == 0x32 ? "OK" : "FAIL");
    Serial.printf("  MDMCFG4   = 0x%02X (esperado 0x67) %s\n",
        mdmcfg4_read, mdmcfg4_read == 0x67 ? "OK" : "FAIL");
    Serial.printf("  MDMCFG2   = 0x%02X (esperado 0x30) %s\n",
        mdmcfg2_read, mdmcfg2_read == 0x30 ? "OK" : "FAIL");
    Serial.printf("  AGCCTRL2  = 0x%02X (esperado 0x03) %s\n",
        agcctrl2_read, agcctrl2_read == 0x03 ? "OK" : "FAIL");
    Serial.printf("  MARCSTATE = 0x%02X (0x01=IDLE apos reset)\n", marcstate_read);
    Serial.printf("  GDO0 pin  = %d\n", digitalRead(CC1101_GDO0));
    Serial.println("[CC1101] === FIM DIAGNOSTICO ===");
    Serial.flush();

    return true;
}

// ============================================================
// CAPTURE - Copiar Sinal
// ============================================================
void cc1101StartCapture() {
    if (!cc1101Initialized) return;
    cc1101CopyActive = true;
    currentCapture.count = 0;
    currentCapture.startTime = millis();
    captureStartTime = currentCapture.startTime;
    currentFreqIndex = 0;
    currentCapture.frequency = captureFreqs[currentFreqIndex];
    lastFreqSwitch = millis();
    capture_state = STATE_HOPPING;
    isr_enabled = false;
    isr_count = 0;
    capture_started = false;

    // IOCFG0=0x0D = async serial output (os dados OOK saem direto no GDO0)
    cc1101WriteReg(CC1101_IOCFG0, 0x0D);
    pinMode(CC1101_GDO0, INPUT_PULLUP);

    cc1101SetFrequency(currentCapture.frequency);
    // Sequência do datasheet TI: SIDLE → SCAL → SRX
    cc1101SendCommand(CC1101_SIDLE); delay(2);
    cc1101SendCommand(CC1101_SCAL);  delay(2);
    cc1101SendCommand(CC1101_SRX);   delay(5);

    // Habilita ISR desde o início (HOPPING state)
    isr_last_val = digitalRead(CC1101_GDO0);
    isr_last_change = micros();
    isr_enabled = true;

    Serial.printf("[CC1101] Capture iniciada @ %lu Hz, MARCSTATE=0x%02X\n",
        currentCapture.frequency, cc1101ReadStatus(CC1101_MARCSTATE) & 0x1F);
    Serial.flush();
}

void cc1101CaptureLoop() {
    if (!cc1101CopyActive) return;
    unsigned long now = micros();
    unsigned long nowMs = millis();

    if (capture_state == STATE_HOPPING) {
        // Se ISR capturou >5 transições em pouco tempo, há sinal real
        if (isr_count > 5) {
            capture_state = STATE_LOCKED;
        }
        else if (nowMs - lastFreqSwitch > 1000) {
            // Sem sinal: troca de frequência
            isr_enabled = false;
            isr_count = 0;
            capture_started = false;
            currentFreqIndex = (currentFreqIndex + 1) % 4;
            currentCapture.frequency = captureFreqs[currentFreqIndex];
            cc1101SetFrequency(currentCapture.frequency);
            cc1101SendCommand(CC1101_SIDLE); delay(2);
            cc1101SendCommand(CC1101_SCAL);  delay(2);
            cc1101SendCommand(CC1101_SRX);   delay(5);
            isr_last_val = digitalRead(CC1101_GDO0);
            isr_last_change = micros();
            isr_enabled = true;
            lastFreqSwitch = nowMs;
        }
    }
    else if (capture_state == STATE_LOCKED) {
        // Resetamos o contador e iniciamos a captura "real"
        isr_count = 0;
        isr_last_val = digitalRead(CC1101_GDO0);
        isr_last_change = micros();
        capture_started = false;
        isr_enabled = true;
        capture_state = STATE_CAPTURING;
    }
    else if (capture_state == STATE_CAPTURING) {
        bool silenceTimeout = (capture_started && (now - isr_last_change > 50000));
        bool noiseTimeout = (capture_started && (now - isr_last_change > 5000) && isr_count < 20);
        bool totalTimeout = (nowMs - currentCapture.startTime > CAPTURE_DURATION);
        bool bufferFull = (isr_count >= 200);

        if (noiseTimeout) {
            // Era ruído: volta para hopping
            isr_enabled = false;
            isr_count = 0;
            capture_started = false;
            capture_state = STATE_HOPPING;
            isr_last_change = micros();
            currentFreqIndex = (currentFreqIndex + 1) % 4;
            currentCapture.frequency = captureFreqs[currentFreqIndex];
            cc1101SetFrequency(currentCapture.frequency);
            cc1101SendCommand(CC1101_SIDLE); delay(2);
            cc1101SendCommand(CC1101_SCAL);  delay(2);
            cc1101SendCommand(CC1101_SRX);   delay(5);
            isr_last_val = digitalRead(CC1101_GDO0);
            isr_last_change = micros();
            isr_enabled = true;
            lastFreqSwitch = nowMs;
        }
        else if ((capture_started && silenceTimeout) || bufferFull) {
            // Captura completa
            isr_enabled = false;
            currentCapture.active = false;
            cc1101CopyActive = false;
            cc1101SendCommand(CC1101_SIDLE);

            if (isr_count > 20 && savedSignalCount < MAX_SAVED_SIGNALS) {
                SignalData* sig = &savedSignals[savedSignalCount];
                sig->length = isr_count;
                sig->frequency = currentCapture.frequency;
                sig->modulation = 0;
                sig->valid = true;
                uint32_t totalDuration = 0;
                for (int i = 0; i < sig->length; i++) {
                    sig->timings[i] = isr_timings[i];
                    totalDuration += sig->timings[i];
                }
                if (sig->length < 25 && totalDuration < 30000)
                    snprintf(sig->name, 16, "Sensor %luM", sig->frequency / 1000000);
                else if (sig->length >= 24 && sig->length <= 50)
                    snprintf(sig->name, 16, "Portao %luM", sig->frequency / 1000000);
                else if (sig->length > 50 || totalDuration > 70000)
                    snprintf(sig->name, 16, "Carro %luM", sig->frequency / 1000000);
                else
                    snprintf(sig->name, 16, "Sinal %luM", sig->frequency / 1000000);
                savedSignalCount++;
                Serial.printf("[CC1101] Sinal capturado: %d pulsos, %lu us total\n",
                    sig->length, totalDuration);
            } else {
                Serial.println("[CC1101] Capture terminou sem sinal valido");
            }
            Serial.flush();
        }
        else if (totalTimeout) {
            // Timeout global: recomeça
            isr_enabled = false;
            currentCapture.count = 0;
            currentCapture.startTime = millis();
            captureStartTime = millis();
            isr_count = 0;
            capture_started = false;
            capture_state = STATE_HOPPING;
            currentFreqIndex = 0;
            currentCapture.frequency = captureFreqs[currentFreqIndex];
            lastFreqSwitch = millis();
            cc1101SetFrequency(currentCapture.frequency);
            cc1101SendCommand(CC1101_SIDLE); delay(2);
            cc1101SendCommand(CC1101_SCAL);  delay(2);
            cc1101SendCommand(CC1101_SRX);   delay(5);
            isr_last_val = digitalRead(CC1101_GDO0);
            isr_last_change = micros();
            isr_enabled = true;
        }
    }
}

void cc1101StopCapture() {
    isr_enabled = false;
    cc1101WriteReg(CC1101_IOCFG0, 0x0D);
    cc1101CopyActive = false;
    currentCapture.active = false;
    cc1101SendCommand(CC1101_SIDLE);
}

uint8_t cc1101GetPulseCount() { return isr_count; }
uint32_t cc1101GetCurrentFreq() { return currentCapture.frequency / 1000000; }
uint8_t cc1101GetPinState() { return digitalRead(CC1101_GDO0); }

// ============================================================
// REPLAY
// ============================================================
void cc1101ReplaySignal(uint8_t index) {
    if (index >= savedSignalCount || !savedSignals[index].valid) return;
    if (!cc1101Initialized) return;
    isr_enabled = false;
    SignalData* sig = &savedSignals[index];
    cc1101SetFrequencyCalibrated(sig->frequency);
    cc1101WriteReg(CC1101_IOCFG0, 0x2E);  // GDO0 = output, driven by MCU
    cc1101SendCommand(CC1101_SIDLE); delay(1);
    cc1101SendCommand(CC1101_STX); delay(1);
    pinMode(CC1101_GDO0, OUTPUT);
    digitalWrite(CC1101_GDO0, LOW);
    for (int i = 0; i < sig->length; i++) {
        digitalWrite(CC1101_GDO0, i % 2 == 0 ? HIGH : LOW);
        delayMicroseconds(sig->timings[i]);
    }
    digitalWrite(CC1101_GDO0, LOW);
    pinMode(CC1101_GDO0, INPUT_PULLUP);
    cc1101WriteReg(CC1101_IOCFG0, 0x0D);
    cc1101SendCommand(CC1101_SIDLE);
}

void cc1101SendBruteForceCode(uint32_t code, uint32_t freq) {
    if (!cc1101Initialized) return;
    isr_enabled = false;
    cc1101SetFrequencyCalibrated(freq);
    cc1101WriteReg(CC1101_IOCFG0, 0x2E);
    cc1101SendCommand(CC1101_SIDLE); delay(1);
    cc1101SendCommand(CC1101_STX); delay(1);
    pinMode(CC1101_GDO0, OUTPUT);
    digitalWrite(CC1101_GDO0, LOW);
    for (int rep = 0; rep < 3; rep++) {
        for (int i = 23; i >= 0; i--) {
            bool bit = (code >> i) & 0x01;
            if (bit) {
                digitalWrite(CC1101_GDO0, HIGH); delayMicroseconds(900);
                digitalWrite(CC1101_GDO0, LOW); delayMicroseconds(300);
                digitalWrite(CC1101_GDO0, HIGH); delayMicroseconds(900);
                digitalWrite(CC1101_GDO0, LOW); delayMicroseconds(300);
            } else {
                digitalWrite(CC1101_GDO0, HIGH); delayMicroseconds(300);
                digitalWrite(CC1101_GDO0, LOW); delayMicroseconds(900);
                digitalWrite(CC1101_GDO0, HIGH); delayMicroseconds(300);
                digitalWrite(CC1101_GDO0, LOW); delayMicroseconds(900);
            }
        }
        digitalWrite(CC1101_GDO0, HIGH); delayMicroseconds(300);
        digitalWrite(CC1101_GDO0, LOW); delayMicroseconds(9300);
    }
    digitalWrite(CC1101_GDO0, LOW);
    pinMode(CC1101_GDO0, INPUT_PULLUP);
    cc1101WriteReg(CC1101_IOCFG0, 0x0D);
    cc1101SendCommand(CC1101_SIDLE);
}

void cc1101StartSubGHzJammer() {
    if (!cc1101Initialized) return;
    isr_enabled = false;
    cc1101SetFrequencyCalibrated(433920000);
    cc1101WriteReg(CC1101_IOCFG0, 0x2E);
    cc1101SendCommand(CC1101_SIDLE); delay(1);
    cc1101SendCommand(CC1101_STX); delay(1);
    pinMode(CC1101_GDO0, OUTPUT);
    digitalWrite(CC1101_GDO0, HIGH);
}

void cc1101StopSubGHzJammer() {
    if (!cc1101Initialized) return;
    digitalWrite(CC1101_GDO0, LOW);
    pinMode(CC1101_GDO0, INPUT_PULLUP);
    cc1101WriteReg(CC1101_IOCFG0, 0x0D);
    cc1101SendCommand(CC1101_SIDLE);
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
    cc1101SetFrequencyCalibrated(currentCapture.frequency);
    cc1101WriteReg(CC1101_IOCFG0, 0x0D);
    pinMode(CC1101_GDO0, INPUT_PULLUP);
    cc1101SendCommand(CC1101_SIDLE); delay(1);
    cc1101SendCommand(CC1101_SRX);
    delay(5);
}

void cc1101RollJamLoop() {
    if (!cc1101RollJamActive) return;
    unsigned long now = millis();
    unsigned long nowUs = micros();

    if (rj_state == 0) {
        if (digitalRead(CC1101_GDO0) == HIGH) {
            isr_count = 0;
            isr_last_val = HIGH;
            isr_last_change = nowUs;
            capture_started = true;
            isr_enabled = true;
            rj_state = 1;
            rj_timer = now;
        }
    }
    else if (rj_state == 1) {
        if (now - rj_timer > 200) {
            isr_enabled = false;
            cc1101SendCommand(CC1101_SCAL); delay(1);
            cc1101WriteReg(CC1101_IOCFG0, 0x2E);
            cc1101SendCommand(CC1101_SIDLE); delay(1);
            cc1101SendCommand(CC1101_STX);
            pinMode(CC1101_GDO0, OUTPUT);
            digitalWrite(CC1101_GDO0, HIGH);
            rj_state = 2;
            rj_timer = now;
        }
    }
    else if (rj_state == 2) {
        if (now - rj_timer > 200) {
            digitalWrite(CC1101_GDO0, LOW);
            pinMode(CC1101_GDO0, INPUT_PULLUP);
            cc1101SendCommand(CC1101_SIDLE);
            if (isr_count > 20 && savedSignalCount < MAX_SAVED_SIGNALS) {
                SignalData* sig = &savedSignals[savedSignalCount];
                sig->length = isr_count;
                sig->frequency = currentCapture.frequency;
                sig->modulation = 0;
                sig->valid = true;
                for (int i = 0; i < sig->length; i++) sig->timings[i] = isr_timings[i];
                snprintf(sig->name, 16, "Roubado %luM", sig->frequency / 1000000);
                savedSignalCount++;
            }
            cc1101RollJamActive = false;
        }
    }
}

void cc1101StopRollJam() {
    cc1101RollJamActive = false;
    isr_enabled = false;
    digitalWrite(CC1101_GDO0, LOW);
    pinMode(CC1101_GDO0, INPUT_PULLUP);
    cc1101WriteReg(CC1101_IOCFG0, 0x0D);
    cc1101SendCommand(CC1101_SIDLE);
}

// ============================================================
// ANALISADOR DE ESPECTRO
// ============================================================
void cc1101StartAnalyzer() {
    if (!cc1101Initialized) return;
    spec_an_running = true;
    spec_an_idx = 0;
    for(int i=0; i<15; i++) spec_an_freqs[i] = 300000000 + (i * 3200000);
    for(int i=0; i<16; i++) spec_an_freqs[15+i] = 387000000 + (i * 4800000);
    for(int i=0; i<33; i++) spec_an_freqs[31+i] = 779000000 + (i * 4500000);
    for(int i=0; i<64; i++) spec_an_values[i] = 0;
    cc1101SetFrequencyCalibrated(spec_an_freqs[0]);
}

void cc1101AnalyzerLoop() {
    if (!spec_an_running) return;
    for(int i=0; i<64; i++) {
        if (spec_an_values[i] > 0) spec_an_values[i]--;
    }
    uint32_t freq = spec_an_freqs[spec_an_idx];
    cc1101SetFrequencyCalibrated(freq);
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

void cc1101ClearSavedSignals() {
    savedSignalCount = 0;
    memset(savedSignals, 0, sizeof(savedSignals));
    Serial.println("[CC1101] Sinais apagados da memoria.");
}

void cc1101DeleteSignal(uint8_t index) {
    if (index >= savedSignalCount) return;
    for (int i = index; i < savedSignalCount - 1; i++) {
        savedSignals[i] = savedSignals[i + 1];
    }
    memset(&savedSignals[savedSignalCount - 1], 0, sizeof(SignalData));
    savedSignalCount--;
    Serial.println("[CC1101] Sinal individual excluido.");
}

// ============================================================
// TRANSMIT RAW (Para Termux Keeloq)
// ============================================================
void cc1101TransmitRaw(uint32_t frequency, uint16_t* timings, uint8_t length) {
    if (!cc1101Initialized || length == 0 || length > 200) return;
    isr_enabled = false;
    cc1101SetFrequencyCalibrated(frequency);
    cc1101WriteReg(CC1101_IOCFG0, 0x2E);
    cc1101SendCommand(CC1101_SIDLE); delay(1);
    cc1101SendCommand(CC1101_STX); delay(1);
    pinMode(CC1101_GDO0, OUTPUT);
    digitalWrite(CC1101_GDO0, LOW);
    for (int i = 0; i < length; i++) {
        digitalWrite(CC1101_GDO0, i % 2 == 0 ? HIGH : LOW);
        delayMicroseconds(timings[i]);
    }
    digitalWrite(CC1101_GDO0, LOW);
    pinMode(CC1101_GDO0, INPUT_PULLUP);
    cc1101WriteReg(CC1101_IOCFG0, 0x0D);
    cc1101SendCommand(CC1101_SIDLE);
}

bool cc1101IsAvailable() { return cc1101Initialized; }
bool cc1101IsCapturing() { return cc1101CopyActive; }
uint8_t cc1101GetSavedCount() { return savedSignalCount; }
SignalData* cc1101GetSignal(uint8_t index) { if (index < savedSignalCount) return &savedSignals[index]; return nullptr; }

int8_t cc1101GetDroneRSSI() {
    if (!cc1101Initialized) return 0;
    int maxRssiDbm = -100;
    int persistentHits = 0;
    for(int freq=0; freq<2; freq++) {
        cc1101SendCommand(CC1101_SIDLE);
        if(freq==0) cc1101SetFrequencyCalibrated(868000000);
        else cc1101SetFrequencyCalibrated(915000000);
        cc1101SendCommand(CC1101_SRX);
        delayMicroseconds(500);
        for(int i=0; i<3; i++) {
            uint8_t rssiDec = cc1101ReadStatus(CC1101_RSSI);
            int rssi = (rssiDec >= 128) ? ((int)rssiDec - 256) / 2 - 74 : (int)rssiDec / 2 - 74;
            if (rssi > maxRssiDbm) maxRssiDbm = rssi;
            if (rssi > -75) persistentHits++;
            delay(5);
        }
    }
    cc1101SendCommand(CC1101_SIDLE);
    if (persistentHits < 2) return 0;
    if (maxRssiDbm < -70) return 0;
    if (maxRssiDbm > -30) return 100;
    return map(maxRssiDbm, -70, -30, 1, 100);
}
