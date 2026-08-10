#include <SPI.h>
#include "config.h"

// ============================================================
// CC1101 Driver — SPI DEDICADO HSPI com CS MANUAL (v14)
//
// CAUSA RAIZ DOS PROBLEMAS ANTERIORES:
//
// 1. BUS CONFLITO: O global SPI no ESP32 Arduino v2.0.x
//    mapeia para HSPI (SPI2) — MESMO periférico que o CC1101.
//    Quando nrf24.cpp chamava SPI.begin(), destruía a config do CC1101.
//    FIX: CC1101 usa SPIClass(HSPI) próprio, NRF24 usa SPIClass(VSPI).
//
// 2. CS QUEBRADO: cc1101SPI.begin(..., 15) fazia o driver SPI
//    assumir o GPIO 15. Os digitalWrite(CSN) viravam NO-OP.
//    Cada transfer() gerava ciclo CS separado → leituras de registradores
//    de 2+ bytes ficavam quebradas (CS HIGH entre bytes).
//    FIX: begin(..., -1) → driver NÃO gerencia CS.
//    transferBytes() faz transferência atômica multi-byte.
//
// 3. testAllPins() DESTRUIA GPIO MATRIX: pinMode() nos pinos HSPI
//    sobrescrevia a rotação do GPIO matrix.
//    FIX: testAllPins() agora pula pinos SPI.
//
// 4. GoRx FALHAVA MESMO NO BOOT: O estado 0x00 após SRX era causado
//    pelo problema #2 (registradores mal escritos → chip não entrava RX).
// ============================================================

// Registradores — ENDEREÇOS CORRETOS (datasheet SWRS061C)
#define CC1101_IOCFG2   0x00
#define CC1101_IOCFG0   0x02
#define CC1101_FIFOTHR  0x03
#define CC1101_FSCTRL1  0x0B
#define CC1101_FSCTRL0  0x0C
#define CC1101_PKTCTRL1 0x07
#define CC1101_PKTCTRL0 0x08
#define CC1101_ADDR     0x09
#define CC1101_CHANNR   0x0A
#define CC1101_PKTLEN   0x06
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
#define CC1101_WORCTRL  0x20
#define CC1101_FREND1   0x21
#define CC1101_FREND0   0x22
#define CC1101_FSCAL3   0x23
#define CC1101_FSCAL2   0x24
#define CC1101_FSCAL1   0x25
#define CC1101_FSCAL0   0x26
#define CC1101_FSTEST   0x29
#define CC1101_TEST2    0x2C
#define CC1101_TEST1    0x2D
#define CC1101_TEST0    0x2E
#define CC1101_PARTNUM  0x30
#define CC1101_VERSION  0x31
#define CC1101_RSSI     0x34
#define CC1101_MARCSTATE 0x35

#define CC1101_SRES     0x30
#define CC1101_SRX      0x34
#define CC1101_STX      0x35
#define CC1101_SIDLE    0x36
#define CC1101_SPWD     0x39
#define CC1101_SFRX     0x3A
#define CC1101_PATABLE  0x3E

#define CC1101_READ_SINGLE  0x80
#define CC1101_READ_BURST   0xC0
#define CC1101_WRITE_BURST  0x40

// ============================================================
// VARIÁVEIS GLOBAIS
// ============================================================
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

static bool cc1101Awake = false;

// ============================================================
// ISR — captura timestamps de transição do GDO0
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
// SPI — BARRAMENTO DEDICADO HSPI (SPI2) com CS MANUAL
//
// CS=-1 no begin(): O driver SPI NÃO gerencia o pino CS.
// Isso permite digitalWrite(CSN, LOW/HIGH) funcionar corretamente.
//
// transferBytes(): Envia/recebe N bytes em UMA única transação
// de hardware (CS fica LOW o tempo todo). Necessário para
// leitura/escrita de registradores do CC1101 (2+ bytes).
// ============================================================

static SPIClass cc1101SPI(HSPI);
static bool cc1101BusInitialized = false;

bool cc1101NeedsSpiReinit = false;

static void cc1101SpiForceReinit() {
    cc1101SPI.end();
    delay(1);
    // Configura pinos CSN como GPIO manual ANTES do begin()
    // para garantir estado determinístico
    pinMode(CC1101_CSN, OUTPUT);
    digitalWrite(CC1101_CSN, HIGH);
    delay(1);
    // CS=-1: driver NÃO gerencia CS → digitalWrite funciona
    cc1101SPI.begin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, -1);
    cc1101SPI.setFrequency(4000000);
    cc1101BusInitialized = true;
    Serial.println("[CC1101] SPI: HSPI reinicializado (CS manual)");
    Serial.flush();
}

static void cc1101SpiStart() {
    if (!cc1101BusInitialized || cc1101NeedsSpiReinit) {
        cc1101SpiForceReinit();
        cc1101NeedsSpiReinit = false;
    }
}

static void cc1101SpiEnd() {
    // NÃO chama end(). O barramento HSPI permanece inicializado.
}

// Aguarda MISO ir para LOW (CHIP_RDYn). Retorna false se timeout.
static bool waitMisoReady(uint16_t timeoutMs = 100) {
    unsigned long start = millis();
    while (digitalRead(CC1101_MISO) != LOW) {
        if (millis() - start > timeoutMs) {
            Serial.println("[CC1101] WARN: MISO timeout!");
            return false;
        }
        yield();
    }
    return true;
}

// ============================================================
// SPI PRIMITIVES — usam transferBytes() para transações atômicas
//
// ANTES (quebrado):
//   digitalWrite(CSN, LOW);
//   cc1101SPI.transfer(reg | 0x80);  // CS HIGH após esta linha!
//   val = cc1101SPI.transfer(0x00);   // Nova transação CS LOW
//   digitalWrite(CSN, HIGH);
//
// DEPOIS (corrigido):
//   digitalWrite(CSN, LOW);
//   uint8_t tx[2] = {reg | 0x80, 0x00};
//   uint8_t rx[2];
//   cc1101SPI.transferBytes(tx, rx, 2);  // CS fica LOW todo o tempo
//   digitalWrite(CSN, HIGH);
// ============================================================

uint8_t cc1101ReadReg(uint8_t reg) {
    cc1101SpiStart();
    uint8_t tx[2] = {(uint8_t)(reg | CC1101_READ_SINGLE), 0x00};
    uint8_t rx[2] = {0, 0};
    digitalWrite(CC1101_CSN, LOW);
    if (!waitMisoReady()) { digitalWrite(CC1101_CSN, HIGH); return 0x00; }
    cc1101SPI.transferBytes(tx, rx, 2);
    digitalWrite(CC1101_CSN, HIGH);
    cc1101SpiEnd();
    return rx[1];
}

uint8_t cc1101ReadStatus(uint8_t reg) {
    cc1101SpiStart();
    uint8_t tx[2] = {(uint8_t)(reg | CC1101_READ_BURST), 0x00};
    uint8_t rx[2] = {0, 0};
    digitalWrite(CC1101_CSN, LOW);
    if (!waitMisoReady()) { digitalWrite(CC1101_CSN, HIGH); return 0x00; }
    cc1101SPI.transferBytes(tx, rx, 2);
    digitalWrite(CC1101_CSN, HIGH);
    cc1101SpiEnd();
    return rx[1];
}

void cc1101WriteReg(uint8_t reg, uint8_t value) {
    cc1101SpiStart();
    uint8_t tx[2] = {reg, value};
    uint8_t rx[2];
    digitalWrite(CC1101_CSN, LOW);
    if (!waitMisoReady()) { digitalWrite(CC1101_CSN, HIGH); return; }
    cc1101SPI.transferBytes(tx, rx, 2);
    digitalWrite(CC1101_CSN, HIGH);
    cc1101SpiEnd();
}

static bool cc1101WriteRegBurst(uint8_t reg, uint8_t* data, uint8_t len) {
    cc1101SpiStart();
    uint8_t tx[9];
    tx[0] = reg | CC1101_WRITE_BURST;
    for (uint8_t i = 0; i < len && i < 8; i++) tx[i + 1] = data[i];
    uint8_t rx[9];
    digitalWrite(CC1101_CSN, LOW);
    if (!waitMisoReady()) { digitalWrite(CC1101_CSN, HIGH); return false; }
    cc1101SPI.transferBytes(tx, rx, len + 1);
    digitalWrite(CC1101_CSN, HIGH);
    cc1101SpiEnd();
    return true;
}

void cc1101SendCommand(uint8_t cmd) {
    cc1101SpiStart();
    uint8_t rx;
    digitalWrite(CC1101_CSN, LOW);
    if (!waitMisoReady()) { digitalWrite(CC1101_CSN, HIGH); return; }
    cc1101SPI.transfer(&cmd, &rx, 1);
    digitalWrite(CC1101_CSN, HIGH);
    cc1101SpiEnd();
}

// ============================================================
// FREQUÊNCIA
// ============================================================

void cc1101SetFrequency(uint32_t freqHz) {
    uint32_t freqWord = (uint32_t)((freqHz / 26000000.0) * 65536);
    cc1101WriteReg(CC1101_FREQ2, (freqWord >> 16) & 0xFF);
    cc1101WriteReg(CC1101_FREQ1, (freqWord >> 8) & 0xFF);
    cc1101WriteReg(CC1101_FREQ0, freqWord & 0xFF);
}

// ============================================================
// CALIBRAÇÃO POR BANDA — idêntica ao ELECHOUSE Calibrate()
// ============================================================

static void cc1101CalibrateBand(float freqMHz) {
    if (freqMHz >= 300.0f && freqMHz <= 348.0f) {
        int fsctrl0_val = (int)(24.0f + (freqMHz - 300.0f) / (348.0f - 300.0f) * (28.0f - 24.0f));
        cc1101WriteReg(CC1101_FSCTRL0, (uint8_t)fsctrl0_val);
        if (freqMHz < 322.88f) {
            cc1101WriteReg(CC1101_TEST0, 0x0B);
        } else {
            cc1101WriteReg(CC1101_TEST0, 0x09);
            uint8_t s = cc1101ReadReg(CC1101_FSCAL2);
            if (s < 32) cc1101WriteReg(CC1101_FSCAL2, s + 32);
        }
    } else if (freqMHz >= 378.0f && freqMHz <= 464.0f) {
        int fsctrl0_val = (int)(31.0f + (freqMHz - 378.0f) / (464.0f - 378.0f) * (38.0f - 31.0f));
        cc1101WriteReg(CC1101_FSCTRL0, (uint8_t)fsctrl0_val);
        if (freqMHz < 430.5f) {
            cc1101WriteReg(CC1101_TEST0, 0x0B);
        } else {
            cc1101WriteReg(CC1101_TEST0, 0x09);
            uint8_t s = cc1101ReadReg(CC1101_FSCAL2);
            if (s < 32) cc1101WriteReg(CC1101_FSCAL2, s + 32);
        }
    } else if (freqMHz >= 779.0f && freqMHz <= 899.99f) {
        int fsctrl0_val = (int)(65.0f + (freqMHz - 779.0f) / (899.0f - 779.0f) * (76.0f - 65.0f));
        cc1101WriteReg(CC1101_FSCTRL0, (uint8_t)fsctrl0_val);
        if (freqMHz < 861.0f) {
            cc1101WriteReg(CC1101_TEST0, 0x0B);
        } else {
            cc1101WriteReg(CC1101_TEST0, 0x09);
            uint8_t s = cc1101ReadReg(CC1101_FSCAL2);
            if (s < 32) cc1101WriteReg(CC1101_FSCAL2, s + 32);
        }
    } else if (freqMHz >= 900.0f && freqMHz <= 928.0f) {
        int fsctrl0_val = (int)(77.0f + (freqMHz - 900.0f) / (928.0f - 900.0f) * (79.0f - 77.0f));
        cc1101WriteReg(CC1101_FSCTRL0, (uint8_t)fsctrl0_val);
        cc1101WriteReg(CC1101_TEST0, 0x09);
        uint8_t s = cc1101ReadReg(CC1101_FSCAL2);
        if (s < 32) cc1101WriteReg(CC1101_FSCAL2, s + 32);
    }
}

// Seta frequência com calibração por banda
static void cc1101SetFrequencyCalibrated(uint32_t freqHz) {
    float freqMHz = freqHz / 1000000.0f;
    cc1101SendCommand(CC1101_SIDLE);
    delay(1);
    cc1101SetFrequency(freqHz);
    cc1101CalibrateBand(freqMHz);
}

// Forward declarations
static bool cc1101GoRx(uint32_t freqHz);
void cc1101Sleep();
bool cc1101Wake();

// ============================================================
// RESET — sequência do datasheet
// ============================================================
static bool cc1101Reset() {
    cc1101SpiStart();
    // Pulso CSN para acordar o chip do SLEEP
    digitalWrite(CC1101_CSN, HIGH);
    delayMicroseconds(2);
    digitalWrite(CC1101_CSN, LOW);
    delay(1);
    digitalWrite(CC1101_CSN, HIGH);
    delay(1);
    // Envio do comando SRES
    uint8_t tx = CC1101_SRES;
    uint8_t rx;
    digitalWrite(CC1101_CSN, LOW);
    if (!waitMisoReady()) {
        Serial.println("[CC1101] RESET: MISO timeout no wake");
        digitalWrite(CC1101_CSN, HIGH);
        return false;
    }
    cc1101SPI.transfer(&tx, &rx, 1);
    if (!waitMisoReady(200)) {
        Serial.println("[CC1101] RESET: MISO timeout pos-SRES");
        digitalWrite(CC1101_CSN, HIGH);
        return false;
    }
    digitalWrite(CC1101_CSN, HIGH);
    cc1101SpiEnd();
    delay(1);
    return true;
}

// ============================================================
// CONFIGURAÇÃO — idêntica ao ELECHOUSE RegConfigSettings()
// ============================================================
static void cc1101ConfigureRegs() {
    cc1101WriteReg(CC1101_IOCFG2,   0x0D);
    cc1101WriteReg(CC1101_IOCFG0,   0x0D);
    cc1101WriteReg(CC1101_PKTCTRL0, 0x32);
    cc1101WriteReg(CC1101_MDMCFG3,  0x93);
    cc1101WriteReg(CC1101_MDMCFG4,  0x07);
    cc1101WriteReg(CC1101_MDMCFG2,  0x30);
    cc1101WriteReg(CC1101_FREND0,   0x11);
    cc1101WriteReg(CC1101_FSCTRL1,  0x06);
    cc1101WriteReg(CC1101_MDMCFG1,  0x02);
    cc1101WriteReg(CC1101_MDMCFG0,  0xF8);
    cc1101WriteReg(CC1101_CHANNR,   0x00);
    cc1101WriteReg(CC1101_DEVIATN,  0x47);
    cc1101WriteReg(CC1101_FREND1,   0x56);
    cc1101WriteReg(CC1101_MCSM0,    0x38);
    cc1101WriteReg(CC1101_FOCCFG,   0x16);
    cc1101WriteReg(CC1101_BSCFG,    0x1C);
    cc1101WriteReg(CC1101_AGCCTRL2, 0xC7);
    cc1101WriteReg(CC1101_AGCCTRL1, 0x00);
    cc1101WriteReg(CC1101_AGCCTRL0, 0xB2);
    cc1101WriteReg(CC1101_FSCAL3,   0xE9);
    cc1101WriteReg(CC1101_FSCAL2,   0x2A);
    cc1101WriteReg(CC1101_FSCAL1,   0x00);
    cc1101WriteReg(CC1101_FSCAL0,   0x1F);
    cc1101WriteReg(CC1101_FSTEST,   0x59);
    cc1101WriteReg(CC1101_TEST2,    0x81);
    cc1101WriteReg(CC1101_TEST1,    0x35);
    cc1101WriteReg(CC1101_TEST0,    0x09);
    cc1101WriteReg(CC1101_PKTCTRL1, 0x04);
    cc1101WriteReg(CC1101_ADDR,     0x00);
    cc1101WriteReg(CC1101_PKTLEN,   0x00);
    cc1101WriteReg(CC1101_MDMCFG4,  0x27);
    uint8_t paTable[8] = {0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    cc1101WriteRegBurst(CC1101_PATABLE, paTable, 8);
}

// ============================================================
// INIT
// ============================================================
bool cc1101Init() {
    Serial.println("[CC1101] Inicializando...");
    Serial.flush();
    // Configura CSN como GPIO ANTES do begin() — garante controle manual
    pinMode(CC1101_CSN, OUTPUT);
    digitalWrite(CC1101_CSN, HIGH);
    pinMode(CC1101_GDO0, INPUT_PULLUP);
    pinMode(CC1101_GDO2, INPUT);
    delay(10);
    // Inicializa HSPI com CS manual (-1)
    cc1101SpiForceReinit();
    if (!cc1101Reset()) {
        Serial.println("[CC1101] FAIL: reset falhou");
        return false;
    }
    uint8_t partnum = cc1101ReadStatus(CC1101_PARTNUM);
    uint8_t version = cc1101ReadStatus(CC1101_VERSION);
    Serial.printf("[CC1101] PARTNUM=0x%02X VERSION=0x%02X\n", partnum, version);
    Serial.flush();
    if (partnum == 0xFF && version == 0xFF) {
        Serial.println("[CC1101] FAIL: modulo nao responde");
        return false;
    }
    cc1101ConfigureRegs();
    cc1101Initialized = true;
    Serial.println("[CC1101] Configurado com sucesso!");
    Serial.flush();
    // Diagnóstico
    Serial.println("[CC1101] === DIAGNOSTICO ===");
    uint8_t r;
    r = cc1101ReadReg(CC1101_FSCTRL1);  Serial.printf("  FSCTRL1   = 0x%02X %s\n", r, r==0x06?"OK":"FAIL");
    r = cc1101ReadReg(CC1101_FREND1);   Serial.printf("  FREND1    = 0x%02X %s\n", r, r==0x56?"OK":"FAIL");
    r = cc1101ReadReg(CC1101_IOCFG0);   Serial.printf("  IOCFG0    = 0x%02X %s\n", r, r==0x0D?"OK":"FAIL");
    r = cc1101ReadReg(CC1101_PKTCTRL0); Serial.printf("  PKTCTRL0  = 0x%02X %s\n", r, r==0x32?"OK":"FAIL");
    r = cc1101ReadReg(CC1101_MDMCFG4);  Serial.printf("  MDMCFG4   = 0x%02X %s\n", r, r==0x27?"OK":"FAIL");
    r = cc1101ReadReg(CC1101_MDMCFG2);  Serial.printf("  MDMCFG2   = 0x%02X %s\n", r, r==0x30?"OK":"FAIL");
    r = cc1101ReadReg(CC1101_AGCCTRL2); Serial.printf("  AGCCTRL2  = 0x%02X %s\n", r, r==0xC7?"OK":"FAIL");
    r = cc1101ReadStatus(CC1101_MARCSTATE) & 0x1F;
    Serial.printf("  MARCSTATE = 0x%02X (0x01=IDLE)\n", r);
    Serial.println("[CC1101] === FIM DIAGNOSTICO ===");
    Serial.flush();
    cc1101Sleep();
    return true;
}

// ============================================================
// SLEEP / WAKE
// ============================================================
void cc1101Sleep() {
    if (!cc1101Initialized) return;
    isr_enabled = false;
    cc1101SendCommand(CC1101_SIDLE);
    delay(1);
    cc1101SendCommand(CC1101_SPWD);
    cc1101Awake = false;
    Serial.println("[CC1101] Modulo em SLEEP");
}

bool cc1101Wake() {
    if (!cc1101Initialized) return false;
    Serial.println("[CC1101] WAKE: acordando modulo...");
    Serial.flush();
    // Re-inicializa o barramento HSPI a cada wake para garantir
    // que os pinos estejam corretos (proteção contra testAllPins/WiFi)
    cc1101NeedsSpiReinit = true;
    cc1101SpiStart();
    if (!cc1101Reset()) {
        Serial.println("[CC1101] WAKE: reset falhou!");
        cc1101Awake = false;
        return false;
    }
    cc1101ConfigureRegs();
    detachInterrupt(digitalPinToInterrupt(CC1101_GDO0));
    pinMode(CC1101_GDO0, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(CC1101_GDO0), cc1101ISR, CHANGE);
    uint8_t state = cc1101ReadStatus(CC1101_MARCSTATE) & 0x1F;
    uint8_t version = cc1101ReadStatus(CC1101_VERSION);
    Serial.printf("[CC1101] WAKE: MARCSTATE=0x%02X, VERSION=0x%02X\n", state, version);
    if (state != 0x01) {
        cc1101SendCommand(CC1101_SIDLE);
        delay(2);
        state = cc1101ReadStatus(CC1101_MARCSTATE) & 0x1F;
    }
    cc1101Awake = (state == 0x01);
    Serial.printf("[CC1101] WAKE: %s\n", cc1101Awake ? "OK (IDLE)" : "FALHOU");
    Serial.flush();
    return cc1101Awake;
}

// ============================================================
// GoRx — entra em RX com retry e delay seguro (errata SWRZ020)
// ============================================================
static bool cc1101GoRx(uint32_t freqHz) {
    uint8_t state;
    float freqMHz = freqHz / 1000000.0f;

    state = cc1101ReadStatus(CC1101_MARCSTATE) & 0x1F;
    Serial.printf("[CC1101] GoRx S0: state=0x%02X\n", state);

    if (state == 0x00) {
        Serial.println("[CC1101] GoRx: SLEEP, reset...");
        if (!cc1101Reset()) return false;
        cc1101ConfigureRegs();
    }

    cc1101SendCommand(CC1101_SIDLE);
    delay(1);
    state = cc1101ReadStatus(CC1101_MARCSTATE) & 0x1F;
    Serial.printf("[CC1101] GoRx S1: pos-SIDLE=0x%02X\n", state);

    cc1101SetFrequency(freqHz);
    cc1101CalibrateBand(freqMHz);
    state = cc1101ReadStatus(CC1101_MARCSTATE) & 0x1F;
    Serial.printf("[CC1101] GoRx S2: pos-cal=0x%02X\n", state);

    cc1101SendCommand(CC1101_SIDLE);
    delayMicroseconds(100);
    cc1101SendCommand(CC1101_SRX);
    Serial.println("[CC1101] GoRx S3: SRX enviado, aguardando...");

    // Errata SWRZ020: delay >5ms antes de ler MARCSTATE
    delay(5);

    state = cc1101ReadStatus(CC1101_MARCSTATE) & 0x1F;
    Serial.printf("[CC1101] GoRx S4: final=0x%02X @ %lu Hz\n", state, freqHz);

    if (state == 0x0D || state == 0x08) return true;

    // Retry
    Serial.printf("[CC1101] GoRx: tentativa 2 com delay extra...\n");
    delay(10);
    state = cc1101ReadStatus(CC1101_MARCSTATE) & 0x1F;
    Serial.printf("[CC1101] GoRx S5: retry=0x%02X\n", state);
    if (state == 0x0D || state == 0x08) return true;

    // Aceita estados de calibração
    if (state >= 0x02 && state <= 0x0C) {
        Serial.printf("[CC1101] GoRx: estado transicao 0x%02X, aguardando...\n", state);
        delay(10);
        state = cc1101ReadStatus(CC1101_MARCSTATE) & 0x1F;
        Serial.printf("[CC1101] GoRx S6: pos-transicao=0x%02X\n", state);
        if (state == 0x0D || state == 0x08) return true;
    }

    Serial.printf("[CC1101] GoRx FALHOU: state=0x%02X\n", state);
    return false;
}

// ============================================================
// CAPTURE - Copiar Sinal
// ============================================================
void cc1101StartCapture() {
    if (!cc1101Initialized) return;
    if (!cc1101Wake()) {
        Serial.println("[CC1101] CAPTURE: falha ao acordar!");
        return;
    }
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
    bool rxOk = cc1101GoRx(currentCapture.frequency);
    if (!rxOk) {
        Serial.println("[CC1101] WARN: Falha RX, tentando...");
    }
    isr_last_val = digitalRead(CC1101_GDO0);
    isr_last_change = micros();
    isr_enabled = true;
    Serial.printf("[CC1101] Capture @ %lu Hz\n", currentCapture.frequency);
    Serial.flush();
}

void cc1101CaptureLoop() {
    if (!cc1101CopyActive) return;
    unsigned long now = micros();
    unsigned long nowMs = millis();

    if (capture_state == STATE_HOPPING) {
        if (isr_count > 5) {
            capture_state = STATE_LOCKED;
        }
        else if (nowMs - lastFreqSwitch > 1000) {
            isr_enabled = false;
            isr_count = 0;
            capture_started = false;
            currentFreqIndex = (currentFreqIndex + 1) % 4;
            currentCapture.frequency = captureFreqs[currentFreqIndex];
            cc1101GoRx(currentCapture.frequency);
            isr_last_val = digitalRead(CC1101_GDO0);
            isr_last_change = micros();
            isr_enabled = true;
            lastFreqSwitch = nowMs;
        }
    }
    else if (capture_state == STATE_LOCKED) {
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
            isr_enabled = false;
            isr_count = 0;
            capture_started = false;
            capture_state = STATE_HOPPING;
            isr_last_change = micros();
            currentFreqIndex = (currentFreqIndex + 1) % 4;
            currentCapture.frequency = captureFreqs[currentFreqIndex];
            cc1101GoRx(currentCapture.frequency);
            isr_last_val = digitalRead(CC1101_GDO0);
            isr_last_change = micros();
            isr_enabled = true;
            lastFreqSwitch = nowMs;
        }
        else if ((capture_started && silenceTimeout) || bufferFull) {
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
                Serial.printf("[CC1101] Capturado: %d pulsos, %lu us\n", sig->length, totalDuration);
            } else {
                Serial.println("[CC1101] Sem sinal valido");
            }
            Serial.flush();
        }
        else if (totalTimeout) {
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
            cc1101GoRx(currentCapture.frequency);
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
    if (!cc1101Wake()) return;
    isr_enabled = false;
    SignalData* sig = &savedSignals[index];
    cc1101SetFrequencyCalibrated(sig->frequency);
    cc1101WriteReg(CC1101_IOCFG0, 0x2E);
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
    cc1101Sleep();
}

void cc1101SendBruteForceCode(uint32_t code, uint32_t freq) {
    if (!cc1101Initialized) return;
    if (!cc1101Wake()) return;
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
    cc1101Sleep();
}

void cc1101StartSubGHzJammer() {
    if (!cc1101Initialized) return;
    if (!cc1101Wake()) return;
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
    cc1101Sleep();
}

// ============================================================
// ROLLJAM
// ============================================================
void cc1101StartRollJam() {
    if (!cc1101Initialized) return;
    if (!cc1101Wake()) return;
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
            cc1101Sleep();
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
    cc1101Sleep();
}

// ============================================================
// ANALISADOR DE ESPECTRO
// ============================================================
void cc1101StartAnalyzer() {
    if (!cc1101Initialized) return;
    if (!cc1101Wake()) return;
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
    cc1101Sleep();
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
    Serial.println("[CC1101] Sinais apagados.");
}

void cc1101DeleteSignal(uint8_t index) {
    if (index >= savedSignalCount) return;
    for (int i = index; i < savedSignalCount - 1; i++) {
        savedSignals[i] = savedSignals[i + 1];
    }
    memset(&savedSignals[savedSignalCount - 1], 0, sizeof(SignalData));
    savedSignalCount--;
}

// ============================================================
// TRANSMIT RAW
// ============================================================
void cc1101TransmitRaw(uint32_t frequency, uint16_t* timings, uint8_t length) {
    if (!cc1101Initialized || length == 0 || length > 200) return;
    if (!cc1101Wake()) return;
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
    cc1101Sleep();
}

bool cc1101IsAvailable() { return cc1101Initialized; }
bool cc1101IsCapturing() { return cc1101CopyActive; }
uint8_t cc1101GetSavedCount() { return savedSignalCount; }
SignalData* cc1101GetSignal(uint8_t index) { if (index < savedSignalCount) return &savedSignals[index]; return nullptr; }

int8_t cc1101GetDroneRSSI() {
    if (!cc1101Initialized) return 0;
    if (!cc1101Wake()) return 0;
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
    cc1101Sleep();
    if (persistentHits < 2) return 0;
    if (maxRssiDbm < -70) return 0;
    if (maxRssiDbm > -30) return 100;
    return map(maxRssiDbm, -70, -30, 1, 100);
}
