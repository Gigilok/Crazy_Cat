#include <SPI.h>
#include "config.h"

// ============================================================
// CC1101 - Driver custom v6
//
// BASE: Versão .fixed (a que funcionava parcialmente com WiFi OFF)
// CORREÇÃO CRÍTICA vs .fixed:
//   portENTER_CRITICAL() sem argumento foi removido no ESP-IDF 4.4+.
//   Substituído por noInterrupts()/interrupts() que é o equivalente
//   correto: DESATIVA interrupções no core atual durante SPI transfer.
//
// POR QUE noInterrupts() funciona e portENTER_CRITICAL(&mutex) NÃO:
//   - portENTER_CRITICAL(&mutex) = spinlock. Bloqueia outras tasks
//     no mesmo core mas NÃO desativa interrupções WiFi DMA.
//   - noInterrupts() = xt_set_interrupt_level(1). DESATIVA todas
//     interrupções mascaráveis no core atual. WiFi DMA não consegue
//     interromper a transferência SPI.
//
// OUTRAS CORREÇÕES vs v5:
//   - v5 adicionou waitMisoReady() que NÃO funciona em operação
//     normal (MISO é data output, não crystal ready indicator)
//   - v5 usou delayMicroseconds() que não yielda — WiFi TX bursts
//     se acumulam e corrompem MOSI por EMI
//   - v5 retry com SIDLE em chip SLEEP — inútil
//   - v5 reset falso-positivo (retorna true com MARCSTATE=0x00)
//   - Todos esses bugs foram removidos. Volta ao padrão .fixed.
//
// MANTIDO do v3/v5:
//   - ISR attach/detach sob demanda (nunca left armada)
//   - Re-init HSPI no wake
//   - Status byte logado (pode ser stale —不影响功能)
// ============================================================

SPIClass spiCC1101(HSPI);
static SPISettings cc1101SPISettings(2000000, MSBFIRST, SPI_MODE0);
static bool spiInitialized = false;

// ============================================================
// ENDEREÇOS — verificados contra datasheet SWRS061C
// ============================================================
#define CC1101_IOCFG2   0x00
#define CC1101_IOCFG0   0x02
#define CC1101_FIFOTHR  0x03
#define CC1101_PKTLEN   0x06
#define CC1101_PKTCTRL1 0x07
#define CC1101_PKTCTRL0  0x08
#define CC1101_ADDR     0x09
#define CC1101_CHANNR   0x0A
#define CC1101_FSCTRL1  0x0B
#define CC1101_FSCTRL0  0x0C
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

// Status registers
#define CC1101_PARTNUM  0x30
#define CC1101_VERSION  0x31
#define CC1101_RSSI     0x34
#define CC1101_MARCSTATE 0x35

// Strobe commands
#define CC1101_SRES     0x30
#define CC1101_SCAL     0x33
#define CC1101_SRX      0x34
#define CC1101_STX      0x35
#define CC1101_SIDLE    0x36
#define CC1101_SPWD     0x39
#define CC1101_SFRX     0x3A
#define CC1101_SNOP     0x3D
#define CC1101_PATABLE  0x3E

// SPI access
#define CC1101_READ_SINGLE  0x80
#define CC1101_READ_BURST   0xC0
#define CC1101_WRITE_BURST  0x40

bool cc1101Initialized = false;
bool cc1101CopyActive = false;
uint8_t rj_state = 0;
unsigned long rj_timer = 0;
bool cc1101RollJamActive = false;

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

// Flag para saber se ISR está realmente attachada
// Evita erro 'gpio_isr_service is not installed' no ESP32
static bool isrActuallyAttached = false;

// ============================================================
// ISR — attach/detach sob demanda, nunca left armada
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
// SPI TRANSPORT v6 — noInterrupts() PROTEGE CONTRA WIFI DMA
//
// PROBLEMA: portENTER_CRITICAL(&mutex) é spinlock que NÃO desativa
// interrupções no ESP32 (dual-core). WiFi DMA interrompe a
// transferência SPI, corrompendo o comando (SRX 0x34 → SPWD 0x39).
//
// SOLUÇÃO: noInterrupts() chama xt_set_interrupt_level(1) que
// DESATIVA todas as interrupções mascaráveis no core atual.
// A transferência SPI completa sem interrupção.
//
// REGRA: Dentro de noInterrupts(), usar APENAS:
//   - delayMicroseconds() (busy-wait, não precisa de scheduler)
//   - digitalRead/Write (acesso direto ao registrador GPIO)
//   - spiCC1101.transfer() (hardware SPI, não precisa de CPU)
// NUNCA usar delay(), Serial.printf(), ou qualquer função que
// dependa de interrupções ou do scheduler.
// ============================================================

// Inicializa SPI. Pode ser chamado múltiplas vezes (re-init).
static void cc1101SpiInit() {
    pinMode(CC1101_SCK, OUTPUT);
    pinMode(CC1101_MISO, INPUT);
    pinMode(CC1101_MOSI, OUTPUT);
    pinMode(CC1101_CSN, OUTPUT);
    digitalWrite(CC1101_CSN, HIGH);
    digitalWrite(CC1101_SCK, LOW);
    digitalWrite(CC1101_MOSI, LOW);
    spiCC1101.begin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CSN);
    spiInitialized = true;
    Serial.println("[CC1101] SPI init: HSPI @ 2MHz MODE0");
}

// Re-inicializa o periférico HSPI. Necessário após longo sleep
// porque o WiFi pode ter alterado registradores do GPIO matrix.
static void cc1101SpiReinit() {
    if (spiInitialized) {
        spiCC1101.end();
        spiInitialized = false;
        delayMicroseconds(10);
    }
    cc1101SpiInit();
}

// Delay fixo após CSN LOW — igual ELECHOUSE/Flipper
// 10µs é suficiente quando o crystal está rodando (IDLE).
static inline void spiCsDelay() {
    delayMicroseconds(10);
}

// Espera o crystal estabilizar após reset (MISO LOW = pronto).
// Usa busy-wait SEM yield — seguro dentro de noInterrupts().
// Timeout 200ms (crystal deve estabilizar em <1ms).
static void waitCrystalReady() {
    unsigned long t0 = millis();
    while (digitalRead(CC1101_MISO) != LOW) {
        if (millis() - t0 > 200) {
            Serial.println("[CC1101] WARN: crystal timeout!");
            return;
        }
        // SEM yield()! Busy-wait apenas.
    }
}

// ============================================================
// SPI PRIMITIVES — noInterrupts() protege contra WiFi DMA
// ============================================================

uint8_t cc1101ReadReg(uint8_t reg) {
    uint8_t val;
    noInterrupts();
    spiCC1101.beginTransaction(cc1101SPISettings);
    digitalWrite(CC1101_CSN, LOW);
    spiCsDelay();
    spiCC1101.transfer(reg | CC1101_READ_SINGLE);
    val = spiCC1101.transfer(0x00);
    digitalWrite(CC1101_CSN, HIGH);
    spiCC1101.endTransaction();
    interrupts();
    return val;
}

uint8_t cc1101ReadStatus(uint8_t reg) {
    uint8_t val;
    noInterrupts();
    spiCC1101.beginTransaction(cc1101SPISettings);
    digitalWrite(CC1101_CSN, LOW);
    spiCsDelay();
    spiCC1101.transfer(reg | CC1101_READ_BURST);
    val = spiCC1101.transfer(0x00);
    digitalWrite(CC1101_CSN, HIGH);
    spiCC1101.endTransaction();
    interrupts();
    return val;
}

void cc1101WriteReg(uint8_t reg, uint8_t value) {
    noInterrupts();
    spiCC1101.beginTransaction(cc1101SPISettings);
    digitalWrite(CC1101_CSN, LOW);
    spiCsDelay();
    spiCC1101.transfer(reg);
    spiCC1101.transfer(value);
    digitalWrite(CC1101_CSN, HIGH);
    spiCC1101.endTransaction();
    interrupts();
}

static bool cc1101WriteRegBurst(uint8_t reg, uint8_t* data, uint8_t len) {
    noInterrupts();
    spiCC1101.beginTransaction(cc1101SPISettings);
    digitalWrite(CC1101_CSN, LOW);
    spiCsDelay();
    spiCC1101.transfer(reg | CC1101_WRITE_BURST);
    for (uint8_t i = 0; i < len; i++) {
        spiCC1101.transfer(data[i]);
    }
    digitalWrite(CC1101_CSN, HIGH);
    spiCC1101.endTransaction();
    interrupts();
    return true;
}

// ============================================================
// COMMAND STROBE — single CSN cycle (padrão ELECHOUSE)
//
// NOTA: O status byte lido pode ser stale (do RX FIFO interno
// do ESP32 HSPI). Isso é APENAS um problema de log — o comando
// É realmente enviado ao CC1101 via MOSI. A função do chip
// depende do que recebe em MOSI, não do que o ESP32 lê em MISO.
// ============================================================
void cc1101SendCommand(uint8_t cmd) {
    uint8_t status;
    noInterrupts();
    spiCC1101.beginTransaction(cc1101SPISettings);
    digitalWrite(CC1101_CSN, LOW);
    spiCsDelay();
    status = spiCC1101.transfer(cmd);
    digitalWrite(CC1101_CSN, HIGH);
    spiCC1101.endTransaction();
    interrupts();
    // Log — fora da seção crítica (interrupções habilitadas)
    uint8_t chipState = (status >> 4) & 0x07;
    uint8_t chipReady = (status >> 7) & 0x01;
    const char* stateNames[] = {"IDLE","RX","TX","FSTXON","CAL","SETTLE","RX_OVF","TX_UNF"};
    Serial.printf("[CC1101] CMD 0x%02X: status=0x%02X (RDY=%d STATE=%s)\n",
        cmd, status, chipReady, (chipState <= 7) ? stateNames[chipState] : "?");
}

void cc1101SetFrequency(uint32_t freqHz) {
    uint32_t freqWord = (uint32_t)((freqHz / 26000000.0) * 65536);
    cc1101WriteReg(CC1101_FREQ2, (freqWord >> 16) & 0xFF);
    cc1101WriteReg(CC1101_FREQ1, (freqWord >> 8) & 0xFF);
    cc1101WriteReg(CC1101_FREQ0, freqWord & 0xFF);
}

// ============================================================
// RESET — sequência TI datasheet + ELECHOUSE
// Usa noInterrupts() para proteção contra WiFi DMA
// ============================================================
static bool cc1101Reset() {
    Serial.println("[CC1101] RESET: iniciando...");
    Serial.flush();

    // Re-init do HSPI para garantir estado limpo
    cc1101SpiReinit();

    // Sequência de reset do datasheet TI:
    // 1. CSN LOW, espera ≥40µs, CSN HIGH, espera ≥10µs
    // 2. CSN LOW, espera MISO LOW (crystal ok)
    // 3. Envia SRES
    // 4. Espera MISO LOW (crystal ok após reset)
    // 5. CSN HIGH
    noInterrupts();
    spiCC1101.beginTransaction(cc1101SPISettings);

    digitalWrite(CC1101_CSN, LOW);
    delayMicroseconds(50);   // ≥40µs
    digitalWrite(CC1101_CSN, HIGH);
    delayMicroseconds(20);   // ≥10µs
    digitalWrite(CC1101_CSN, LOW);
    waitCrystalReady();      // espera crystal (MISO LOW)
    spiCC1101.transfer(CC1101_SRES);
    waitCrystalReady();      // espera crystal após reset

    digitalWrite(CC1101_CSN, HIGH);
    spiCC1101.endTransaction();
    interrupts();

    // Datasheet: após SRES, espera ≥150µs para chip estabilizar
    delay(2);  // 2ms = bem acima do mínimo

    // Verifica se chip responde
    uint8_t pn = cc1101ReadStatus(CC1101_PARTNUM);
    Serial.printf("[CC1101] RESET: PARTNUM=0x%02X\n", pn);

    if (pn == 0xFF) {
        Serial.println("[CC1101] RESET: chip nao responde");
        return false;
    }

    // Verifica estado — deve estar em IDLE (0x01)
    uint8_t ms = cc1101ReadStatus(CC1101_MARCSTATE) & 0x1F;
    Serial.printf("[CC1101] RESET: MARCSTATE=0x%02X\n", ms);

    if (ms != 0x01) {
        Serial.printf("[CC1101] RESET: estado inesperado, enviando SIDLE...\n");
        noInterrupts();
        spiCC1101.beginTransaction(cc1101SPISettings);
        digitalWrite(CC1101_CSN, LOW);
        spiCsDelay();
        spiCC1101.transfer(CC1101_SIDLE);
        digitalWrite(CC1101_CSN, HIGH);
        spiCC1101.endTransaction();
        interrupts();
        delay(1);
        ms = cc1101ReadStatus(CC1101_MARCSTATE) & 0x1F;
        Serial.printf("[CC1101] RESET: apos SIDLE, MARCSTATE=0x%02X\n", ms);
        
        // Se ainda não está em IDLE, o reset falhou
        if (ms != 0x01) {
            Serial.printf("[CC1101] RESET: FALHOU - chip nao vai para IDLE (0x%02X)\n", ms);
            return false;
        }
    }

    Serial.println("[CC1101] RESET: OK");
    return true;
}

// ============================================================
// CONFIGURAÇÃO — ELECHOUSE + MCSM0 com FS_AUTOCAL=1
// MCSM0=0x38: FS_AUTOCAL=1 (calibra ao entrar RX/TX)
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
    // MCSM0 = 0x38: FS_AUTOCAL=1 (calibra ao entrar RX/TX)
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

    // Verifica MCSM0
    uint8_t mcsm0 = cc1101ReadReg(CC1101_MCSM0);
    Serial.printf("[CC1101] CONFIG: MCSM0=0x%02X (esperado 0x38)\n", mcsm0);
}

// ============================================================
// CALIBRAÇÃO POR BANDA
// ============================================================
static void cc1101CalibrateBand(float freqMHz) {
    if (freqMHz >= 300.0f && freqMHz <= 348.0f) {
        int v = (int)(24.0f + (freqMHz - 300.0f) / (348.0f - 300.0f) * 4.0f);
        cc1101WriteReg(CC1101_FSCTRL0, (uint8_t)v);
        if (freqMHz < 322.88f) {
            cc1101WriteReg(CC1101_TEST0, 0x0B);
        } else {
            cc1101WriteReg(CC1101_TEST0, 0x09);
            uint8_t s = cc1101ReadReg(CC1101_FSCAL2);
            if (s < 32) cc1101WriteReg(CC1101_FSCAL2, s + 32);
        }
    } else if (freqMHz >= 378.0f && freqMHz <= 464.0f) {
        int v = (int)(31.0f + (freqMHz - 378.0f) / (464.0f - 378.0f) * 7.0f);
        cc1101WriteReg(CC1101_FSCTRL0, (uint8_t)v);
        if (freqMHz < 430.5f) {
            cc1101WriteReg(CC1101_TEST0, 0x0B);
        } else {
            cc1101WriteReg(CC1101_TEST0, 0x09);
            uint8_t s = cc1101ReadReg(CC1101_FSCAL2);
            if (s < 32) cc1101WriteReg(CC1101_FSCAL2, s + 32);
        }
    } else if (freqMHz >= 779.0f && freqMHz <= 899.99f) {
        int v = (int)(65.0f + (freqMHz - 779.0f) / (899.0f - 779.0f) * 11.0f);
        cc1101WriteReg(CC1101_FSCTRL0, (uint8_t)v);
        if (freqMHz < 861.0f) {
            cc1101WriteReg(CC1101_TEST0, 0x0B);
        } else {
            cc1101WriteReg(CC1101_TEST0, 0x09);
            uint8_t s = cc1101ReadReg(CC1101_FSCAL2);
            if (s < 32) cc1101WriteReg(CC1101_FSCAL2, s + 32);
        }
    } else if (freqMHz >= 900.0f && freqMHz <= 928.0f) {
        int v = (int)(77.0f + (freqMHz - 900.0f) / (928.0f - 900.0f) * 2.0f);
        cc1101WriteReg(CC1101_FSCTRL0, (uint8_t)v);
        cc1101WriteReg(CC1101_TEST0, 0x09);
        uint8_t s = cc1101ReadReg(CC1101_FSCAL2);
        if (s < 32) cc1101WriteReg(CC1101_FSCAL2, s + 32);
    }
}

static uint8_t readMarcState() {
    return cc1101ReadStatus(CC1101_MARCSTATE) & 0x1F;
}

// ============================================================
// Attach/detach ISR — só quando necessário
// ============================================================
static void cc1101AttachISR() {
    if (!isrActuallyAttached) {
        attachInterrupt(digitalPinToInterrupt(CC1101_GDO0), cc1101ISR, CHANGE);
        isrActuallyAttached = true;
    }
}

static void cc1101DetachISR() {
    if (isrActuallyAttached) {
        detachInterrupt(digitalPinToInterrupt(CC1101_GDO0));
        isrActuallyAttached = false;
    }
}

// ============================================================
// GoRx — Entrar em modo RX com calibração automática
// Baseado no .fixed que funcionava parcialmente.
// Usa delay() (não delayMicroseconds()) para permitir que
// WiFi complete TX bursts entre operações SPI.
// ============================================================
static bool cc1101GoRx(uint32_t freqHz) {
    float freqMHz = freqHz / 1000000.0f;
    uint8_t state = readMarcState();
    Serial.printf("[CC1101] GoRx: estado inicial=0x%02X\n", state);

    if (state == 0x00) {
        Serial.println("[CC1101] GoRx: chip em SLEEP, resetando...");
        cc1101DetachISR();
        if (!cc1101Reset()) return false;
        cc1101ConfigureRegs();
    }

    cc1101DetachISR();

    // IDLE → set frequency → calibrate → IDLE → RX
    cc1101SendCommand(CC1101_SIDLE);
    delay(1);

    cc1101SetFrequency(freqHz);
    cc1101CalibrateBand(freqMHz);

    cc1101SendCommand(CC1101_SIDLE);
    delay(1);

    // Agora envia SRX
    cc1101SendCommand(CC1101_SRX);

    // Espera calibração automática + settling
    // delay() yielda para o scheduler, permitindo WiFi TX
    // O SRX já foi enviado com noInterrupts(), então está seguro
    delay(10);

    state = readMarcState();
    uint8_t raw = cc1101ReadStatus(CC1101_MARCSTATE);
    Serial.printf("[CC1101] GoRx: estado final=0x%02X (raw=0x%02X) @ %lu Hz\n", state, raw, freqHz);

    if (state == 0x0D || state == 0x0E || state == 0x08 || state == 0x06 || state == 0x0B) {
        Serial.printf("[CC1101] GoRx: SUCESSO (state=0x%02X)\n", state);
        return true;
    }

    // Se falhou, tenta mais uma vez com delay maior
    Serial.printf("[CC1101] GoRx: primeira tentativa falhou (0x%02X), retry...\n", state);
    cc1101SendCommand(CC1101_SIDLE);
    delay(5);
    cc1101SendCommand(CC1101_SRX);
    delay(20);

    state = readMarcState();
    Serial.printf("[CC1101] GoRx: retry state=0x%02X\n", state);

    if (state == 0x0D || state == 0x0E || state == 0x08 || state == 0x06 || state == 0x0B) {
        Serial.printf("[CC1101] GoRx: SUCESSO no retry (state=0x%02X)\n", state);
        return true;
    }

    Serial.printf("[CC1101] GoRx: FALHOU, state=0x%02X\n", state);
    return false;
}

static void cc1101SetFrequencyCalibrated(uint32_t freqHz) {
    float freqMHz = freqHz / 1000000.0f;
    cc1101SendCommand(CC1101_SIDLE);
    delay(1);
    cc1101SetFrequency(freqHz);
    cc1101CalibrateBand(freqMHz);
}

// ============================================================
// SLEEP / WAKE / INIT
// ============================================================
void cc1101Sleep() {
    if (!cc1101Initialized) return;
    cc1101DetachISR();
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

    cc1101DetachISR();
    isr_enabled = false;

    if (!cc1101Reset()) {
        Serial.println("[CC1101] WAKE: reset falhou!");
        cc1101Awake = false;
        return false;
    }

    cc1101ConfigureRegs();

    uint8_t state = readMarcState();
    uint8_t version = cc1101ReadStatus(CC1101_VERSION);
    Serial.printf("[CC1101] WAKE: MARCSTATE=0x%02X, VERSION=0x%02X\n", state, version);

    if (state != 0x01) {
        cc1101SendCommand(CC1101_SIDLE);
        delay(2);
        state = readMarcState();
        Serial.printf("[CC1101] WAKE: apos SIDLE, state=0x%02X\n", state);
    }

    cc1101Awake = (state == 0x01);
    if (cc1101Awake) {
        Serial.println("[CC1101] WAKE: modulo pronto (IDLE)");
    } else {
        Serial.println("[CC1101] WAKE: FALHOU");
    }
    Serial.flush();
    return cc1101Awake;
}

bool cc1101Init() {
    Serial.println("[CC1101] Inicializando...");
    Serial.flush();

    pinMode(CC1101_CSN, OUTPUT);
    digitalWrite(CC1101_CSN, HIGH);
    pinMode(CC1101_GDO0, INPUT);
    pinMode(CC1101_GDO2, INPUT);
    delay(10);

    cc1101SpiInit();

    if (!cc1101Reset()) {
        Serial.println("[CC1101] FAIL: reset falhou");
        return false;
    }

    uint8_t partnum = cc1101ReadStatus(CC1101_PARTNUM);
    uint8_t version = cc1101ReadStatus(CC1101_VERSION);
    Serial.printf("[CC1101] PARTNUM=0x%02X VERSION=0x%02X\n", partnum, version);
    Serial.flush();

    if (partnum == 0xFF && version == 0xFF) {
        Serial.println("[CC1101] FAIL: modulo nao responde (0xFF)");
        return false;
    }

    cc1101ConfigureRegs();

    cc1101Initialized = true;
    Serial.println("[CC1101] Configurado com sucesso!");
    Serial.flush();

    Serial.println("[CC1101] === DIAGNOSTICO ===");
    uint8_t r;
    r = cc1101ReadReg(CC1101_FSCTRL1);  Serial.printf("  FSCTRL1   = 0x%02X %s\n", r, r==0x06?"OK":"FAIL");
    r = cc1101ReadReg(CC1101_FREND1);   Serial.printf("  FREND1    = 0x%02X %s\n", r, r==0x56?"OK":"FAIL");
    r = cc1101ReadReg(CC1101_IOCFG0);   Serial.printf("  IOCFG0    = 0x%02X %s\n", r, r==0x0D?"OK":"FAIL");
    r = cc1101ReadReg(CC1101_PKTCTRL0); Serial.printf("  PKTCTRL0  = 0x%02X %s\n", r, r==0x32?"OK":"FAIL");
    r = cc1101ReadReg(CC1101_MDMCFG4);  Serial.printf("  MDMCFG4   = 0x%02X %s\n", r, r==0x27?"OK":"FAIL");
    r = cc1101ReadReg(CC1101_MDMCFG2);  Serial.printf("  MDMCFG2   = 0x%02X %s\n", r, r==0x30?"OK":"FAIL");
    r = cc1101ReadReg(CC1101_AGCCTRL2); Serial.printf("  AGCCTRL2  = 0x%02X %s\n", r, r==0xC7?"OK":"FAIL");
    r = cc1101ReadReg(CC1101_MCSM0);    Serial.printf("  MCSM0     = 0x%02X %s\n", r, r==0x38?"OK":"FAIL");
    r = cc1101ReadStatus(CC1101_MARCSTATE) & 0x1F;
    Serial.printf("  MARCSTATE = 0x%02X (0x01=IDLE)\n", r);
    Serial.printf("  GDO0 pin  = %d\n", digitalRead(CC1101_GDO0));
    Serial.println("[CC1101] === FIM DIAGNOSTICO ===");
    Serial.flush();

    cc1101Sleep();
    return true;
}

// ============================================================
// CAPTURE
// ============================================================
void cc1101StartCapture() {
    if (!cc1101Initialized) return;
    if (!cc1101Wake()) {
        Serial.println("[CC1101] CAPTURE: falha ao acordar modulo!");
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
        Serial.println("[CC1101] WARN: Falha ao entrar em RX, tentando continuar...");
    }

    isr_last_val = digitalRead(CC1101_GDO0);
    isr_last_change = micros();
    isr_enabled = true;
    cc1101AttachISR();
    Serial.printf("[CC1101] Capture iniciada @ %lu Hz, MARCSTATE=0x%02X, GDO0=%d\n",
        currentCapture.frequency, readMarcState(), digitalRead(CC1101_GDO0));
    Serial.flush();
}

void cc1101CaptureLoop() {
    if (!cc1101CopyActive) return;
    unsigned long now = micros();
    unsigned long nowMs = millis();
    static unsigned long lastStateCheck = 0;
    if (nowMs - lastStateCheck > 2000) {
        lastStateCheck = nowMs;
        uint8_t ms = readMarcState();
        if (ms != 0x0D && ms != 0x0E && ms != 0x0F) {
            Serial.printf("[CC1101] CAPLOOP: estado errado 0x%02X, reentrando RX\n", ms);
            isr_enabled = false;
            cc1101DetachISR();
            cc1101GoRx(currentCapture.frequency);
            isr_last_val = digitalRead(CC1101_GDO0);
            isr_last_change = micros();
            isr_enabled = true;
            cc1101AttachISR();
        }
    }
    if (capture_state == STATE_HOPPING) {
        if (isr_count > 5) {
            capture_state = STATE_LOCKED;
        }
        else if (nowMs - lastFreqSwitch > 1000) {
            isr_enabled = false;
            cc1101DetachISR();
            isr_count = 0;
            capture_started = false;
            currentFreqIndex = (currentFreqIndex + 1) % 4;
            currentCapture.frequency = captureFreqs[currentFreqIndex];
            cc1101GoRx(currentCapture.frequency);
            isr_last_val = digitalRead(CC1101_GDO0);
            isr_last_change = micros();
            isr_enabled = true;
            cc1101AttachISR();
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
            cc1101DetachISR();
            isr_count = 0; capture_started = false;
            capture_state = STATE_HOPPING; isr_last_change = micros();
            currentFreqIndex = (currentFreqIndex + 1) % 4;
            currentCapture.frequency = captureFreqs[currentFreqIndex];
            cc1101GoRx(currentCapture.frequency);
            isr_last_val = digitalRead(CC1101_GDO0); isr_last_change = micros();
            isr_enabled = true;
            cc1101AttachISR();
            lastFreqSwitch = nowMs;
        }
        else if ((capture_started && silenceTimeout) || bufferFull) {
            isr_enabled = false;
            cc1101DetachISR();
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
                Serial.printf("[CC1101] Sinal capturado: %d pulsos, %lu us total\n", sig->length, totalDuration);
            } else {
                Serial.println("[CC1101] Capture terminou sem sinal valido");
            }
            Serial.flush();
            cc1101Sleep();
        }
        else if (totalTimeout) {
            isr_enabled = false;
            cc1101DetachISR();
            currentCapture.count = 0;
            currentCapture.startTime = millis();
            captureStartTime = millis();
            isr_count = 0; capture_started = false;
            capture_state = STATE_HOPPING; currentFreqIndex = 0;
            currentCapture.frequency = captureFreqs[currentFreqIndex];
            lastFreqSwitch = millis();
            cc1101GoRx(currentCapture.frequency);
            isr_last_val = digitalRead(CC1101_GDO0); isr_last_change = micros();
            isr_enabled = true;
            cc1101AttachISR();
        }
    }
}

void cc1101StopCapture() {
    isr_enabled = false;
    cc1101DetachISR();
    cc1101CopyActive = false;
    currentCapture.active = false;
    if (cc1101Awake) { cc1101SendCommand(CC1101_SIDLE); cc1101Sleep(); }
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
    cc1101DetachISR();
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
    pinMode(CC1101_GDO0, INPUT);
    cc1101WriteReg(CC1101_IOCFG0, 0x0D);
    cc1101SendCommand(CC1101_SIDLE);
    cc1101Sleep();
}

void cc1101SendBruteForceCode(uint32_t code, uint32_t freq) {
    if (!cc1101Initialized) return;
    if (!cc1101Wake()) return;
    isr_enabled = false;
    cc1101DetachISR();
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
    pinMode(CC1101_GDO0, INPUT);
    cc1101WriteReg(CC1101_IOCFG0, 0x0D);
    cc1101SendCommand(CC1101_SIDLE);
    cc1101Sleep();
}

void cc1101StartSubGHzJammer() {
    if (!cc1101Initialized) return;
    if (!cc1101Wake()) return;
    isr_enabled = false;
    cc1101DetachISR();
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
    pinMode(CC1101_GDO0, INPUT);
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
    pinMode(CC1101_GDO0, INPUT);
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
            isr_count = 0; isr_last_val = HIGH; isr_last_change = nowUs;
            capture_started = true; isr_enabled = true; rj_state = 1; rj_timer = now;
            cc1101AttachISR();
        }
    }
    else if (rj_state == 1) {
        if (now - rj_timer > 200) {
            isr_enabled = false;
            cc1101DetachISR();
            cc1101SendCommand(CC1101_SIDLE); delay(1);
            cc1101WriteReg(CC1101_IOCFG0, 0x2E);
            cc1101SendCommand(CC1101_SIDLE); delay(1);
            cc1101SendCommand(CC1101_STX);
            pinMode(CC1101_GDO0, OUTPUT);
            digitalWrite(CC1101_GDO0, HIGH);
            rj_state = 2; rj_timer = now;
        }
    }
    else if (rj_state == 2) {
        if (now - rj_timer > 200) {
            digitalWrite(CC1101_GDO0, LOW);
            pinMode(CC1101_GDO0, INPUT);
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
    cc1101RollJamActive = false; isr_enabled = false;
    cc1101DetachISR();
    digitalWrite(CC1101_GDO0, LOW);
    pinMode(CC1101_GDO0, INPUT);
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
    cc1101DetachISR();
    spec_an_running = true; spec_an_idx = 0;
    for(int i=0; i<15; i++) spec_an_freqs[i] = 300000000 + (i * 3200000);
    for(int i=0; i<16; i++) spec_an_freqs[15+i] = 387000000 + (i * 4800000);
    for(int i=0; i<33; i++) spec_an_freqs[31+i] = 779000000 + (i * 4500000);
    for (int i=0; i<64; i++) spec_an_values[i] = 0;
    cc1101SetFrequencyCalibrated(spec_an_freqs[0]);
}

void cc1101AnalyzerLoop() {
    if (!spec_an_running) return;
    for(int i=0; i<64; i++) { if (spec_an_values[i] > 0) spec_an_values[i]--; }
    uint32_t freq = spec_an_freqs[spec_an_idx];
    cc1101SetFrequencyCalibrated(freq);
    cc1101SendCommand(CC1101_SIDLE); delay(1);
    cc1101SendCommand(CC1101_SRX);
    delayMicroseconds(500);
    uint8_t rssiDec = cc1101ReadStatus(CC1101_RSSI);
    int rssi = (rssiDec >= 128) ? ((int)rssiDec - 256) / 2 - 74 : (int)rssiDec / 2 - 74;
    if (rssi < -90) rssi = -90;
    if (rssi > -50) rssi = -50;
    uint16_t target_h = map(rssi, -90, -50, 0, 40);
    if (target_h > spec_an_values[spec_an_idx]) spec_an_values[spec_an_idx] = target_h;
    spec_an_idx++;
    if (spec_an_idx >= 64) spec_an_idx = 0;
}

void cc1101StopAnalyzer() {
    spec_an_running = false;
    cc1101SendCommand(CC1101_SIDLE);
    cc1101Sleep();
}

bool cc1101AnalyzerIsRunning() { return spec_an_running; }
uint16_t cc1101GetAnalyzerValue(int idx) { if (idx < 0 || idx >= 64) return 0; return spec_an_values[idx]; }
uint32_t cc1101GetAnalyzerFreq(int idx) { if (idx < 0 || idx >= 64) return 0; return spec_an_freqs[idx] / 1000000; }
uint8_t cc1101GetAnalyzerSelected() { return spec_an_idx; }

void cc1101ClearSavedSignals() {
    savedSignalCount = 0;
    memset(savedSignals, 0, sizeof(savedSignals));
}

void cc1101DeleteSignal(uint8_t index) {
    if (index >= savedSignalCount) return;
    for (int i = index; i < savedSignalCount - 1; i++) savedSignals[i] = savedSignals[i + 1];
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
    cc1101DetachISR();
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
    pinMode(CC1101_GDO0, INPUT);
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
    cc1101DetachISR();
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
