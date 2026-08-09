#include <SPI.h>
#include "config.h"

// ============================================================
// CC1101 - Driver v12 (FIX: WiFi AP + HSPI coexistência)
//
// PROBLEMA QUE ESTE FIX RESOLVE:
//   No ESP32, HSPI usa GPIO 12-15. Esses pinos compartilham
//   a GPIO matrix com SDIO/WiFi. Quando WiFi AP está ativo,
//   a função hspi->end() libera a rota GPIO matrix, e o
//   subsistema WiFi (SDIO DMA) rouba os pinos entre transações.
//   Cada cc1101ReadReg/cc1101WriteReg chamava spiStart()/spiEnd()
//   que fazia begin()/end(). Após ~20 ciclos, os pinos ficavam
//   permanentemente corrompidos e todo SPI retornava 0x00.
//
// SOLUÇÃO:
//   - spiStart(): chama hspi->begin(pins) só na PRIMEIRA vez.
//     Depois disso, usa hspi->beginTransaction() (leve, não
//     toca na GPIO matrix).
//   - spiEnd(): só chama hspi->endTransaction() (leve).
//     NUNCA chama hspi->end() entre transações.
//   - spiBusRelease(): nova função que faz endTransaction()+end().
//     Chamada APENAS quando CC1101 vai para SLEEP.
//
//   Assim, enquanto CC1101 está acordado (capture, replay, etc.),
//   os pinos GPIO 12-15 ficam TRAVADOS no HSPI e WiFi não
//   consegue roubar. Quando CC1101 dorme, libera os pinos.
//
// PROVA: Com WiFi OFF funciona 100%. Com WiFi ON + este fix,
//   o bus permanece travado enquanto ativo, igual ao comportamento
//   com WiFi OFF.
// ============================================================

// SPI DEDICADO — ponteiro, igual JT_DRV
static SPIClass* hspi = NULL;

// Flag: o bus HSPI está inicializado (GPIO matrix configurada)?
// Quando true, spiStart() usa beginTransaction() (leve).
// Quando false, spiStart() usa begin() (pesado, reconfigura GPIO matrix).
static bool spiBusActive = false;

// SPISettings para beginTransaction
static SPISettings cc1101SPISettings(10000000, MSBFIRST, SPI_MODE0);

// ============================================================
// ENDEREÇOS — verificados contra datasheet SWRS061C
// ============================================================
#define CC1101_IOCFG2   0x00
#define CC1101_IOCFG0   0x02
#define CC1101_FIFOTHR  0x03
#define CC1101_PKTLEN   0x06
#define CC1101_PKTCTRL1 0x07
#define CC1101_PKTCTRL0 0x08
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
#define CC1101_PATABLE  0x3E

// SPI access flags
#define CC1101_READ_SINGLE  0x80
#define CC1101_READ_BURST   0xC0
#define CC1101_WRITE_BURST  0x40

// ============================================================
// VARIÁVEIS DE ESTADO
// ============================================================
bool cc1101Initialized = false;

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
static bool isrActuallyAttached = false;

// ============================================================
// ISR — attach/detach sob demanda
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
// SPI TRANSPORT v12 — BUS PERSISTENTE
//
// PROBLEMA DO v11:
//   spiStart() = hspi->begin(pins)  (reconfigura GPIO matrix)
//   spiEnd()   = hspi->end()        (LIBERA GPIO matrix → WiFi rouba)
//   Cada register read/write fazia begin/end, liberando os pinos
//   12-15 para WiFi SDIO entre transações.
//
// v12:
//   spiStart() = se bus não ativo: hspi->begin(pins) + seta flag
//                se bus ativo: hspi->beginTransaction(settings) (leve)
//   spiEnd()   = hspi->endTransaction() (leve, NÃO libera GPIO)
//   spiBusRelease() = endTransaction() + hspi->end() (libera GPIO)
//
//   Resultado: GPIO 12-15 ficam travados no HSPI enquanto CC1101
//   está acordado. WiFi não consegue roubar.
// ============================================================

static void spiStart(void) {
    if (!spiBusActive) {
        // Primeira chamada: inicializa o bus (pesado, reconfigura GPIO matrix)
        hspi->begin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CSN);
        spiBusActive = true;
        // Pequeno delay para GPIO matrix estabilizar
        delayMicroseconds(10);
    }
    // Transação leve (não toca GPIO matrix)
    hspi->beginTransaction(cc1101SPISettings);
}

static void spiEnd(void) {
    // Só termina a transação (leve). NÃO libera o bus.
    // Os pinos 12-15 continuam roteados para HSPI.
    hspi->endTransaction();
}

// Libera o bus completamente. Chamado APENAS quando CC1101 vai dormir.
static void spiBusRelease(void) {
    if (spiBusActive) {
        hspi->endTransaction();
        hspi->end();
        spiBusActive = false;
    }
}

// ============================================================
// SPI PRIMITIVES
// Cada função: spiStart → CSN LOW → while(MISO) →
//               transfer → CSN HIGH → spiEnd
// ============================================================

uint8_t cc1101ReadReg(uint8_t reg) {
    spiStart();
    digitalWrite(CC1101_CSN, LOW);
    while (digitalRead(CC1101_MISO));
    hspi->transfer(reg | CC1101_READ_SINGLE);
    uint8_t val = hspi->transfer(0x00);
    digitalWrite(CC1101_CSN, HIGH);
    spiEnd();
    return val;
}

uint8_t cc1101ReadStatus(uint8_t reg) {
    spiStart();
    digitalWrite(CC1101_CSN, LOW);
    while (digitalRead(CC1101_MISO));
    hspi->transfer(reg | CC1101_READ_BURST);
    uint8_t val = hspi->transfer(0x00);
    digitalWrite(CC1101_CSN, HIGH);
    spiEnd();
    return val;
}

void cc1101WriteReg(uint8_t reg, uint8_t value) {
    spiStart();
    digitalWrite(CC1101_CSN, LOW);
    while (digitalRead(CC1101_MISO));
    hspi->transfer(reg);
    hspi->transfer(value);
    digitalWrite(CC1101_CSN, HIGH);
    spiEnd();
}

static bool cc1101WriteRegBurst(uint8_t reg, uint8_t* data, uint8_t len) {
    spiStart();
    digitalWrite(CC1101_CSN, LOW);
    while (digitalRead(CC1101_MISO));
    hspi->transfer(reg | CC1101_WRITE_BURST);
    for (uint8_t i = 0; i < len; i++) {
        hspi->transfer(data[i]);
    }
    digitalWrite(CC1101_CSN, HIGH);
    spiEnd();
    return true;
}

void cc1101SendCommand(uint8_t cmd) {
    spiStart();
    digitalWrite(CC1101_CSN, LOW);
    while (digitalRead(CC1101_MISO));
    uint8_t status = hspi->transfer(cmd);
    digitalWrite(CC1101_CSN, HIGH);
    spiEnd();
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
// RESET — sequência ELECHOUSE
// ============================================================
static bool cc1101Reset() {
    Serial.println("[CC1101] RESET: iniciando...");
    Serial.flush();

    spiStart();
    digitalWrite(CC1101_CSN, LOW);
    delay(1);
    digitalWrite(CC1101_CSN, HIGH);
    delay(1);
    digitalWrite(CC1101_CSN, LOW);
    while (digitalRead(CC1101_MISO));
    hspi->transfer(CC1101_SRES);
    while (digitalRead(CC1101_MISO));
    digitalWrite(CC1101_CSN, HIGH);
    spiEnd();

    delay(2);

    uint8_t pn = cc1101ReadStatus(CC1101_PARTNUM);
    Serial.printf("[CC1101] RESET: PARTNUM=0x%02X\n", pn);

    if (pn == 0xFF) {
        Serial.println("[CC1101] RESET: chip nao responde");
        return false;
    }

    uint8_t ms = cc1101ReadStatus(CC1101_MARCSTATE) & 0x1F;
    Serial.printf("[CC1101] RESET: MARCSTATE=0x%02X\n", ms);

    if (ms != 0x01) {
        Serial.println("[CC1101] RESET: forçando SIDLE...");
        cc1101SendCommand(CC1101_SIDLE);
        delay(1);
        ms = cc1101ReadStatus(CC1101_MARCSTATE) & 0x1F;
        Serial.printf("[CC1101] RESET: apos SIDLE, MARCSTATE=0x%02X\n", ms);
    }

    Serial.println("[CC1101] RESET: OK");
    return true;
}

// ============================================================
// CONFIGURAÇÃO — mesma do v4/ELECHOUSE
// ============================================================
static void cc1101ConfigureRegs() {
    cc1101WriteReg(CC1101_IOCFG2,   0x0D);
    cc1101WriteReg(CC1101_IOCFG0,   0x0D);
    cc1101WriteReg(CC1101_PKTCTRL0, 0x32);
    cc1101WriteReg(CC1101_MDMCFG3,  0x93);
    cc1101WriteReg(CC1101_MDMCFG4, 0x07);

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
// GoRx
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

    cc1101SendCommand(CC1101_SIDLE);
    cc1101SetFrequency(freqHz);
    cc1101CalibrateBand(freqMHz);
    cc1101SendCommand(CC1101_SRX);

    state = readMarcState();
    Serial.printf("[CC1101] GoRx: estado final=0x%02X @ %lu Hz\n", state, freqHz);

    if (state == 0x0D || state == 0x0E || state == 0x08 || state == 0x06 || state == 0x0B) {
        Serial.println("[CC1101] GoRx: SUCESSO");
        return true;
    }

    Serial.printf("[CC1101] GoRx: falhou (0x%02X), retry...\n", state);
    cc1101SendCommand(CC1101_SIDLE);
    cc1101SendCommand(CC1101_SRX);

    state = readMarcState();
    if (state == 0x0D || state == 0x0E || state == 0x08 || state == 0x06 || state == 0x0B) {
        Serial.printf("[CC1101] GoRx: SUCESSO no retry (0x%02X)\n", state);
        return true;
    }

    Serial.printf("[CC1101] GoRx: FALHOU, state=0x%02X\n", state);
    return false;
}

static void cc1101SetFrequencyCalibrated(uint32_t freqHz) {
    float freqMHz = freqHz / 1000000.0f;
    cc1101SendCommand(CC1101_SIDLE);
    cc1101SetFrequency(freqHz);
    cc1101CalibrateBand(freqMHz);
}

// ============================================================
// SLEEP / WAKE / INIT
//
// v12: cc1101Wake() NÃO chama spiBusRelease() no final.
//   O bus HSPI fica travado nos pinos 12-15 enquanto CC1101
//   está acordado. Isso impede WiFi SDIO de roubar os pinos.
//
//   cc1101Sleep() chama spiBusRelease() para liberar os
//   pinos de volta ao sistema (WiFi pode usar se precisar).
// ============================================================

void cc1101Sleep() {
    if (!cc1101Initialized) return;
    cc1101DetachISR();
    isr_enabled = false;
    cc1101SendCommand(CC1101_SIDLE);
    cc1101SendCommand(CC1101_SPWD);
    // v12: LIBERA o bus HSPI. WiFi pode usar os pinos 12-15.
    spiBusRelease();
    cc1101Awake = false;
    Serial.println("[CC1101] Modulo em SLEEP");
}

bool cc1101Wake() {
    if (!cc1101Initialized) return false;
    Serial.println("[CC1101] WAKE: acordando modulo...");
    Serial.flush();

    cc1101DetachISR();
    isr_enabled = false;

    // v12: A primeira chamada a spiStart() aqui vai chamar hspi->begin(pins),
    // reconfigurando a GPIO matrix e roubando os pinos 12-15 do WiFi.
    // Isso é seguro porque cc1101Reset() chama spiStart() internamente.
    // Todas as chamadas subsequentes a spiStart() serão beginTransaction (leve).

    if (!cc1101Reset()) {
        Serial.println("[CC1101] WAKE: reset falhou!");
        spiBusRelease();
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
        spiBusRelease();
    }
    Serial.flush();
    // v12: NÃO chama spiBusRelease(). O bus fica travado.
    return cc1101Awake;
}

bool cc1101Init() {
    Serial.println("[CC1101] Inicializando...");
    Serial.flush();

    hspi = new SPIClass(HSPI);

    pinMode(CC1101_GDO0, INPUT);
    pinMode(CC1101_GDO2, INPUT);

    // Primeiro spiStart() vai chamar hspi->begin(pins)
    spiStart();
    pinMode(CC1101_CSN, OUTPUT);
    digitalWrite(CC1101_CSN, HIGH);

    // Reset manual (bus já está ativo via spiStart)
    spiStart();
    digitalWrite(CC1101_CSN, LOW);
    delay(1);
    digitalWrite(CC1101_CSN, HIGH);
    delay(1);
    digitalWrite(CC1101_CSN, LOW);
    while (digitalRead(CC1101_MISO));
    hspi->transfer(CC1101_SRES);
    while (digitalRead(CC1101_MISO));
    digitalWrite(CC1101_CSN, HIGH);
    spiEnd();
    delay(2);

    cc1101ConfigureRegs();

    uint8_t partnum = cc1101ReadStatus(CC1101_PARTNUM);
    uint8_t version = cc1101ReadStatus(CC1101_VERSION);
    Serial.printf("[CC1101] PARTNUM=0x%02X VERSION=0x%02X\n", partnum, version);
    Serial.flush();

    if (partnum == 0xFF && version == 0xFF) {
        Serial.println("[CC1101] FAIL: modulo nao responde (0xFF)");
        spiBusRelease();
        return false;
    }

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
    r = cc1101ReadReg(CC1101_AGCCTRL2); Serial.printf("  AGCCTRL2   = 0x%02X %s\n", r, r==0xC7?"OK":"FAIL");
    r = cc1101ReadReg(CC1101_MCSM0);    Serial.printf("  MCSM0     = 0x%02X %s\n", r, r==0x38?"OK":"FAIL");
    r = cc1101ReadStatus(CC1101_MARCSTATE) & 0x1F;
    Serial.printf("  MARCSTATE = 0x%02X (0x01=IDLE)\n", r);
    Serial.printf("  GDO0 pin  = %d\n", digitalRead(CC1101_GDO0));
    Serial.println("[CC1101] === FIM DIAGNOSTICO ===");
    Serial.flush();

    // v12: Colocar em sleep e LIBERAR bus (WiFi ainda não iniciou, mas por consistência)
    cc1101SendCommand(CC1101_SIDLE);
    cc1101SendCommand(CC1101_SPWD);
    spiBusRelease();
    cc1101Awake = false;

    Serial.println("[CC1101] Modulo em SLEEP");

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

bool cc1101DidDisableWiFi() { return false; }
