#include <SPI.h>
#include "config.h"

// ============================================================
// CC1101 - Driver custom (baseado no padrão ELECHOUSE/DIV/Flipper)
// 
// REFERÊNCIAS:
//   - ELECHOUSE_cc1101 (SmartRC-CC1101-Driver-Lib) V3.0.2
//   - Flipper Zero firmware (subghz_device_cc1101_preset_ook_270khz_async)
//   - DIV (Cifertech) — usa ELECHOUSE internamente
//   - CC1101 Datasheet SWRS061C (Texas Instruments)
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
//   - FREND1 não era escrito (front-end analógico errado)
//   - PATABLE todo 0xC0 (OOK sem modulação)
//   - AGCCTRL2/0 com valores subótimos
//   - MDMCFG4 banda muito larga (812kHz → 270kHz)
//   - cc1101Reset(): endTransaction() ANTES de CSN HIGH (corrompia SRES)
//   - Módulos NRF24/CC1101 ativos simultaneamente (interferência SPI)
//   - *** ENDEREÇOS DE REGISTRADORES ERRADOS ***
//     FSCTRL1 era 0x07 (FIFOTHR!), correto é 0x0B
//     PKTCTRL1 era 0x09 (ADDR!), correto é 0x07
//     → Isso fazia escrever nos registradores ERRADOS, destruindo a config
//     → O chip nunca conseguia entrar em RX por causa disso
// ============================================================

SPIClass spiCC1101(HSPI);

// ============================================================
// ENDEREÇOS DE REGISTRADORES — verificados contra datasheet SWRS061C
// e contra cc1101_regs.h do Flipper Zero. NÃO mudar sem verificar!
// Mapa: 0x00=IOCFG2  0x02=IOCFG0  0x03=FIFOTHR  0x06=PKTLEN
//   0x07=PKTCTRL1  0x08=PKTCTRL0  0x09=ADDR  0x0A=CHANNR
//   0x0B=FSCTRL1  0x0C=FSCTRL0  0x0D=FREQ2  0x0E=FREQ1  0x0F=FREQ0
//   0x10=MDMCFG4  0x11=MDMCFG3  0x12=MDMCFG2  0x13=MDMCFG1  0x14=MDMCFG0
//   0x15=DEVIATN  0x18=MCSM0  0x19=FOCCFG  0x1A=BSCFG
//   0x1B=AGCCTRL2  0x1C=AGCCTRL1  0x1D=AGCCTRL0
//   0x20=WORCTRL  0x21=FREND1  0x22=FREND0
//   0x23=FSCAL3  0x24=FSCAL2  0x25=FSCAL1  0x26=FSCAL0
//   0x29=FSTEST  0x2C=TEST2  0x2D=TEST1  0x2E=TEST0
// ============================================================
#define CC1101_IOCFG2   0x00  // GDO2 output pin configuration
#define CC1101_IOCFG0   0x02  // GDO0 output pin configuration
#define CC1101_FIFOTHR  0x03  // RX FIFO and TX FIFO thresholds
#define CC1101_FSCTRL1  0x0B  // Frequency synthesizer control (ERRO: era 0x07!)
#define CC1101_PKTCTRL1 0x07  // Packet automation control (ERRO: era 0x09!)
#define CC1101_PKTCTRL0 0x08  // Packet automation control
#define CC1101_ADDR     0x09  // Device address (antes confundido com PKTCTRL1)
#define CC1101_CHANNR   0x0A  // Channel number
#define CC1101_PKTLEN   0x06  // Packet length
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
#define CC1101_WORCTRL  0x20  // Wake On Radio control
#define CC1101_FSTEST   0x29  // Frequency synthesizer test (ERRO: era 0x2B!)
#define CC1101_TEST2    0x2C  // Various test settings
#define CC1101_TEST1    0x2D  // Various test settings
#define CC1101_TEST0    0x2E  // Various test settings
#define CC1101_PARTNUM  0x30  // Part number (status: 0x00 = CC1101)
#define CC1101_VERSION  0x31  // Chip version (status: 0x14 = Rev C)
#define CC1101_RSSI     0x34  // RSSI status register (0x34 = RSSI, 0x35/0x36 = MARCSTATE)
#define CC1101_MARCSTATE 0x35  // Main radio control state machine status

#define CC1101_SRES     0x30
#define CC1101_SCAL     0x33
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

// Flag para saber se o módulo está "acordado" (ativo) ou "dormindo"
static bool cc1101Awake = false;

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
        if (millis() - start > 200) return false;
    }
    return true;
}

// Select = CSN LOW + espera chip ready (CHIP_RDYn LOW)
// Retorna true se o chip ficou pronto para comunicação.
// FIX: Adicionado delay de 20µs após CSN LOW para garantir
// que o chip processe a borda de descida, mesmo com MISO já LOW.
static bool cc1101Select() {
    digitalWrite(CC1101_CSN, LOW);
    delayMicroseconds(20);  // FIX: tempo mínimo para chip processar CSN falling edge
    bool ready = waitMisoReady();
    if (!ready) {
        digitalWrite(CC1101_CSN, HIGH);
        Serial.println("[CC1101] WARN: MISO timeout no cc1101Select()");
    }
    return ready;
}

// Deselect = CSN HIGH com tempo mínimo de 40µs (t_sp datasheet TI)
static inline void cc1101Deselect() {
    digitalWrite(CC1101_CSN, HIGH);
    delayMicroseconds(50);
}

static inline void cc1101SpiStart() {
    spiCC1101.beginTransaction(CC1101_SPI_SETTINGS);
}

static inline void cc1101SpiEnd() {
    spiCC1101.endTransaction();
}

// ============================================================
// SPI PRIMITIVES — verificam chip ready antes de transferir
// ============================================================

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

static void cc1101SetFrequencyCalibrated(uint32_t freqHz) {
    cc1101SendCommand(CC1101_SIDLE);
    delay(1);
    cc1101SetFrequency(freqHz);
    cc1101SendCommand(CC1101_SCAL);
    delay(2);
}

// ============================================================
// RESET — sequência ELECHOUSE CORRIGIDA
// FIX CRÍTICO: CSN HIGH ANTES de endTransaction()
// O ESP32 pode deconfigurar pinos SPI no endTransaction(),
// causando glitches que corrompem o comando SRES.
// ============================================================
static bool cc1101Reset() {
    // Pulso CSN para garantir que o chip está atento
    digitalWrite(CC1101_CSN, LOW);
    delayMicroseconds(10);
    digitalWrite(CC1101_CSN, HIGH);
    delayMicroseconds(40);
    digitalWrite(CC1101_CSN, LOW);

    // Espera MISO LOW (chip acordou, crystal estável)
    if (!waitMisoReady()) {
        Serial.println("[CC1101] RESET: MISO timeout antes do SRES");
        digitalWrite(CC1101_CSN, HIGH);
        return false;
    }

    // Envia SRES — CSN HIGH antes de endTransaction!
    spiCC1101.beginTransaction(CC1101_SPI_SETTINGS);
    spiCC1101.transfer(CC1101_SRES);
    // FIX: CSN HIGH PRIMEIRO, depois endTransaction
    digitalWrite(CC1101_CSN, HIGH);
    delayMicroseconds(50);
    spiCC1101.endTransaction();

    // Espera MISO LOW novamente (reset completo, chip em IDLE)
    if (!waitMisoReady()) {
        Serial.println("[CC1101] RESET: MISO timeout apos SRES");
        return false;
    }

    delay(1);
    return true;
}

// ============================================================
// CONFIGURAÇÃO DOS REGISTRADORES (extraída de cc1101Init)
// Valores baseados em Flipper Zero (ook_270khz_async) + ELECHOUSE
// ============================================================
static void cc1101ConfigureRegs() {
    // GDO0: async serial data output (dados OOK saem direto no GDO0)
    cc1101WriteReg(CC1101_IOCFG0,   0x0D);

    // FIFO: ADC retention + threshold adequado
    cc1101WriteReg(CC1101_FIFOTHR,  0x47);

    // IF Frequency: OBRIGATÓRIO (faltava no original)
    cc1101WriteReg(CC1101_FSCTRL1,  0x06);

    // Modo de pacote: async serial, sem CRC, sem whitening
    cc1101WriteReg(CC1101_PKTCTRL0, 0x32);
    cc1101WriteReg(CC1101_PKTCTRL1, 0x04);
    cc1101WriteReg(CC1101_ADDR,     0x00);  // Sem filtro de endereço
    cc1101WriteReg(CC1101_PKTLEN,  0x00);  // Tamanho variável (async mode)
    cc1101WriteReg(CC1101_CHANNR,  0x00);  // Canal 0

    // Modulação OOK/ASK
    cc1101WriteReg(CC1101_MDMCFG4,  0x67);
    cc1101WriteReg(CC1101_MDMCFG3,  0x32);
    cc1101WriteReg(CC1101_MDMCFG2,  0x30);
    cc1101WriteReg(CC1101_MDMCFG1,  0x00);
    cc1101WriteReg(CC1101_MDMCFG0,  0x00);

    // Deviation (irrelevante para OOK)
    cc1101WriteReg(CC1101_DEVIATN,  0x15);

    // Controle de máquina de estados
    cc1101WriteReg(CC1101_MCSM0,    0x18);

    // Frequency offset compensation
    cc1101WriteReg(CC1101_FOCCFG,   0x16);

    // Bit synchronization
    cc1101WriteReg(CC1101_BSCFG,    0x1C);

    // AGC: valores do Flipper Zero
    cc1101WriteReg(CC1101_AGCCTRL2, 0x03);
    cc1101WriteReg(CC1101_AGCCTRL1, 0x00);
    cc1101WriteReg(CC1101_AGCCTRL0, 0x40);

    // Front-end analógico RX/TX
    cc1101WriteReg(CC1101_FREND1,   0xB6);
    cc1101WriteReg(CC1101_FREND0,   0x11);

    // Calibração do sintetizador de frequência
    cc1101WriteReg(CC1101_FSCAL3,   0xE9);
    cc1101WriteReg(CC1101_FSCAL2,   0x2A);
    cc1101WriteReg(CC1101_FSCAL1,   0x00);
    cc1101WriteReg(CC1101_FSCAL0,   0x1F);

    // Teste de produção
    cc1101WriteReg(CC1101_FSTEST,   0x59);

    // Test registers
    cc1101WriteReg(CC1101_TEST2,    0x81);
    cc1101WriteReg(CC1101_TEST1,    0x35);
    cc1101WriteReg(CC1101_TEST0,    0x09);

    // PATABLE — OOK: PA[0]=0x00 (OFF), PA[1]=0xC0 (ON)
    uint8_t paTable[8] = {0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    cc1101WriteRegBurst(CC1101_PATABLE, paTable, 8);
}

// ============================================================
// SLEEP — desativa o CC1101 completamente (crystal OFF)
// Deve ser chamado após qualquer operação CC1101
// ============================================================
void cc1101Sleep() {
    if (!cc1101Initialized) return;
    isr_enabled = false;
    cc1101SendCommand(CC1101_SIDLE);
    delay(1);
    cc1101SendCommand(CC1101_SPWD);
    cc1101Awake = false;
    Serial.println("[CC1101] Modulo em SLEEP (desativado)");
}

// ============================================================
// WAKE — acorda o CC1101 do SLEEP, reseta e reconfigura
// Deve ser chamado ANTES de qualquer operação CC1101
// Retorna true se o chip está pronto para uso
// ============================================================
bool cc1101Wake() {
    if (!cc1101Initialized) return false;

    Serial.println("[CC1101] WAKE: acordando modulo...");
    Serial.flush();

    // 1. Reset completo
    if (!cc1101Reset()) {
        Serial.println("[CC1101] WAKE: reset falhou!");
        cc1101Awake = false;
        return false;
    }

    // 2. Reconfigura todos os registradores
    cc1101ConfigureRegs();

    // 3. Anexa ISR no GDO0 (CHANGE para capturar ambas as bordas)
    attachInterrupt(digitalPinToInterrupt(CC1101_GDO0), cc1101ISR, CHANGE);

    // 4. Verifica estado
    uint8_t state = cc1101ReadStatus(CC1101_MARCSTATE) & 0x1F;
    uint8_t version = cc1101ReadStatus(CC1101_VERSION);
    Serial.printf("[CC1101] WAKE: MARCSTATE=0x%02X, VERSION=0x%02X\n", state, version);

    if (state != 0x01) {
        Serial.printf("[CC1101] WAKE: estado inesperado 0x%02X, tentando SIDLE...\n", state);
        cc1101SendCommand(CC1101_SIDLE);
        delay(2);
        state = cc1101ReadStatus(CC1101_MARCSTATE) & 0x1F;
        Serial.printf("[CC1101] WAKE: apos SIDLE, state=0x%02X\n", state);
    }

    cc1101Awake = (state == 0x01);
    if (cc1101Awake) {
        Serial.println("[CC1101] WAKE: modulo pronto (IDLE)");
    } else {
        Serial.println("[CC1101] WAKE: FALHOU — modulo nao atingiu IDLE");
    }
    Serial.flush();
    return cc1101Awake;
}

// ============================================================
// INIT — inicializa o SPI, testa o chip, configura, e DORME
// O chip só acorda quando cc1101Wake() for chamado
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

    // Reset do CC1101
    if (!cc1101Reset()) {
        Serial.println("[CC1101] FAIL: reset falhou");
        return false;
    }

    // Lê PARTNUM e VERSION
    uint8_t partnum = cc1101ReadStatus(CC1101_PARTNUM);
    uint8_t version = cc1101ReadStatus(CC1101_VERSION);
    Serial.printf("[CC1101] PARTNUM=0x%02X VERSION=0x%02X\n", partnum, version);
    Serial.flush();

    if (partnum == 0xFF && version == 0xFF) {
        Serial.println("[CC1101] FAIL: modulo nao responde (PARTNUM=0xFF)");
        Serial.println("[CC1101] Verifique: 1) Fio MISO (GPIO 12) 2) Alimentacao 3.3V 3) Contato CC1101");
        return false;
    }

    // Configura registradores
    cc1101ConfigureRegs();

    cc1101Initialized = true;
    Serial.println("[CC1101] Configurado com sucesso!");
    Serial.flush();

    // Diagnóstico
    Serial.println("[CC1101] === DIAGNOSTICO ===");
    uint8_t r;
    r = cc1101ReadReg(CC1101_FSCTRL1);  Serial.printf("  FSCTRL1   = 0x%02X %s\n", r, r==0x06?"OK":"FAIL");
    r = cc1101ReadReg(CC1101_FREND1);   Serial.printf("  FREND1    = 0x%02X %s\n", r, r==0xB6?"OK":"FAIL");
    r = cc1101ReadReg(CC1101_IOCFG0);   Serial.printf("  IOCFG0    = 0x%02X %s\n", r, r==0x0D?"OK":"FAIL");
    r = cc1101ReadReg(CC1101_PKTCTRL0); Serial.printf("  PKTCTRL0  = 0x%02X %s\n", r, r==0x32?"OK":"FAIL");
    r = cc1101ReadReg(CC1101_MDMCFG4);  Serial.printf("  MDMCFG4   = 0x%02X %s\n", r, r==0x67?"OK":"FAIL");
    r = cc1101ReadReg(CC1101_MDMCFG2);  Serial.printf("  MDMCFG2   = 0x%02X %s\n", r, r==0x30?"OK":"FAIL");
    r = cc1101ReadReg(CC1101_AGCCTRL2); Serial.printf("  AGCCTRL2  = 0x%02X %s\n", r, r==0x03?"OK":"FAIL");
    r = cc1101ReadStatus(CC1101_MARCSTATE) & 0x1F;
    Serial.printf("  MARCSTATE = 0x%02X (0x01=IDLE apos reset)\n", r);
    Serial.printf("  GDO0 pin  = %d\n", digitalRead(CC1101_GDO0));
    Serial.println("[CC1101] === FIM DIAGNOSTICO ===");
    Serial.flush();

    // Coloca o módulo em SLEEP imediatamente após init
    cc1101Sleep();

    return true;
}

// ============================================================
// HELPER: Entra em RX com verificação de estado
// Chip DEVE estar acordado (cc1101Wake chamado antes)
// ============================================================
static uint8_t readMarcState() {
    return cc1101ReadStatus(CC1101_MARCSTATE) & 0x1F;
}

static bool cc1101GoRx(uint32_t freqHz) {
    // === PADRÃO ELECHOUSE (DIV usa exatamente isso): ===
    // 1. cc1101SendCommand(SIDLE) — transação SPI separada
    // 2. cc1101SendCommand(SRX)  — transação SPI separada
    // Cada strobe é uma transação SPI completa com CSN toggle.
    // MCSM0.FS_AUTOCAL=1 faz auto-calibration automaticamente na transição IDLE→RX.
    // NÃO enviamos SCAL manual — igual DIV e Flipper.

    // 1. Seta frequência
    cc1101SetFrequency(freqHz);

    // 2. SIDLE — sai de qualquer estado, volta para IDLE
    cc1101SendCommand(CC1101_SIDLE);
    delayMicroseconds(100);

    // 3. SRX — entra em RX (auto-calibration acontece aqui automaticamente)
    cc1101SendCommand(CC1101_SRX);

    // 4. Espera o chip terminar calibração e entrar em RX
    //    (MISO sobe durante cal, volta LOW quando RX está pronto)
    uint32_t t0 = millis();
    while (digitalRead(CC1101_MISO) != LOW) {
        if (millis() - t0 > 500) {
            Serial.println("[CC1101] GoRx: timeout esperando RX ready");
            break;
        }
    }

    // 5. Verifica estado
    delay(1);
    uint8_t state = readMarcState();
    Serial.printf("[CC1101] GoRx: state=0x%02X @ %lu Hz\n", state, freqHz);

    if (state == 0x0D || state == 0x0E || state == 0x0F) {
        return true;
    }

    // 6. Fallback — tenta mais uma vez com SIDLE + SRX
    Serial.printf("[CC1101] GoRx: falhou (state=0x%02X), retentativa...\n", state);
    cc1101SendCommand(CC1101_SIDLE);
    delay(2);
    cc1101SendCommand(CC1101_SRX);

    t0 = millis();
    while (digitalRead(CC1101_MISO) != LOW) {
        if (millis() - t0 > 500) break;
    }

    delay(2);
    state = readMarcState();
    if (state == 0x0D || state == 0x0E || state == 0x0F) {
        Serial.printf("[CC1101] GoRx: RX OK na 2a tentativa\n");
        return true;
    }

    Serial.printf("[CC1101] GoRx: FALHOU, state final=0x%02X\n", state);
    return false;
}

// ============================================================
// CAPTURE - Copiar Sinal
// ============================================================
void cc1101StartCapture() {
    if (!cc1101Initialized) return;

    // ACORDA o módulo antes de usar
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

    // NOTA: IOCFG0 já está 0x0D do cc1101Wake() e pinMode já foi feito no init.
    // Não re-enviamos aqui para evitar transação SPI desnecessária.

    // Entra em RX
    bool rxOk = cc1101GoRx(currentCapture.frequency);
    if (!rxOk) {
        Serial.println("[CC1101] WARN: Falha ao entrar em RX, tentando continuar...");
    }

    // Habilita ISR
    isr_last_val = digitalRead(CC1101_GDO0);
    isr_last_change = micros();
    isr_enabled = true;

    Serial.printf("[CC1101] Capture iniciada @ %lu Hz, MARCSTATE=0x%02X, GDO0=%d\n",
        currentCapture.frequency, readMarcState(), digitalRead(CC1101_GDO0));
    Serial.flush();
}

void cc1101CaptureLoop() {
    if (!cc1101CopyActive) return;
    unsigned long now = micros();
    unsigned long nowMs = millis();

    // Verificação periódica: chip ainda está em RX?
    static unsigned long lastStateCheck = 0;
    if (nowMs - lastStateCheck > 2000) {
        lastStateCheck = nowMs;
        uint8_t ms = readMarcState();
        if (ms != 0x0D && ms != 0x0E && ms != 0x0F) {
            Serial.printf("[CC1101] CAPLOOP: estado errado 0x%02X, reentrando RX\n", ms);
            isr_enabled = false;
            cc1101GoRx(currentCapture.frequency);
            isr_last_val = digitalRead(CC1101_GDO0);
            isr_last_change = micros();
            isr_enabled = true;
        }
    }

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
                Serial.printf("[CC1101] Sinal capturado: %d pulsos, %lu us total\n",
                    sig->length, totalDuration);
            } else {
                Serial.println("[CC1101] Capture terminou sem sinal valido");
            }
            Serial.flush();

            // Volta o módulo para SLEEP após captura
            cc1101Sleep();
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
    cc1101CopyActive = false;
    currentCapture.active = false;
    if (cc1101Awake) {
        cc1101SendCommand(CC1101_SIDLE);
        cc1101Sleep();
    }
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

    // Acorda o módulo
    if (!cc1101Wake()) return;

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

    // Volta pra SLEEP
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
// ROLLJAM AUTO
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
