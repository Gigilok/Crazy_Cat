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
// TRANSPORTE SPI — idêntico ao ELECHOUSE
// O ELECHOUSE funciona no ESP32. A chave é:
//   - SPI.begin()/SPI.end() a cada transação (reinit HW completo)
//   - SPI MODE 0, MSBFIRST, clock default do ESP32
//   - Sempre esperar MISO LOW (CHIP_RDYn) antes de transferir
//   - Sempre CSN LOW antes, CSN HIGH depois
//
// BUGS ENCONTRADOS E CORRIGIDOS:
//   1. Endereços de registradores errados (FSCTRL1, PKTCTRL1, FSTEST)
//   2. beginTransaction()/endTransaction() corrompia strobes no ESP32 HSPI
//   3. SCAL (0x33) faz chip ir para SLEEP no ESP32
//   4. Leituras de MARCSTATE entre SIDLE e SRX adicionam transações SPI
//      extras que perturbam o chip → SRX vai para SLEEP
//   5. Reset não recuperava chip de estado morto (SLEEP profundo)
//   6. cc1101SpiEnd() chamava endTransaction() desnecessariamente
//
// SOLUÇÃO FINAL:
//   - begin()/end() por transação, sem endTransaction()
//   - Reset com pulso CSN manual (sem depender de SPI.begin)
//   - GoRx SEM leituras de estado entre SIDLE/SRX (igual ELECHOUSE)
//   - Calibração por banda via FSCTRL0/TEST0 (igual ELECHOUSE)
// ============================================================

SPIClass spiCC1101(HSPI);

// ============================================================
// ENDEREÇOS DE REGISTRADORES — verificados contra datasheet SWRS061C
// Mapa correto:
//   0x00=IOCFG2  0x02=IOCFG0  0x03=FIFOTHR  0x06=PKTLEN
//   0x07=PKTCTRL1  0x08=PKTCTRL0  0x09=ADDR  0x0A=CHANNR
//   0x0B=FSCTRL1  0x0C=FSCTRL0  0x0D=FREQ2  0x0E=FREQ1  0x0F=FREQ0
//   0x10=MDMCFG4  0x11=MDMCFG3  0x12=MDMCFG2  0x13=MDMCFG1  0x14=MDMCFG0
//   0x15=DEVIATN  0x18=MCSM0  0x19=FOCCFG  0x1A=BSCFG
//   0x1B=AGCCTRL2  0x1C=AGCCTRL1  0x1D=AGCCTRL0
//   0x20=WORCTRL  0x21=FREND1  0x22=FREND0
//   0x23=FSCAL3  0x24=FSCAL2  0x25=FSCAL1  0x26=FSCAL0
//   0x29=FSTEST  0x2C=TEST2  0x2D=TEST1  0x2E=TEST0
//
//   Registradores de STATUS (lidos com 0x40+addr ou 0x80+addr):
//   0x30=PARTNUM  0x31=VERSION  0x34=RSSI  0x35/0x36=MARCSTATE
//
//   Strobes de COMANDO (enviados direto, sem bits 0x40/0x80):
//   0x30=SRES  0x33=SCAL  0x34=SRX  0x35=STX
//   0x36=SIDLE  0x39=SPWD  0x3A=SFRX
// ============================================================
#define CC1101_IOCFG2   0x00
#define CC1101_IOCFG0   0x02
#define CC1101_FIFOTHR  0x03
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

// Status register addresses (para leitura com CC1101_READ_SINGLE)
#define CC1101_PARTNUM  0x30
#define CC1101_VERSION  0x31
#define CC1101_RSSI     0x34
#define CC1101_MARCSTATE 0x35

// Strobe commands (enviados diretamente como byte de comando)
#define CC1101_SRES     0x30
#define CC1101_SCAL     0x33
#define CC1101_SRX      0x34
#define CC1101_STX      0x35
#define CC1101_SIDLE    0x36
#define CC1101_SPWD     0x39
#define CC1101_SFRX     0x3A
#define CC1101_PATABLE  0x3E

// SPI access modifiers
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

// ============================================================
// ISR - captura timestamps de transição do GDO0
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
// SPI TRANSPORT — idêntico ao ELECHOUSE
// Cada transação: begin() → CSN LOW → wait MISO LOW → transfer → CSN HIGH → end()
// NUNCA usar beginTransaction()/endTransaction() — causa glitches no ESP32 HSPI
// ============================================================

// Espera MISO ir para LOW (CHIP_RDYn ativo-LOW).
// Igual ELECHOUSE: sem timeout.
static void waitMisoReady() {
    while (digitalRead(CC1101_MISO) != LOW) { yield(); }
}

// SPI START — igual ELECHOUSE SpiStart()
// Reconfigura pinos e reinitializa o hardware SPI.
static inline void cc1101SpiStart() {
    pinMode(CC1101_SCK, OUTPUT);
    pinMode(CC1101_MISO, INPUT);
    pinMode(CC1101_MOSI, OUTPUT);
    pinMode(CC1101_CSN, OUTPUT);
    spiCC1101.begin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CSN);
}

// SPI END — igual ELECHOUSE SpiEnd()
// Apenas end(). NÃO chama endTransaction() — begin/end já gerenciam tudo.
static inline void cc1101SpiEnd() {
    spiCC1101.end();
}

// ============================================================
// SPI PRIMITIVES
// ============================================================

uint8_t cc1101ReadReg(uint8_t reg) {
    cc1101SpiStart();
    digitalWrite(CC1101_CSN, LOW);
    waitMisoReady();
    uint8_t val = spiCC1101.transfer(reg | CC1101_READ_SINGLE);
    val = spiCC1101.transfer(0x00);
    digitalWrite(CC1101_CSN, HIGH);
    cc1101SpiEnd();
    return val;
}

uint8_t cc1101ReadStatus(uint8_t reg) {
    cc1101SpiStart();
    digitalWrite(CC1101_CSN, LOW);
    waitMisoReady();
    spiCC1101.transfer(reg | CC1101_READ_BURST);
    uint8_t val = spiCC1101.transfer(0x00);
    digitalWrite(CC1101_CSN, HIGH);
    cc1101SpiEnd();
    return val;
}

void cc1101WriteReg(uint8_t reg, uint8_t value) {
    cc1101SpiStart();
    digitalWrite(CC1101_CSN, LOW);
    waitMisoReady();
    spiCC1101.transfer(reg);
    spiCC1101.transfer(value);
    digitalWrite(CC1101_CSN, HIGH);
    cc1101SpiEnd();
}

static bool cc1101WriteRegBurst(uint8_t reg, uint8_t* data, uint8_t len) {
    cc1101SpiStart();
    digitalWrite(CC1101_CSN, LOW);
    waitMisoReady();
    spiCC1101.transfer(reg | CC1101_WRITE_BURST);
    for (uint8_t i = 0; i < len; i++) {
        spiCC1101.transfer(data[i]);
    }
    digitalWrite(CC1101_CSN, HIGH);
    cc1101SpiEnd();
    return true;
}

void cc1101SendCommand(uint8_t cmd) {
    cc1101SpiStart();
    digitalWrite(CC1101_CSN, LOW);
    waitMisoReady();
    spiCC1101.transfer(cmd);
    digitalWrite(CC1101_CSN, HIGH);
    cc1101SpiEnd();
}

void cc1101SetFrequency(uint32_t freqHz) {
    uint32_t freqWord = (uint32_t)((freqHz / 26000000.0) * 65536);
    cc1101WriteReg(CC1101_FREQ2, (freqWord >> 16) & 0xFF);
    cc1101WriteReg(CC1101_FREQ1, (freqWord >> 8) & 0xFF);
    cc1101WriteReg(CC1101_FREQ0, freqWord & 0xFF);
}

// ============================================================
// RESET — correção CRÍTICA
// O reset anterior usava spiCC1101.begin() + transfer(), mas se o chip
// está em estado morto, begin() pode não configurar o bus corretamente.
// SOLUÇÃO: fazer o pulso CSN MANUALMENTE (bit-bang) antes de usar SPI.
// Isso é mais lento mas é 100% confiável, independente do estado do chip.
// ============================================================
static bool cc1101Reset() {
    Serial.println("[CC1101] RESET: iniciando...");
    Serial.flush();

    // Fase 1: Pulso CSN manual (sem SPI.begin)
    // Isso garante que o chip veja uma transição CSN limpa,
    // mesmo se o bus SPI estava em estado inconsistente.
    pinMode(CC1101_SCK, OUTPUT);
    pinMode(CC1101_MOSI, OUTPUT);
    pinMode(CC1101_MISO, INPUT);
    pinMode(CC1101_CSN, OUTPUT);

    // Garante estado inicial: SCK HIGH, MOSI LOW, CSN HIGH
    digitalWrite(CC1101_SCK, HIGH);
    digitalWrite(CC1101_MOSI, LOW);
    digitalWrite(CC1101_CSN, HIGH);
    delayMicroseconds(5);

    // Pulso CSN: LOW → delay → HIGH → delay → LOW
    digitalWrite(CC1101_CSN, LOW);
    delay(1);
    digitalWrite(CC1101_CSN, HIGH);
    delay(1);
    digitalWrite(CC1101_CSN, LOW);
    delayMicroseconds(10);

    // Espera MISO LOW (chip pronto para receber SRES)
    // Com timeout para evitar travar se hardware estiver desconectado
    unsigned long t0 = millis();
    while (digitalRead(CC1101_MISO) != LOW) {
        if (millis() - t0 > 100) {
            Serial.println("[CC1101] RESET: timeout esperando MISO LOW (fase 1)");
            digitalWrite(CC1101_CSN, HIGH);
            return false;
        }
        yield();
    }

    // Agora inicializa o SPI e envia SRES
    spiCC1101.begin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CSN);
    
    // CSN já está LOW, MISO já está LOW — envia SRES
    spiCC1101.transfer(CC1101_SRES);

    // Espera MISO LOW novamente (reset em andamento)
    t0 = millis();
    while (digitalRead(CC1101_MISO) != LOW) {
        if (millis() - t0 > 100) {
            Serial.println("[CC1101] RESET: timeout esperando MISO LOW (fase 2)");
            digitalWrite(CC1101_CSN, HIGH);
            spiCC1101.end();
            return false;
        }
        yield();
    }

    digitalWrite(CC1101_CSN, HIGH);
    spiCC1101.end();

    delay(1);

    // Verifica se reset funcionou lendo PARTNUM
    cc1101SpiStart();
    digitalWrite(CC1101_CSN, LOW);
    waitMisoReady();
    spiCC1101.transfer(CC1101_PARTNUM | CC1101_READ_BURST);
    uint8_t partnum = spiCC1101.transfer(0x00);
    digitalWrite(CC1101_CSN, HIGH);
    cc1101SpiEnd();

    Serial.printf("[CC1101] RESET: PARTNUM=0x%02X\n", partnum);
    Serial.flush();

    if (partnum == 0x00 || partnum == 0xFF) {
        // 0x00 = CC1101 (esperado!), 0xFF = sem resposta
        if (partnum == 0xFF) {
            Serial.println("[CC1101] RESET: chip nao responde (0xFF)");
            return false;
        }
    }

    Serial.println("[CC1101] RESET: OK");
    return true;
}

// ============================================================
// CONFIGURAÇÃO — mesma sequência do ELECHOUSE RegConfigSettings()
// + setCCMode(0) + setModulation(2) + setRxBW(500)
// A ÚNICA diferença: PATABLE para OOK (0x00/0xC0)
// ============================================================
static void cc1101ConfigureRegs() {
    // --- setCCMode(0) ---
    cc1101WriteReg(CC1101_IOCFG2,   0x0D);  // GDO2 = serial clk
    cc1101WriteReg(CC1101_IOCFG0,   0x0D);  // GDO0 = serial data
    cc1101WriteReg(CC1101_PKTCTRL0, 0x32);  // Async serial, sem CRC
    cc1101WriteReg(CC1101_MDMCFG3,  0x93);  // Data rate
    cc1101WriteReg(CC1101_MDMCFG4,  0x07);  // Base RX BW (sobrescrito depois)

    // --- setModulation(2) = ASK/OOK ---
    cc1101WriteReg(CC1101_MDMCFG2,  0x30);  // ASK/OOK
    cc1101WriteReg(CC1101_FREND0,   0x11);  // Front-end TX (ASK)

    // --- RegConfigSettings ---
    cc1101WriteReg(CC1101_FSCTRL1,  0x06);  // IF frequency
    cc1101WriteReg(CC1101_MDMCFG1,  0x02);  // Channel spacing
    cc1101WriteReg(CC1101_MDMCFG0,  0xF8);  // Channel spacing
    cc1101WriteReg(CC1101_CHANNR,   0x00);  // Channel 0
    cc1101WriteReg(CC1101_DEVIATN,  0x47);  // Deviation
    cc1101WriteReg(CC1101_FREND1,   0x56);  // Front-end RX
    cc1101WriteReg(CC1101_MCSM0,    0x18);  // PO timeout, PIN ctrl
    cc1101WriteReg(CC1101_FOCCFG,   0x16);  // Freq offset comp
    cc1101WriteReg(CC1101_BSCFG,    0x1C);  // Bit sync
    cc1101WriteReg(CC1101_AGCCTRL2, 0xC7);  // AGC max LNA gain
    cc1101WriteReg(CC1101_AGCCTRL1, 0x00);  // AGC
    cc1101WriteReg(CC1101_AGCCTRL0, 0xB2);  // AGC boundary

    // --- CRÍTICO: FSCAL e TEST ---
    cc1101WriteReg(CC1101_FSCAL3,   0xE9);  // VCO current
    cc1101WriteReg(CC1101_FSCAL2,   0x2A);  // VCO cal range
    cc1101WriteReg(CC1101_FSCAL1,   0x00);  // VCO
    cc1101WriteReg(CC1101_FSCAL0,   0x1F);  // VCO cap array
    cc1101WriteReg(CC1101_FSTEST,   0x59);  // Synth test
    cc1101WriteReg(CC1101_TEST2,    0x81);  // Test settings
    cc1101WriteReg(CC1101_TEST1,    0x35);  // Test settings
    cc1101WriteReg(CC1101_TEST0,    0x09);  // Test settings

    // --- Packet ---
    cc1101WriteReg(CC1101_PKTCTRL1, 0x04);  // Preamble count
    cc1101WriteReg(CC1101_ADDR,     0x00);  // No address check
    cc1101WriteReg(CC1101_PKTLEN,   0x00);  // Packet length

    // --- setRxBW(500) → MDMCFG4 = 0x27 ---
    cc1101WriteReg(CC1101_MDMCFG4,  0x27);

    // PATABLE — OOK: PA[0]=0x00 (OFF), PA[1]=0xC0 (ON)
    uint8_t paTable[8] = {0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    cc1101WriteRegBurst(CC1101_PATABLE, paTable, 8);

    Serial.println("[CC1101] CONFIG: registradores escritos");
}

// ============================================================
// SLEEP — desativa o CC1101 (crystal OFF)
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

// ============================================================
// CALIBRAÇÃO POR BANDA — estilo ELECHOUSE Calibrate()
// O SCAL (strobe 0x33) falha no ESP32. O ELECHOUSE nunca envia SCAL.
// Em vez disso, escreve FSCTRL0 e TEST0 baseado na frequência.
// ============================================================
static void cc1101CalibrateBand(float freqMHz) {
    if (freqMHz >= 300.0f && freqMHz <= 348.0f) {
        // Banda 315 MHz
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
        // Banda 433 MHz
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
        // Banda 868 MHz (baixa)
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
        // Banda 915 MHz
        int fsctrl0_val = (int)(77.0f + (freqMHz - 900.0f) / (928.0f - 900.0f) * (79.0f - 77.0f));
        cc1101WriteReg(CC1101_FSCTRL0, (uint8_t)fsctrl0_val);
        cc1101WriteReg(CC1101_TEST0, 0x09);
        uint8_t s = cc1101ReadReg(CC1101_FSCAL2);
        if (s < 32) cc1101WriteReg(CC1101_FSCAL2, s + 32);
    }
}

// ============================================================
// HELPER: Lê MARCSTATE
// ============================================================
static uint8_t readMarcState() {
    return cc1101ReadStatus(CC1101_MARCSTATE) & 0x1F;
}

// ============================================================
// GoRx — ENTRADA EM MODO RX
//
// BUG ANTIGO: lia MARCSTATE entre SIDLE e SRX. Cada leitura
// adicionava uma transação SPI (begin/wait/transfer/end) que
// perturbava o chip no ESP32. O ELECHOUSE NUNCA faz isso —
// ele faz SIDLE → setFreq → Calibrate → SIDLE → SRX direto,
// sem nenhuma leitura de estado no meio.
//
// NOVA VERSÃO: segue EXATAMENTE o padrão ELECHOUSE SetRx().
// Só lê MARCSTATE DEPOIS do SRX, para verificação final.
// ============================================================
static bool cc1101GoRx(uint32_t freqHz) {
    float freqMHz = freqHz / 1000000.0f;

    // Verifica estado atual — se está em SLEEP, precisa de reset
    uint8_t state = readMarcState();
    Serial.printf("[CC1101] GoRx: estado inicial=0x%02X\n", state);

    if (state == 0x00) {
        Serial.println("[CC1101] GoRx: chip em SLEEP, resetando...");
        if (!cc1101Reset()) return false;
        cc1101ConfigureRegs();
    }

    // === SEQUÊNCIA ELECHOUSE SetRx() ===
    // 1. SIDLE
    cc1101SendCommand(CC1101_SIDLE);

    // 2. Seta frequência
    cc1101SetFrequency(freqHz);

    // 3. Calibração por banda
    cc1101CalibrateBand(freqMHz);

    // 4. SIDLE novamente (igual ELECHOUSE)
    cc1101SendCommand(CC1101_SIDLE);

    // 5. SRX — ENTRADA EM RX
    //    Sem nenhuma leitura de estado entre SIDLE e SRX!
    cc1101SendCommand(CC1101_SRX);

    // 6. Espera settling (calibração VCO + entrada em RX)
    //    O chip precisa de ~1-2ms para calibrar e entrar em RX.
    delay(2);

    // 7. Verifica estado FINAL — só aqui lemos MARCSTATE
    state = readMarcState();
    Serial.printf("[CC1101] GoRx: estado final=0x%02X @ %lu Hz\n", state, freqHz);

    if (state == 0x0D || state == 0x08) {
        // 0x0D = RX, 0x08 = CALIBRATE (transiciona para RX automaticamente)
        Serial.println("[CC1101] GoRx: SUCESSO");
        return true;
    }

    Serial.printf("[CC1101] GoRx: FALHOU, state=0x%02X\n", state);
    return false;
}

// ============================================================
// SetFrequency + Calibrate (para TX e funções que precisam de cal)
// ============================================================
static void cc1101SetFrequencyCalibrated(uint32_t freqHz) {
    float freqMHz = freqHz / 1000000.0f;
    cc1101SendCommand(CC1101_SIDLE);
    delay(1);
    cc1101SetFrequency(freqHz);
    cc1101CalibrateBand(freqMHz);
}

// ============================================================
// WAKE — acorda o CC1101 do SLEEP
// ============================================================
bool cc1101Wake() {
    if (!cc1101Initialized) return false;

    Serial.println("[CC1101] WAKE: acordando modulo...");
    Serial.flush();

    if (!cc1101Reset()) {
        Serial.println("[CC1101] WAKE: reset falhou!");
        cc1101Awake = false;
        return false;
    }

    cc1101ConfigureRegs();

    // ISR no GDO0
    attachInterrupt(digitalPinToInterrupt(CC1101_GDO0), cc1101ISR, CHANGE);

    uint8_t state = readMarcState();
    uint8_t version = cc1101ReadStatus(CC1101_VERSION);
    Serial.printf("[CC1101] WAKE: MARCSTATE=0x%02X, VERSION=0x%02X\n", state, version);

    if (state != 0x01) {
        Serial.printf("[CC1101] WAKE: estado inesperado 0x%02X, tentando SIDLE...\n", state);
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

// ============================================================
// INIT
// ============================================================
bool cc1101Init() {
    Serial.println("[CC1101] Inicializando...");
    Serial.flush();

    pinMode(CC1101_CSN, OUTPUT);
    digitalWrite(CC1101_CSN, HIGH);
    pinMode(CC1101_GDO0, INPUT_PULLUP);
    pinMode(CC1101_GDO2, INPUT);
    delay(10);

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
    Serial.printf("  GDO0 pin  = %d\n", digitalRead(CC1101_GDO0));
    Serial.println("[CC1101] === FIM DIAGNOSTICO ===");
    Serial.flush();

    // Coloca em SLEEP imediatamente
    cc1101Sleep();

    return true;
}

// ============================================================
// CAPTURE - Copiar Sinal
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
            // NÃO envia SCAL — falha no ESP32. Use SIDLE + reconfig.
            cc1101SendCommand(CC1101_SIDLE); delay(1);
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
