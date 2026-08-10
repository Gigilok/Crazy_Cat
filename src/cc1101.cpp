#include <SPI.h>
#include <driver/gpio.h>
#include "config.h"

// ============================================================
// Registradores CC1101
// ============================================================
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
#define CC1101_SFSTXON  0x31
#define CC1101_SXOFF    0x32
#define CC1101_SCAL     0x33
#define CC1101_SRX      0x34
#define CC1101_STX      0x35
#define CC1101_SIDLE    0x36
#define CC1101_SAFC     0x37
#define CC1101_SWOR     0x38
#define CC1101_SPWD     0x39
#define CC1101_SFRX     0x3A
#define CC1101_SFTX     0x3B
#define CC1101_PATABLE  0x3E
#define CC1101_SNOP     0x3D

#define CC1101_READ_SINGLE  0x80
#define CC1101_READ_BURST   0xC0
#define CC1101_WRITE_BURST  0x40

// ============================================================
// Variáveis globais
// ============================================================
bool cc1101Initialized = false;
bool cc1101CopyActive = false;
bool cc1101JammerActive = false;
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

static bool isr_service_installed = false;

uint16_t spec_an_values[64];
uint32_t spec_an_freqs[64];
uint8_t spec_an_idx = 0;
bool spec_an_running = false;

// ============================================================
// ISR
// ============================================================
void IRAM_ATTR cc1101ISR() {
    if (!isr_enabled) return;
    unsigned long now = micros();
    uint8_t gdo0_val = digitalRead(CC1101_GDO0);
    uint8_t data_val;
    if (gdo0_val == HIGH) {
        data_val = digitalRead(CC1101_GDO2);
    } else {
        data_val = gdo0_val;
    }
    if (gdo0_val == HIGH && data_val != isr_last_val) {
        unsigned long dt = now - isr_last_change;
        if (dt > 50 && dt < 100000) {
            if (isr_count < 200) {
                isr_timings[isr_count] = dt;
                isr_count++;
            }
        }
        isr_last_val = data_val;
        isr_last_change = now;
        capture_started = true;
    }
}

// ============================================================
// SPI (usando SPI global = VSPI compartilhado com NRF24)
// ------------------------------------------------------------
// PROBLEMA ORIGINAL (resolvido aqui):
//   O CC1101 e o NRF24 compartilham o mesmo barramento SPI (VSPI).
//   O driver antigo chamava SPI.transfer() direto, sem
//   SPI.beginTransaction()/endTransaction(). Isso causava race
//   condition com a biblioteca RF24 do NRF24, que também assume
//   controle do barramento. Resultado: a leitura de PARTNUM/VERSION
//   retornava 0xFF e o CC1101 era reportado como "nao responde".
//
// CORRECAO (baseada no ESP32-DIV / SmartRC-CC1101):
//   1. Toda operacao SPI do CC1101 eh envelopada por
//      SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0))
//      e SPI.endTransaction(). Isso reconfigura o barramento para
//      4 MHz, MODE0 (CC1101 nao aguenta > 10 MHz e usa MODE0).
//   2. Antes de baixar o CSN do CC1101, subimos o CSN do NRF24
//      (e vice-versa) para garantir que apenas um slave fale no
//      barramento. A biblioteca RF24 soh sobe o CSN quando termina
//      a propria transacao, mas o SPI global pode estar "livre"
//      com o CSN do NRF24 ainda em LOW em casos de abort.
//   3. Esperamos MISO em LOW (CC1101 pronto) antes de transferir.
// ============================================================

// Configuracao SPI para o CC1101: 4 MHz, MODE0, MSB first.
// O CC1101 aguenta ate 10 MHz, mas 4 MHz eh o padrao usado pelo
// ESP32-DIV e garante margem em wiring com fios jumpers.
static const SPISettings CC1101_SPI_SETTINGS(4000000, MSBFIRST, SPI_MODE0);

static void cs_low()  { digitalWrite(CC1101_CSN, LOW); }
static void cs_high() { digitalWrite(CC1101_CSN, HIGH); }

// Sobe o CSN do NRF24 para liberar o barramento MISO.
// Necessario porque o NRF24 compartilha os mesmos SCK/MOSI/MISO.
static void nrf24_release_bus() {
    pinMode(NRF_CSN, OUTPUT);
    digitalWrite(NRF_CSN, HIGH);
    pinMode(NRF_CE, OUTPUT);
    digitalWrite(NRF_CE, LOW);  // CE em LOW = standby, evita TX/RX acidental
}

static bool waitMisoReady() {
    uint32_t start = micros();
    while (digitalRead(CC1101_MISO) != LOW) {
        if (micros() - start > 2000) return false;
    }
    return true;
}

uint8_t cc1101ReadReg(uint8_t reg) {
    nrf24_release_bus();
    SPI.beginTransaction(CC1101_SPI_SETTINGS);
    cs_low();
    waitMisoReady();
    SPI.transfer(reg | CC1101_READ_SINGLE);
    uint8_t val = SPI.transfer(0x00);
    cs_high();
    SPI.endTransaction();
    return val;
}

uint8_t cc1101ReadStatus(uint8_t reg) {
    nrf24_release_bus();
    SPI.beginTransaction(CC1101_SPI_SETTINGS);
    cs_low();
    waitMisoReady();
    SPI.transfer(reg | CC1101_READ_BURST);
    uint8_t val = SPI.transfer(CC1101_SNOP);
    cs_high();
    SPI.endTransaction();
    return val;
}

void cc1101WriteReg(uint8_t reg, uint8_t value) {
    nrf24_release_bus();
    SPI.beginTransaction(CC1101_SPI_SETTINGS);
    cs_low();
    waitMisoReady();
    SPI.transfer(reg);
    SPI.transfer(value);
    cs_high();
    SPI.endTransaction();
}

static void cc1101WriteRegBurst(uint8_t reg, uint8_t* data, uint8_t len) {
    nrf24_release_bus();
    SPI.beginTransaction(CC1101_SPI_SETTINGS);
    cs_low();
    waitMisoReady();
    SPI.transfer(reg | CC1101_WRITE_BURST);
    for (uint8_t i = 0; i < len; i++) SPI.transfer(data[i]);
    cs_high();
    SPI.endTransaction();
}

void cc1101SendCommand(uint8_t cmd) {
    nrf24_release_bus();
    SPI.beginTransaction(CC1101_SPI_SETTINGS);
    cs_low();
    waitMisoReady();
    SPI.transfer(cmd);
    cs_high();
    SPI.endTransaction();
}

// ============================================================
// Funções auxiliares
// ============================================================
void cc1101SetFrequency(uint32_t freqHz) {
    uint32_t freqWord = (uint32_t)((freqHz / 26000000.0) * 65536);
    cc1101WriteReg(CC1101_FREQ2, (freqWord >> 16) & 0xFF);
    cc1101WriteReg(CC1101_FREQ1, (freqWord >> 8) & 0xFF);
    cc1101WriteReg(CC1101_FREQ0, freqWord & 0xFF);
}

static void cc1101CalibrateBand(float freqMHz) {
    if (freqMHz >= 300.0f && freqMHz <= 348.0f) {
        int fsctrl0_val = (int)(24.0f + (freqMHz - 300.0f) / (348.0f - 300.0f) * (28.0f - 24.0f));
        cc1101WriteReg(CC1101_FSCTRL0, (uint8_t)fsctrl0_val);
        if (freqMHz < 322.88f) cc1101WriteReg(CC1101_TEST0, 0x0B);
        else {
            cc1101WriteReg(CC1101_TEST0, 0x09);
            uint8_t s = cc1101ReadReg(CC1101_FSCAL2);
            if (s < 32) cc1101WriteReg(CC1101_FSCAL2, s + 32);
        }
    } else if (freqMHz >= 378.0f && freqMHz <= 464.0f) {
        int fsctrl0_val = (int)(31.0f + (freqMHz - 378.0f) / (464.0f - 378.0f) * (38.0f - 31.0f));
        cc1101WriteReg(CC1101_FSCTRL0, (uint8_t)fsctrl0_val);
        if (freqMHz < 430.5f) cc1101WriteReg(CC1101_TEST0, 0x0B);
        else {
            cc1101WriteReg(CC1101_TEST0, 0x09);
            uint8_t s = cc1101ReadReg(CC1101_FSCAL2);
            if (s < 32) cc1101WriteReg(CC1101_FSCAL2, s + 32);
        }
    } else if (freqMHz >= 779.0f && freqMHz <= 899.99f) {
        int fsctrl0_val = (int)(65.0f + (freqMHz - 779.0f) / (899.0f - 779.0f) * (76.0f - 65.0f));
        cc1101WriteReg(CC1101_FSCTRL0, (uint8_t)fsctrl0_val);
        if (freqMHz < 861.0f) cc1101WriteReg(CC1101_TEST0, 0x0B);
        else {
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

static void cc1101SetFrequencyCalibrated(uint32_t freqHz) {
    float freqMHz = freqHz / 1000000.0f;
    cc1101SendCommand(CC1101_SIDLE);
    delay(1);
    cc1101SetFrequency(freqHz);
    cc1101CalibrateBand(freqMHz);
}

// ============================================================
// Reset e configuração
// ============================================================
// Reset do CC1101 conforme datasheet:
//  1. Sobe CSN (HIGH) por >40us
//  2. Baixa CSN (LOW) por >40us
//  3. Sobe CSN novamente e espera MISO ir para LOW
//  4. Envia strobe SRES (0x30)
//  5. Espera MISO voltar para LOW (chip resetou)
//  6. Sobe CSN
//
// IMPORTANTE: NAO chamar pinMode() nos pinos SCK/MOSI/MISO!
// No Arduino-ESP32, isso desconecta o periférico SPI da GPIO matrix
// e o SPI para de funcionar. Só configuramos o CSN (manual).
static bool cc1101Reset() {
    nrf24_release_bus();

    // Apenas CSN é controlado manualmente. SCK/MOSI/MISO sao do periferico SPI.
    pinMode(CC1101_CSN, OUTPUT);

    SPI.beginTransaction(CC1101_SPI_SETTINGS);

    // Pulso de reset hardware-style (datasheet sec. 10.1)
    cs_high(); delayMicroseconds(100);
    cs_low();  delayMicroseconds(100);
    cs_high(); delayMicroseconds(100);
    cs_low();

    // Espera MISO baixar (chip pronto pra receber comando)
    uint32_t start = micros();
    while (digitalRead(CC1101_MISO) != LOW) {
        if (micros() - start > 10000) {
            Serial.println(F("[CC1101] Reset: timeout esperando MISO"));
            cs_high();
            SPI.endTransaction();
            return false;
        }
    }

    SPI.transfer(CC1101_SRES);

    // Espera MISO voltar a LOW (reset completou — pode demorar ate ~150us)
    start = micros();
    while (digitalRead(CC1101_MISO) != LOW) {
        if (micros() - start > 10000) break;
    }

    cs_high();
    SPI.endTransaction();
    delay(2);
    return true;
}

static void cc1101ConfigureRegs() {
    // IOCFG0 = 0x2E: Transparent async serial data output (RX data via GDO0)
    // Essencial para captura raw de sinais. O valor antigo 0x0D era
    // "serial clock output" e NAO fornecia dados no GDO0, por isso o
    // ISR nunca disparava.
    cc1101WriteReg(CC1101_IOCFG2,   0x0D);
    cc1101WriteReg(CC1101_IOCFG0,   0x2E);
    cc1101WriteReg(CC1101_PKTCTRL0, 0x32);
    cc1101WriteReg(CC1101_MDMCFG3,  0x93);
    cc1101WriteReg(CC1101_MDMCFG2,  0x30);   // ASK/OOK modulation
    cc1101WriteReg(CC1101_FREND0,   0x11);
    cc1101WriteReg(CC1101_FSCTRL1,  0x06);
    cc1101WriteReg(CC1101_MDMCFG4,  0x27);
    cc1101WriteReg(CC1101_MDMCFG1,  0x02);
    cc1101WriteReg(CC1101_MDMCFG0,  0xF8);
    cc1101WriteReg(CC1101_CHANNR,   0x00);
    cc1101WriteReg(CC1101_DEVIATN,  0x47);
    cc1101WriteReg(CC1101_FREND1,   0x56);
    cc1101WriteReg(CC1101_MCSM0,    0x18);
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
    cc1101WriteReg(CC1101_PKTCTRL1, 0x00);
    cc1101WriteReg(CC1101_ADDR,     0x00);
    cc1101WriteReg(CC1101_PKTLEN,   0x00);
    uint8_t paTable[8] = {0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    cc1101WriteRegBurst(CC1101_PATABLE, paTable, 8);
}

// ============================================================
// Inicialização
// ============================================================
bool cc1101Init() {
    Serial.println(F("[CC1101] Inicializando..."));
    Serial.flush();

    if (!isr_service_installed) {
        esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
        if (err == ESP_OK) isr_service_installed = true;
        else if (err == ESP_ERR_INVALID_STATE) isr_service_installed = true;
    }

    // Garante estado deterministico dos pinos CC1101
    // IMPORTANTE: NAO fazer pinMode() em SCK/MOSI/MISO — isso desconecta
    // o periferico SPI da GPIO matrix no Arduino-ESP32 e quebra o SPI.
    // Apenas CSN, GDO0 e GDO2 sao controlados manualmente.
    pinMode(CC1101_CSN, OUTPUT);
    digitalWrite(CC1101_CSN, HIGH);
    pinMode(CC1101_GDO0, INPUT);
    pinMode(CC1101_GDO2, INPUT);

    // Libera o NRF24 do barramento SPI compartilhado antes de qualquer
    // coisa. Sem isso, o MISO pode estar sendo puxado LOW pelo NRF24 e
    // o CC1101 nunca responde.
    nrf24_release_bus();
    delay(10);

    // Tentativa de reset + leitura com retry (ate 3x).
    // Se a primeira tentativa falhar (modulo ainda acordando, wiring ruidoso,
    // etc), tentamos novamente apos um delay maior.
    uint8_t partnum = 0xFF;
    uint8_t version = 0xFF;

    for (int attempt = 1; attempt <= 3; attempt++) {
        Serial.print(F("[CC1101] Reset attempt "));
        Serial.print(attempt);
        Serial.println(F("/3"));
        Serial.flush();

        if (!cc1101Reset()) {
            Serial.println(F("[CC1101] Reset falhou (timeout MISO)"));
            delay(50);
            continue;
        }

        delay(5);

        partnum = cc1101ReadStatus(CC1101_PARTNUM);
        version = cc1101ReadStatus(CC1101_VERSION);
        Serial.print(F("[CC1101] PARTNUM=0x"));
        Serial.print(partnum, HEX);
        Serial.print(F(" VERSION=0x"));
        Serial.println(version, HEX);
        Serial.flush();

        // CC1101 genuino (datasheet):
        //   PARTNUM  = 0x00 (sempre 0x00 para CC1101 — identifica o chip)
        //   VERSION  = 0x04 (rev B) ou 0x14 (rev E/F) — qualquer valor
        //              != 0x00 e != 0xFF indica que o chip respondeu.
        //
        // Validacao correta:
        //   - partnum == 0x00 E version != 0x00 E version != 0xFF -> OK
        //   - partnum == 0xFF -> MISO em HIGH (chip nao responde / NRF24 segurando)
        //   - version == 0xFF -> idem
        //   - version == 0x00 -> MISO em LOW (chip nao alimentado ou CSN nao chega)
        if (partnum == 0x00 && version != 0x00 && version != 0xFF) {
            Serial.print(F("[CC1101] CC1101 rev "));
            Serial.print((version & 0xF0) >> 4, HEX);
            Serial.print(F("."));
            Serial.println(version & 0x0F, HEX);
            break;  // sucesso
        }

        // Falhou — espera e tenta de novo
        delay(100);
    }

    // Validacao final: CC1101 respondeu?
    // PARTNUM deve ser 0x00 e VERSION deve ser != 0x00 e != 0xFF
    if (partnum != 0x00 || version == 0x00 || version == 0xFF) {
        Serial.println(F("[CC1101] FAIL: modulo nao responde"));
        Serial.println(F("[CC1101] Valores lidos:"));
        Serial.print(F("  PARTNUM = 0x"));
        Serial.print(partnum, HEX);
        Serial.print(F(" (esperado: 0x00)"));
        if (partnum == 0xFF) Serial.print(F(" <- MISO em HIGH (barramento ocupado)"));
        Serial.println();
        Serial.print(F("  VERSION = 0x"));
        Serial.print(version, HEX);
        Serial.print(F(" (esperado: 0x04 ou 0x14)"));
        if (version == 0xFF) Serial.print(F(" <- MISO em HIGH"));
        else if (version == 0x00) Serial.print(F(" <- MISO em LOW (chip sem alimentacao?)"));
        Serial.println();
        Serial.println(F("[CC1101] Verifique:"));
        Serial.println(F("  - Fios SCK/MOSI/MISO/CSN conectados"));
        Serial.println(F("  - Alimentacao 3.3V (NAO 5V!)"));
        Serial.println(F("  - NRF24 desconectado durante teste (conflito SPI)"));
        return false;
    }

    Serial.println(F("[CC1101] Modulo respondeu. Configurando registradores..."));
    cc1101ConfigureRegs();
    cc1101Initialized = true;
    cc1101SendCommand(CC1101_SIDLE);
    Serial.println(F("[CC1101] OK - pronto para uso"));
    return true;
}

// ============================================================
// Sleep / Wake
// ============================================================
void cc1101Sleep() {
    if (!cc1101Initialized) return;
    isr_enabled = false;
    detachInterrupt(digitalPinToInterrupt(CC1101_GDO0));
    cc1101SendCommand(CC1101_SIDLE);
    cc1101SendCommand(CC1101_SPWD);
}

bool cc1101Wake() {
    if (!cc1101Initialized) return false;

    // Tenta acordar do SLEEP (SPWD). Se o chip estiver em IDLE, o MISO
    // ja esta em LOW e o SIDLE eh aceito imediatamente.
    nrf24_release_bus();
    SPI.beginTransaction(CC1101_SPI_SETTINGS);

    // Baixa CSN para acordar o chip (se estiver em SPWD)
    digitalWrite(CC1101_CSN, LOW);

    // Espera MISO baixar (chip acordou)
    uint32_t start = micros();
    while (digitalRead(CC1101_MISO) != LOW) {
        if (micros() - start > 10000) break;  // 10ms timeout
    }

    // Envia SIDLE para garantir estado conhecido
    SPI.transfer(CC1101_SIDLE);
    digitalWrite(CC1101_CSN, HIGH);
    SPI.endTransaction();

    delay(1);

    // Reconfigura registradores (garantir, caso o chip tenha resetado)
    cc1101ConfigureRegs();

    pinMode(CC1101_GDO0, INPUT);
    pinMode(CC1101_GDO2, INPUT);
    attachInterrupt(digitalPinToInterrupt(CC1101_GDO0), cc1101ISR, CHANGE);
    return true;
}

// ============================================================
// GoRx - entra em modo RX
// ============================================================
// Baseado no fluxo do ESP32-DIV / SmartRC:
//   setSidle() -> setMHZ() -> SetRx()
// Adicionamos SCAL (calibrate) antes de SRX para garantir que o
// sintetizador de frequencia esteja calibrado, e validamos que
// MARCSTATE == 0x0D (RX) apos SRX.
static bool cc1101GoRx(uint32_t freqHz) {
    float freqMHz = freqHz / 1000000.0f;

    // 0. Validacao: ler PARTNUM para confirmar que o SPI esta funcionando
    uint8_t partnum_check = cc1101ReadStatus(CC1101_PARTNUM);
    if (partnum_check != 0x00) {
        Serial.print(F("[CC1101] GoRx: AVISO - PARTNUM=0x"));
        Serial.print(partnum_check, HEX);
        Serial.println(F(" (esperado 0x00). SPI pode estar instavel."));
    }

    // 1. Sair de qualquer estado -> IDLE
    cc1101SendCommand(CC1101_SIDLE);
    delay(2);

    // 2. Configurar frequencia + calibracao de banda
    cc1101SetFrequency(freqHz);
    cc1101CalibrateBand(freqMHz);

    // 3. Garantir IOCFG0 = 0x2E (transparent async serial data output)
    //    Isso faz o GDO0 outputar os dados raw recebidos (necessario para ISR)
    cc1101WriteReg(CC1101_IOCFG0, 0x2E);

    // 4. Calibrar sintetizador (SCAL) antes de SRX
    cc1101SendCommand(CC1101_SCAL);
    delayMicroseconds(500);

    // 5. Entrar em RX
    cc1101SendCommand(CC1101_SRX);
    delayMicroseconds(500);

    // 6. Verificar MARCSTATE (deve ser 0x0D = RX)
    unsigned long start = micros();
    uint8_t marcstate = 0;
    do {
        marcstate = cc1101ReadStatus(CC1101_MARCSTATE) & 0x1F;
        if (marcstate == 0x0D) break;
        delayMicroseconds(500);
    } while (micros() - start < 50000);  // 50ms timeout

    // 7. Ler RSSI
    uint8_t rssiRaw = cc1101ReadStatus(CC1101_RSSI);
    int rssiDbm = (rssiRaw >= 128) ? ((int)rssiRaw - 256) / 2 - 74 : (int)rssiRaw / 2 - 74;

    Serial.print(F("[CC1101] GoRx "));
    Serial.print(freqHz);
    Serial.print(F(" Hz | MARCSTATE=0x"));
    Serial.print(marcstate, HEX);
    Serial.print(F(" RSSI="));
    Serial.print(rssiDbm);
    Serial.print(F(" GDO0="));
    Serial.print(digitalRead(CC1101_GDO0));
    Serial.print(F(" GDO2="));
    Serial.println(digitalRead(CC1101_GDO2));

    if (marcstate != 0x0D) {
        Serial.println(F("[CC1101] GoRx: FALHOU - MARCSTATE != 0x0D (RX)"));
    }

    return marcstate == 0x0D;
}

// ============================================================
// Captura RAW
// ============================================================
void cc1101StartCapture() {
    if (!cc1101Initialized) return;
    if (!cc1101Wake()) return;
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

    cc1101GoRx(currentCapture.frequency);

    isr_last_val = digitalRead(CC1101_GDO2);
    isr_last_change = micros();
    isr_enabled = true;
}

void cc1101CaptureLoop() {
    if (!cc1101CopyActive) return;
    unsigned long now = micros();
    unsigned long nowMs = millis();

    if (capture_state == STATE_HOPPING) {
        if (isr_count > 5) {
            capture_state = STATE_LOCKED;
            Serial.printf("[CC1101] LOCKED freq=%luM, pulses=%d\n", currentCapture.frequency / 1000000, isr_count);
        }
        else if (nowMs - lastFreqSwitch > 1000) {
            if (isr_count == 0 && !capture_started) {
                Serial.printf("[CC1101] Hop %luM: GDO0=%d GDO2=%d ISR=0 ISRsvc=%s\n",
                    currentCapture.frequency / 1000000,
                    digitalRead(CC1101_GDO0), digitalRead(CC1101_GDO2),
                    isr_service_installed ? "OK" : "FAIL!");
            }
            isr_enabled = false;
            isr_count = 0;
            capture_started = false;
            currentFreqIndex = (currentFreqIndex + 1) % 4;
            currentCapture.frequency = captureFreqs[currentFreqIndex];
            cc1101GoRx(currentCapture.frequency);
            isr_last_val = digitalRead(CC1101_GDO2);
            isr_last_change = micros();
            isr_enabled = true;
            lastFreqSwitch = nowMs;
        }
    }
    else if (capture_state == STATE_LOCKED) {
        isr_count = 0;
        isr_last_val = digitalRead(CC1101_GDO2);
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
            isr_last_val = digitalRead(CC1101_GDO2);
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
            isr_last_val = digitalRead(CC1101_GDO2);
            isr_last_change = micros();
            isr_enabled = true;
        }
    }
}

void cc1101StopCapture() {
    isr_enabled = false;
    cc1101CopyActive = false;
    currentCapture.active = false;
    cc1101SendCommand(CC1101_SIDLE);
}

uint8_t cc1101GetPulseCount() { return isr_count; }
uint32_t cc1101GetCurrentFreq() { return currentCapture.frequency / 1000000; }
uint8_t cc1101GetPinState() { return digitalRead(CC1101_GDO0); }

// ============================================================
// Replay
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
    cc1101WriteReg(CC1101_IOCFG0, 0x2E);
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
    cc1101WriteReg(CC1101_IOCFG0, 0x2E);
    cc1101SendCommand(CC1101_SIDLE);
    cc1101Sleep();
}

// ============================================================
// Jammer
// ============================================================
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
    cc1101JammerActive = true;
}

void cc1101StopSubGHzJammer() {
    if (!cc1101Initialized) return;
    digitalWrite(CC1101_GDO0, LOW);
    pinMode(CC1101_GDO0, INPUT_PULLUP);
    cc1101WriteReg(CC1101_IOCFG0, 0x2E);
    cc1101SendCommand(CC1101_SIDLE);
    cc1101Sleep();
    cc1101JammerActive = false;
}

// ============================================================
// RollJam
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
    cc1101WriteReg(CC1101_IOCFG0, 0x2E);
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
        if (digitalRead(CC1101_GDO2) == HIGH) {
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
    cc1101RollJamActive = false;
    isr_enabled = false;
    digitalWrite(CC1101_GDO0, LOW);
    pinMode(CC1101_GDO0, INPUT);
    cc1101WriteReg(CC1101_IOCFG0, 0x2E);
    cc1101SendCommand(CC1101_SIDLE);
    cc1101Sleep();
}

// ============================================================
// Analisador de espectro
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
    cc1101SendCommand(CC1101_SRX);
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
// Gerenciamento de sinais
// ============================================================
void cc1101ClearSavedSignals() {
    savedSignalCount = 0;
    memset(savedSignals, 0, sizeof(savedSignals));
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
// Transmissão RAW
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
    cc1101WriteReg(CC1101_IOCFG0, 0x2E);
    cc1101SendCommand(CC1101_SIDLE);
    cc1101Sleep();
}

// ============================================================
// Status e acesso
// ============================================================
bool cc1101IsAvailable() { return cc1101Initialized; }
bool cc1101IsCapturing() { return cc1101CopyActive; }
uint8_t cc1101GetSavedCount() { return savedSignalCount; }
SignalData* cc1101GetSignal(uint8_t index) {
    if (index < savedSignalCount) return &savedSignals[index];
    return nullptr;
}

// ============================================================
// Drone RSSI
// ============================================================
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
