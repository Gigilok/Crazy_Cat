// ============================================================
// CC1101 - Driver baseado em SmartRC-CC1101 (ELECHOUSE)
// ------------------------------------------------------------
// REESCRITO para replicar exatamente o padrao do ESP32-DIV:
//   - reclaimSpiBus() UMA VEZ ao iniciar feature CC1101
//   - restoreNrf24Spi() UMA VEZ ao sair do feature CC1101
//   - Sem restoreNrf24Spi() entre operacoes individuais
//   - Usa RCSwitch para recepcao (igual ao DIV)
//   - Nao verifica MARCSTATE (o DIV nao verifica)
//   - Nao chama goSleep()/SPWD (nenhum dos dois chama)
// ============================================================

#include <SPI.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <RCSwitch.h>
#include "config.h"
#include "driver/gpio.h"

// ============================================================
// Variaveis globais (interface publica mantida)
// ============================================================
bool cc1101Initialized = false;
bool cc1101CopyActive = false;
bool cc1101JammerActive = false;
bool cc1101RollJamActive = false;
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

// Buffer de captura raw (mantido para compatibilidade)
volatile uint16_t isr_timings[200];
volatile uint8_t isr_count = 0;
volatile unsigned long isr_last_change = 0;
volatile uint8_t isr_last_val = 0;
volatile bool capture_started = false;
volatile bool isr_enabled = false;

// RCSwitch para decodificar sinais (igual ao DIV)
RCSwitch mySwitch = RCSwitch();
static bool rcswitch_armed = false;

// Analyzer de espectro
uint16_t spec_an_values[64];
uint32_t spec_an_freqs[64];
uint8_t spec_an_idx = 0;
bool spec_an_running = false;

// Flag: SPI esta configurado para CC1101?
static bool spi_owned_by_cc1101 = false;

// ============================================================
// ISR para captura raw (fallback quando RCSwitch nao resolve)
// ============================================================
void IRAM_ATTR cc1101ISR() {
    if (!isr_enabled) return;
    unsigned long now = micros();
    uint8_t gdo0_val = digitalRead(CC1101_GDO0);
    if (gdo0_val != isr_last_val) {
        unsigned long dt = now - isr_last_change;
        if (dt > 50 && dt < 100000) {
            if (isr_count < 200) {
                isr_timings[isr_count] = dt;
                isr_count++;
            }
        }
        isr_last_val = gdo0_val;
        isr_last_change = now;
        capture_started = true;
    }
}

// ============================================================
// reclaimSpiBus - baseado no reclaimSharedSpiBus() do DIV
// Chamada UMA VEZ ao entrar no feature CC1101.
// Sobe todos os CS, faz SPI.end() + SPI.begin() com pinos CC1101.
// ============================================================
static void reclaimSpiBus() {
    pinMode(NRF_CSN, OUTPUT);
    digitalWrite(NRF_CSN, HIGH);
    pinMode(NRF_CE, OUTPUT);
    digitalWrite(NRF_CE, LOW);
    pinMode(CC1101_CSN, OUTPUT);
    digitalWrite(CC1101_CSN, HIGH);

    SPI.end();
    SPI.begin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CSN);
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    SPI.setFrequency(4000000);
    delay(2);
    spi_owned_by_cc1101 = true;
}

// restoreNrf24Spi - chamada UMA VEZ ao SAIR do feature CC1101
static void restoreNrf24Spi() {
    if (!spi_owned_by_cc1101) return;
    pinMode(CC1101_CSN, OUTPUT);
    digitalWrite(CC1101_CSN, HIGH);
    pinMode(NRF_CSN, OUTPUT);
    digitalWrite(NRF_CSN, HIGH);
    SPI.end();
    SPI.begin(NRF_SCK, NRF_MISO, NRF_MOSI, -1);
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    SPI.setFrequency(8000000);
    spi_owned_by_cc1101 = false;
}

// Helper legacy
static void nrf24_release_bus() {
    if (!spi_owned_by_cc1101) reclaimSpiBus();
}

// ============================================================
// API publica - mantida para compatibilidade
// Estas funcoes sao chamadas por wifi_attacks.cpp (drone jammer)
// e settings.cpp. Elas fazem reclaim/restore apenas se necessario.
// ============================================================
void cc1101SetFrequency(uint32_t freqHz) {
    if (!cc1101Initialized) return;
    nrf24_release_bus();
    ELECHOUSE_cc1101.setMHZ(freqHz / 1000000.0);
}

void cc1101WriteReg(uint8_t reg, uint8_t value) {
    if (!cc1101Initialized) return;
    nrf24_release_bus();
    ELECHOUSE_cc1101.SpiWriteReg(reg, value);
}

void cc1101SendCommand(uint8_t cmd) {
    if (!cc1101Initialized) return;
    nrf24_release_bus();
    ELECHOUSE_cc1101.SpiStrobe(cmd);
}

uint8_t cc1101ReadStatus(uint8_t reg) {
    if (!cc1101Initialized) return 0;
    nrf24_release_bus();
    return ELECHOUSE_cc1101.SpiReadStatus(reg);
}

// ============================================================
// Inicializacao - replica exatamente o ReplayAttackSetup do DIV
// ============================================================
bool cc1101Init() {
    Serial.println(F("[CC1101] Inicializando com SmartRC..."));
    Serial.flush();

    // Configura pinos GDO
    pinMode(CC1101_GDO0, INPUT);
    pinMode(CC1101_GDO2, INPUT);

    // Reclaim SPI UMA VEZ
    reclaimSpiBus();

    // Configura SmartRC (igual ao DIV)
    ELECHOUSE_cc1101.setSpiPin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CSN);
    ELECHOUSE_cc1101.setGDO(CC1101_GDO0, CC1101_GDO2);

    // Init + config (igual ao DIV)
    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setCCMode(0);        // modo transparent
    ELECHOUSE_cc1101.setModulation(2);    // ASK/OOK
    ELECHOUSE_cc1101.setRxBW(500.0);      // 500 kHz

    // Verificacao (opcional - o DIV nao faz)
    uint8_t partnum = ELECHOUSE_cc1101.SpiReadStatus(0x30);
    uint8_t version = ELECHOUSE_cc1101.SpiReadStatus(0x31);
    Serial.print(F("[CC1101] PARTNUM=0x"));
    Serial.print(partnum, HEX);
    Serial.print(F(" VERSION=0x"));
    Serial.println(version, HEX);

    if (partnum != 0x00 || version == 0x00 || version == 0xFF) {
        Serial.println(F("[CC1101] FAIL: modulo nao responde"));
        restoreNrf24Spi();
        return false;
    }

    cc1101Initialized = true;
    ELECHOUSE_cc1101.setSidle();
    Serial.println(F("[CC1101] OK - pronto para uso"));

    // Restaurar SPI para NRF24 (init e uma feature unica)
    restoreNrf24Spi();
    return true;
}

// ============================================================
// Sleep / Wake
// NAO usamos goSleep()/SPWD. Apenas setSidle().
// ============================================================
void cc1101Sleep() {
    if (!cc1101Initialized) return;
    isr_enabled = false;
    if (rcswitch_armed) {
        mySwitch.disableReceive();
        rcswitch_armed = false;
    }
    detachInterrupt(digitalPinToInterrupt(CC1101_GDO0));
    nrf24_release_bus();
    ELECHOUSE_cc1101.setSidle();
    // NAO chamar goSleep()/SPWD
    // Restaurar SPI para NRF24 ao sair do feature
    restoreNrf24Spi();
}

bool cc1101Wake() {
    if (!cc1101Initialized) return false;
    reclaimSpiBus();

    // Re-configurar SmartRC (caso pinos tenham mudado)
    ELECHOUSE_cc1101.setSpiPin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CSN);
    ELECHOUSE_cc1101.setGDO(CC1101_GDO0, CC1101_GDO2);

    // Init + config (igual ao DIV)
    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setCCMode(0);
    ELECHOUSE_cc1101.setModulation(2);
    ELECHOUSE_cc1101.setRxBW(500.0);

    pinMode(CC1101_GDO0, INPUT);
    pinMode(CC1101_GDO2, INPUT);

    // NAO restaurar SPI aqui - quem chamou cc1101Wake vai usar CC1101
    return true;
}

// ============================================================
// GoRx - entra em modo RX
// Replica exatamente o tuneToIndex do DIV:
//   setSidle() -> setMHZ() -> SetRx()
// Sem pre-check, sem MARCSTATE, sem retry.
// ============================================================
static bool cc1101GoRx(uint32_t freqHz) {
    if (!cc1101Initialized) return false;
    // SPI ja foi reclaimed por cc1101Wake() - nao fazer nada aqui

    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.setMHZ(freqHz / 1000000.0);
    ELECHOUSE_cc1101.SetRx();
    delay(5);

    // Log simples (nao bloqueia)
    uint8_t marcstate = ELECHOUSE_cc1101.SpiReadStatus(0x35) & 0x1F;
    int rssi = ELECHOUSE_cc1101.getRssi();
    Serial.print(F("[CC1101] GoRx "));
    Serial.print(freqHz);
    Serial.print(F(" Hz | MARC=0x"));
    Serial.print(marcstate, HEX);
    Serial.print(F(" RSSI="));
    Serial.print(rssi);
    Serial.print(F(" GDO0="));
    Serial.println(digitalRead(CC1101_GDO0));

    return true;
}

// ============================================================
// Captura RAW via RCSwitch (igual ao DIV)
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
    isr_count = 0;
    capture_started = false;

    cc1101GoRx(currentCapture.frequency);

    // Armar RCSwitch no GDO0 (igual ao DIV arma no GDO2)
    pinMode(CC1101_GDO0, INPUT);
    mySwitch.enableReceive(CC1101_GDO0);
    rcswitch_armed = true;

    // Tambem armar ISR raw para captura de timings
    isr_last_val = digitalRead(CC1101_GDO0);
    isr_last_change = micros();
    isr_enabled = true;
    attachInterrupt(digitalPinToInterrupt(CC1101_GDO0), cc1101ISR, CHANGE);
}

void cc1101CaptureLoop() {
    if (!cc1101CopyActive) return;
    unsigned long now = micros();
    unsigned long nowMs = millis();

    // Verificar se RCSwitch decodificou algo
    if (rcswitch_armed && mySwitch.available()) {
        uint32_t value = mySwitch.getReceivedValue();
        uint16_t bits = mySwitch.getReceivedBitlength();
        uint16_t proto = mySwitch.getReceivedProtocol();
        mySwitch.resetAvailable();

        if (value > 0 && savedSignalCount < MAX_SAVED_SIGNALS) {
            SignalData* sig = &savedSignals[savedSignalCount];
            sig->length = 1;  // RCSwitch decodifica, nao precisa de timings
            sig->frequency = currentCapture.frequency;
            sig->modulation = 0;
            sig->valid = true;
            // Guardar value e bits nos primeiros timings para compatibilidade
            sig->timings[0] = (uint16_t)(value & 0xFFFF);
            sig->timings[1] = (uint16_t)(value >> 16);
            sig->timings[2] = bits;
            sig->timings[3] = proto;
            snprintf(sig->name, 16, "Cod %luM", sig->frequency / 1000000);
            savedSignalCount++;
            Serial.print(F("[CC1101] RCSwitch: val="));
            Serial.print(value);
            Serial.print(F(" bits="));
            Serial.print(bits);
            Serial.print(F(" proto="));
            Serial.println(proto);
        }
    }

    // Captura raw via ISR (fallback)
    if (capture_state == STATE_HOPPING) {
        if (isr_count > 5) {
            capture_state = STATE_LOCKED;
            Serial.print(F("[CC1101] LOCKED freq="));
            Serial.print(currentCapture.frequency / 1000000);
            Serial.print(F("M, pulses="));
            Serial.println(isr_count);
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
            cc1101CopyActive = false;
            detachInterrupt(digitalPinToInterrupt(CC1101_GDO0));
            if (rcswitch_armed) {
                mySwitch.disableReceive();
                rcswitch_armed = false;
            }
            ELECHOUSE_cc1101.setSidle();
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
                Serial.print(F("[CC1101] Capturado: "));
                Serial.print(sig->length);
                Serial.print(F(" pulsos, "));
                Serial.print(totalDuration);
                Serial.println(F(" us"));
            }
            cc1101Sleep();
        }
        else if (totalTimeout) {
            isr_enabled = false;
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
    detachInterrupt(digitalPinToInterrupt(CC1101_GDO0));
    if (rcswitch_armed) {
        mySwitch.disableReceive();
        rcswitch_armed = false;
    }
    cc1101Sleep();
}

uint8_t cc1101GetPulseCount() { return isr_count; }
uint32_t cc1101GetCurrentFreq() { return currentCapture.frequency / 1000000; }
uint8_t cc1101GetPinState() { return digitalRead(CC1101_GDO0); }

// ============================================================
// Replay - transmite sinal gravado via GDO0
// ============================================================
void cc1101ReplaySignal(uint8_t index) {
    if (index >= savedSignalCount || !savedSignals[index].valid) return;
    if (!cc1101Initialized) return;
    if (!cc1101Wake()) return;

    SignalData* sig = &savedSignals[index];

    // Se foi capturado via RCSwitch (timings[2] = bits), usar RCSwitch.send()
    if (sig->length == 1 && sig->timings[2] > 0) {
        uint32_t value = (uint32_t)sig->timings[1] << 16 | sig->timings[0];
        uint16_t bits = sig->timings[2];
        uint16_t proto = sig->timings[3];

        ELECHOUSE_cc1101.setSidle();
        ELECHOUSE_cc1101.setMHZ(sig->frequency / 1000000.0);
        ELECHOUSE_cc1101.SetTx();
        delay(2);

        mySwitch.setProtocol(proto);
        mySwitch.enableTransmit(CC1101_GDO0);
        mySwitch.send(value, bits);
        mySwitch.disableTransmit();

        ELECHOUSE_cc1101.setSidle();
    } else {
        // Captura raw via ISR - transmitir timings manualmente
        ELECHOUSE_cc1101.setSidle();
        ELECHOUSE_cc1101.setMHZ(sig->frequency / 1000000.0);
        ELECHOUSE_cc1101.SpiWriteReg(0x02, 0x2E);  // IOCFG0 = TX async
        ELECHOUSE_cc1101.SetTx();
        delay(2);

        pinMode(CC1101_GDO0, OUTPUT);
        digitalWrite(CC1101_GDO0, LOW);
        for (int i = 0; i < sig->length; i++) {
            digitalWrite(CC1101_GDO0, i % 2 == 0 ? HIGH : LOW);
            delayMicroseconds(sig->timings[i]);
        }
        digitalWrite(CC1101_GDO0, LOW);
        pinMode(CC1101_GDO0, INPUT_PULLUP);
        ELECHOUSE_cc1101.setSidle();
    }
    cc1101Sleep();
}

void cc1101SendBruteForceCode(uint32_t code, uint32_t freq) {
    if (!cc1101Initialized) return;
    if (!cc1101Wake()) return;

    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.setMHZ(freq / 1000000.0);
    ELECHOUSE_cc1101.SpiWriteReg(0x02, 0x2E);  // IOCFG0 = TX async
    ELECHOUSE_cc1101.SetTx();
    delay(2);

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
    ELECHOUSE_cc1101.setSidle();
    cc1101Sleep();
}

// ============================================================
// Jammer Sub-GHz
// ============================================================
void cc1101StartSubGHzJammer() {
    if (!cc1101Initialized) return;
    if (!cc1101Wake()) return;

    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.setMHZ(433.92);
    ELECHOUSE_cc1101.SpiWriteReg(0x02, 0x2E);
    ELECHOUSE_cc1101.SetTx();
    delay(2);

    pinMode(CC1101_GDO0, OUTPUT);
    digitalWrite(CC1101_GDO0, HIGH);
    cc1101JammerActive = true;
}

void cc1101StopSubGHzJammer() {
    if (!cc1101Initialized) return;
    digitalWrite(CC1101_GDO0, LOW);
    pinMode(CC1101_GDO0, INPUT_PULLUP);
    ELECHOUSE_cc1101.setSidle();
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

    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.setMHZ(433.92);
    pinMode(CC1101_GDO0, INPUT);
    ELECHOUSE_cc1101.SetRx();
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
            attachInterrupt(digitalPinToInterrupt(CC1101_GDO0), cc1101ISR, CHANGE);
            rj_state = 1;
            rj_timer = now;
        }
    }
    else if (rj_state == 1) {
        if (now - rj_timer > 200) {
            isr_enabled = false;
            detachInterrupt(digitalPinToInterrupt(CC1101_GDO0));
            ELECHOUSE_cc1101.setSidle();
            ELECHOUSE_cc1101.SetTx();
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
            ELECHOUSE_cc1101.setSidle();
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
    detachInterrupt(digitalPinToInterrupt(CC1101_GDO0));
    digitalWrite(CC1101_GDO0, LOW);
    pinMode(CC1101_GDO0, INPUT);
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
    ELECHOUSE_cc1101.setMHZ(spec_an_freqs[0] / 1000000.0);
    ELECHOUSE_cc1101.SetRx();
}

void cc1101AnalyzerLoop() {
    if (!spec_an_running) return;
    for(int i=0; i<64; i++) {
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
    if (target_h > spec_an_values[spec_an_idx]) spec_an_values[spec_an_idx] = target_h;
    spec_an_idx++;
    if (spec_an_idx >= 64) spec_an_idx = 0;
}

void cc1101StopAnalyzer() {
    spec_an_running = false;
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
// Transmissao RAW (para APK Keeloq)
// ============================================================
void cc1101TransmitRaw(uint32_t frequency, uint16_t* timings, uint8_t length) {
    if (!cc1101Initialized || length == 0 || length > 200) return;
    if (!cc1101Wake()) return;

    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.setMHZ(frequency / 1000000.0);
    ELECHOUSE_cc1101.SpiWriteReg(0x02, 0x2E);
    ELECHOUSE_cc1101.SetTx();
    delay(2);

    pinMode(CC1101_GDO0, OUTPUT);
    digitalWrite(CC1101_GDO0, LOW);
    for (int i = 0; i < length; i++) {
        digitalWrite(CC1101_GDO0, i % 2 == 0 ? HIGH : LOW);
        delayMicroseconds(timings[i]);
    }
    digitalWrite(CC1101_GDO0, LOW);
    pinMode(CC1101_GDO0, INPUT_PULLUP);
    ELECHOUSE_cc1101.setSidle();
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
        ELECHOUSE_cc1101.setSidle();
        if(freq==0) ELECHOUSE_cc1101.setMHZ(868.0);
        else        ELECHOUSE_cc1101.setMHZ(915.0);
        ELECHOUSE_cc1101.SetRx();
        delayMicroseconds(500);
        for(int i=0; i<3; i++) {
            int rssi = ELECHOUSE_cc1101.getRssi();
            if (rssi > maxRssiDbm) maxRssiDbm = rssi;
            if (rssi > -75) persistentHits++;
            delay(5);
        }
    }
    ELECHOUSE_cc1101.setSidle();
    cc1101Sleep();
    if (persistentHits < 2) return 0;
    if (maxRssiDbm < -70) return 0;
    if (maxRssiDbm > -30) return 100;
    return map(maxRssiDbm, -70, -30, 1, 100);
}
