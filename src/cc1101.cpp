#include <SPI.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include "config.h"

// ============================================================
// CC1101 - Usa biblioteca ELECHOUSE (mesma do ESP32-DIV)
// Elimina todos os problemas de SPI custom
// ============================================================

bool cc1101Initialized = false;
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

// === ISR com filtro de ruído (estilo rc-switch) ===
void IRAM_ATTR cc1101ISR() {
    if (!isr_enabled) return; 
    unsigned long now = micros();
    uint8_t val = digitalRead(CC1101_GDO0);
    if (val != isr_last_val) {
        unsigned long duration = now - isr_last_change;
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

// === Funções wrapper para compatibilidade ===
void cc1101Select() {}
void cc1101Deselect() {}

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
}
void cc1101SetFrequency(uint32_t freqHz) {
    ELECHOUSE_cc1101.setMHZ(freqHz / 1000000.0);
}

// === INIT usando ELECHOUSE (igual ESP32-DIV) ===
bool cc1101Init() {
    Serial.println("[CC1101] Inicializando com ELECHOUSE library...");
    
    // Configura pinos SPI (igual ESP32-DIV)
    ELECHOUSE_cc1101.setSpiPin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CSN);
    ELECHOUSE_cc1101.setGDO(CC1101_GDO0, CC1101_GDO2);
    
    // Init (faz Reset + RegConfigSettings automaticamente)
    ELECHOUSE_cc1101.Init();
    
    // Verifica se o módulo responde
    if (!ELECHOUSE_cc1101.getCC1101()) {
        Serial.println("[CC1101] ERRO: Modulo nao encontrado!");
        return false;
    }
    
    // Configura OOK (igual ESP32-DIV: setModulation(2))
    ELECHOUSE_cc1101.setModulation(2);     // 2 = ASK/OOK
    ELECHOUSE_cc1101.setRxBW(500.0);       // 500kHz bandwidth
    ELECHOUSE_cc1101.setSyncMode(0);       // No preamble/sync
    ELECHOUSE_cc1101.setMHZ(433.92);       // 433.92 MHz
    
    // CORREÇÃO CRÍTICA: Mudar IOCFG0 para 0x0D (async serial data).
    // O Init() da ELECHOUSE configura IOCFG0=0x06 (FIFO) por padrão.
    // Mas 0x06 NÃO oscila o GDO0 com dados OOK — só sinaliza FIFO cheio.
    // Para captura RAW de timings, precisamos do dado demodulado direto no GDO0.
    // 0x0D = GDO0 output = async serial data (copia o sinal OOK demodulado).
    ELECHOUSE_cc1101.SpiWriteReg(0x02, 0x0D);  // IOCFG0 = 0x0D
    
    // Entra em RX
    ELECHOUSE_cc1101.SetRx();
    
    // CORREÇÃO: INPUT_PULLUP no GDO0 para evitar flutuação.
    // GPIO16 não é strapping pin, INPUT_PULLUP é seguro.
    // Sem pull, o pino flutua e capta ruído do ambiente (dedo faz ler).
    pinMode(CC1101_GDO0, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(CC1101_GDO0), cc1101ISR, CHANGE);
    
    cc1101Initialized = true;
    Serial.println("[CC1101] Configurado com sucesso! (ELECHOUSE)");
    Serial.printf("[CC1101] PARTNUM=0x%02X VERSION=0x%02X\n", 
                  ELECHOUSE_cc1101.SpiReadStatus(0x30),
                  ELECHOUSE_cc1101.SpiReadStatus(0x31));
    return true;
}

// === CAPTURA ===
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
    isr_count = 0;
    capture_started = false;
    
    // Configura frequência e entra em RX (estilo ELECHOUSE)
    ELECHOUSE_cc1101.setMHZ(currentCapture.frequency / 1000000.0);
    // Garante IOCFG0=0x0D (async serial) para captura RAW
    ELECHOUSE_cc1101.SpiWriteReg(0x02, 0x0D);
    ELECHOUSE_cc1101.SetRx();
    delay(10);
    
    // Habilita ISR
    isr_last_val = digitalRead(CC1101_GDO0);
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
        } 
        else if (nowMs - lastFreqSwitch > 1000) {
            isr_enabled = false;
            isr_count = 0;
            capture_started = false;
            currentFreqIndex = (currentFreqIndex + 1) % 4;
            currentCapture.frequency = captureFreqs[currentFreqIndex];
            ELECHOUSE_cc1101.setMHZ(currentCapture.frequency / 1000000.0);
            ELECHOUSE_cc1101.SpiWriteReg(0x02, 0x0D); // IOCFG0 = async serial
            ELECHOUSE_cc1101.SetRx();
            delay(5);
            isr_last_val = digitalRead(CC1101_GDO0);
            isr_last_change = micros();
            isr_enabled = true;
            lastFreqSwitch = nowMs;
        }
    } 
    else if (capture_state == STATE_LOCKED) {
        delay(2); 
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
            lastFreqSwitch = nowMs;
        }
        else if ((capture_started && silenceTimeout) || bufferFull) {
            isr_enabled = false; 
            currentCapture.active = false;
            cc1101CopyActive = false;
            ELECHOUSE_cc1101.SpiStrobe(0x36); // SIDLE

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
                if (sig->length < 25 && totalDuration < 30000) snprintf(sig->name, 16, "Sensor %luM", sig->frequency / 1000000);
                else if (sig->length >= 24 && sig->length <= 50) snprintf(sig->name, 16, "Portao %luM", sig->frequency / 1000000);
                else if (sig->length > 50 || totalDuration > 70000) snprintf(sig->name, 16, "Carro %luM", sig->frequency / 1000000);
                else snprintf(sig->name, 16, "Sinal %luM", sig->frequency / 1000000);
                savedSignalCount++;
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
            ELECHOUSE_cc1101.setMHZ(currentCapture.frequency / 1000000.0);
            ELECHOUSE_cc1101.SpiWriteReg(0x02, 0x0D); // IOCFG0 = async serial
            ELECHOUSE_cc1101.SetRx();
            delay(5);
        }
    }
}

void cc1101StopCapture() {
    isr_enabled = false; 
    cc1101CopyActive = false;
    currentCapture.active = false;
    ELECHOUSE_cc1101.SpiStrobe(0x36); // SIDLE
}

uint8_t cc1101GetPulseCount() { return isr_count; }
uint32_t cc1101GetCurrentFreq() { return currentCapture.frequency / 1000000; }
uint8_t cc1101GetPinState() { return digitalRead(CC1101_GDO0); } 

// === REPLAY ===
void cc1101ReplaySignal(uint8_t index) {
    if (index >= savedSignalCount || !savedSignals[index].valid) return;
    if (!cc1101Initialized) return;
    isr_enabled = false; 
    SignalData* sig = &savedSignals[index];
    ELECHOUSE_cc1101.setMHZ(sig->frequency / 1000000.0);
    ELECHOUSE_cc1101.SetTx();
    delay(2);
    pinMode(CC1101_GDO0, OUTPUT);
    for (int i = 0; i < sig->length; i++) {
        digitalWrite(CC1101_GDO0, i % 2 == 0 ? HIGH : LOW);
        delayMicroseconds(sig->timings[i]);
    }
    digitalWrite(CC1101_GDO0, LOW);
    pinMode(CC1101_GDO0, INPUT_PULLUP);
    ELECHOUSE_cc1101.SetRx();
}

// === BRUTE FORCE ===
void cc1101SendBruteForceCode(uint32_t code, uint32_t freq) {
    if (!cc1101Initialized) return;
    isr_enabled = false; 
    ELECHOUSE_cc1101.setMHZ(freq / 1000000.0);
    ELECHOUSE_cc1101.SetTx();
    delay(2);
    pinMode(CC1101_GDO0, OUTPUT);
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
    ELECHOUSE_cc1101.SetRx();
}

// === JAMMER ===
void cc1101StartSubGHzJammer() {
    if (!cc1101Initialized) return;
    isr_enabled = false;
    ELECHOUSE_cc1101.setMHZ(433.92);
    ELECHOUSE_cc1101.SetTx();
    delay(2);
    pinMode(CC1101_GDO0, OUTPUT);
    digitalWrite(CC1101_GDO0, HIGH); 
}

void cc1101StopSubGHzJammer() {
    if (!cc1101Initialized) return;
    digitalWrite(CC1101_GDO0, LOW);
    pinMode(CC1101_GDO0, INPUT_PULLUP);
    ELECHOUSE_cc1101.SetRx();
}

// === ROLLJAM ===
void cc1101StartRollJam() {
    if (!cc1101Initialized) return;
    cc1101RollJamActive = true;
    rj_state = 0; 
    rj_timer = millis();
    currentCapture.frequency = 433920000; 
    isr_enabled = false;
    ELECHOUSE_cc1101.setMHZ(433.92);
    pinMode(CC1101_GDO0, INPUT_PULLUP);
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
            rj_state = 1; 
            rj_timer = now;
        }
    } 
    else if (rj_state == 1) {
        if (now - rj_timer > 200) {
            isr_enabled = false; 
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
            pinMode(CC1101_GDO0, INPUT_PULLUP);
            ELECHOUSE_cc1101.SetRx();
            
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
    ELECHOUSE_cc1101.SetRx();
}

// === ANALYZER ===
void cc1101StartAnalyzer() {
    if (!cc1101Initialized) return;
    spec_an_running = true;
    spec_an_idx = 0;
    for(int i=0; i<15; i++) spec_an_freqs[i] = 300000000 + (i * 3200000);
    for(int i=0; i<16; i++) spec_an_freqs[15+i] = 387000000 + (i * 4800000);
    for(int i=0; i<33; i++) spec_an_freqs[31+i] = 779000000 + (i * 4500000);
    for(int i=0; i<64; i++) spec_an_values[i] = 0; 
}

void cc1101AnalyzerLoop() {
    if (!spec_an_running) return;
    for(int i=0; i<64; i++) {
        if (spec_an_values[i] > 0) spec_an_values[i]--;
    }
    uint32_t freq = spec_an_freqs[spec_an_idx];
    ELECHOUSE_cc1101.setMHZ(freq / 1000000.0);
    ELECHOUSE_cc1101.SetRx();
    delay(2);
    int rssi = ELECHOUSE_cc1101.getRssi();
    if (rssi < -90) rssi = -90;
    if (rssi > -30) rssi = -30;
    uint16_t target_h = map(rssi, -90, -30, 0, 40);
    if (rssi < -80) target_h = 0;
    if (target_h > spec_an_values[spec_an_idx]) {
        spec_an_values[spec_an_idx] = target_h;
    }
    spec_an_idx++;
    if (spec_an_idx >= 64) spec_an_idx = 0; 
}

void cc1101StopAnalyzer() {
    spec_an_running = false;
    ELECHOUSE_cc1101.SpiStrobe(0x36); // SIDLE
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
}

void cc1101DeleteSignal(uint8_t index) {
    if (index >= savedSignalCount) return;
    for (int i = index; i < savedSignalCount - 1; i++) {
        savedSignals[i] = savedSignals[i + 1];
    }
    memset(&savedSignals[savedSignalCount - 1], 0, sizeof(SignalData));
    savedSignalCount--;
}

void cc1101TransmitRaw(uint32_t frequency, uint16_t* timings, uint8_t length) {
    if (!cc1101Initialized || length == 0 || length > 200) return;
    isr_enabled = false; 
    ELECHOUSE_cc1101.setMHZ(frequency / 1000000.0);
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
    ELECHOUSE_cc1101.SetRx();
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
        ELECHOUSE_cc1101.SpiStrobe(0x36); // SIDLE
        if(freq==0) ELECHOUSE_cc1101.setMHZ(868.0);
        else ELECHOUSE_cc1101.setMHZ(915.0);
        ELECHOUSE_cc1101.SetRx();
        delayMicroseconds(500); 
        for(int i=0; i<3; i++) {
            int rssi = ELECHOUSE_cc1101.getRssi();
            if (rssi > maxRssiDbm) maxRssiDbm = rssi;
            if (rssi > -70) persistentHits++; 
            delay(5); 
        }
    }
    ELECHOUSE_cc1101.SpiStrobe(0x36); // SIDLE
    if (persistentHits < 3) return 0;
    if (maxRssiDbm < -65) return 0;
    if (maxRssiDbm > -30) return 100;
    return map(maxRssiDbm, -65, -30, 1, 100);
}
