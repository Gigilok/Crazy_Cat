#include <SPI.h>
#include "config.h"

SPIClass spiCC1101(HSPI);

#define CC1101_IOCFG2   0x00
#define CC1101_IOCFG0   0x02
#define CC1101_FIFOTHR  0x03
#define CC1101_PKTCTRL0 0x08
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
#define CC1101_FREND0   0x22
#define CC1101_FSCAL3   0x23
#define CC1101_FSCAL2   0x24
#define CC1101_FSCAL1   0x25
#define CC1101_FSCAL0   0x26
#define CC1101_TEST2    0x2C
#define CC1101_TEST1    0x2D
#define CC1101_TEST0    0x2E
#define CC1101_PARTNUM  0x30
#define CC1101_VERSION  0x31
#define CC1101_MARCSTATE 0x35
#define CC1101_RSSI     0x34

#define CC1101_SRES     0x30
#define CC1101_SCAL     0x33
#define CC1101_SRX      0x34
#define CC1101_STX      0x35
#define CC1101_SIDLE    0x36
#define CC1101_PATABLE  0x3E

#define CC1101_READ_SINGLE  0x80
#define CC1101_READ_BURST   0xC0
#define CC1101_WRITE_BURST  0x40

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

void IRAM_ATTR cc1101ISR() {
    if (!isr_enabled) return; 
    unsigned long now = micros();
    uint8_t val = digitalRead(CC1101_GDO0);
    if (val != isr_last_val) {
        if (isr_count < 200) {
            isr_timings[isr_count] = now - isr_last_change;
            isr_count++;
        }
        isr_last_val = val;
        isr_last_change = now;
        capture_started = true;
    }
}

void cc1101Select() { digitalWrite(CC1101_CSN, LOW); delayMicroseconds(10); }
void cc1101Deselect() { digitalWrite(CC1101_CSN, HIGH); }

uint8_t cc1101ReadReg(uint8_t reg) {
    spiCC1101.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    cc1101Select(); spiCC1101.transfer(reg | CC1101_READ_SINGLE);
    uint8_t val = spiCC1101.transfer(0x00); cc1101Deselect(); spiCC1101.endTransaction();
    return val;
}
uint8_t cc1101ReadStatus(uint8_t reg) {
    spiCC1101.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    cc1101Select(); spiCC1101.transfer(reg | CC1101_READ_BURST);
    uint8_t val = spiCC1101.transfer(0x00); cc1101Deselect(); spiCC1101.endTransaction();
    return val;
}
void cc1101WriteReg(uint8_t reg, uint8_t value) {
    spiCC1101.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    cc1101Select(); spiCC1101.transfer(reg); spiCC1101.transfer(value);
    cc1101Deselect(); spiCC1101.endTransaction();
}
void cc1101SendCommand(uint8_t cmd) {
    spiCC1101.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    cc1101Select(); spiCC1101.transfer(cmd); cc1101Deselect(); spiCC1101.endTransaction();
}
void cc1101SetFrequency(uint32_t freqHz) {
    uint32_t freqWord = (uint32_t)((freqHz / 26000000.0) * 65536);
    cc1101WriteReg(CC1101_FREQ2, (freqWord >> 16) & 0xFF);
    cc1101WriteReg(CC1101_FREQ1, (freqWord >> 8) & 0xFF);
    cc1101WriteReg(CC1101_FREQ0, freqWord & 0xFF);
}

bool cc1101Init() {
    Serial.println("[CC1101] Inicializando...");
    spiCC1101.begin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CSN);
    pinMode(CC1101_CSN, OUTPUT); digitalWrite(CC1101_CSN, HIGH);
    pinMode(CC1101_GDO0, INPUT); pinMode(CC1101_GDO2, INPUT);

    digitalWrite(CC1101_CSN, LOW); delayMicroseconds(10);
    digitalWrite(CC1101_CSN, HIGH); delay(100);

    uint8_t partnum = 0xFF;
    for (int i = 0; i < 3; i++) {
        cc1101SendCommand(CC1101_SRES); delay(10);
        partnum = cc1101ReadStatus(CC1101_PARTNUM);
        // CORREÇÃO CRÍTICA: PARTNUM=0x00 é o valor CORRETO do CC1101!
        // O datasheet diz que CC1101 retorna 0x00 no registrador PARTNUM.
        // Antes o código rejeitava 0x00 achando que era falha, mas é sucesso.
        // Só 0xFF significa que o módulo não responde.
        if (partnum != 0xFF) break;
        delay(50);
    }
    if (partnum == 0xFF) return false;  // só rejeita 0xFF (sem resposta)

    attachInterrupt(digitalPinToInterrupt(CC1101_GDO0), cc1101ISR, CHANGE);

    cc1101WriteReg(CC1101_IOCFG0, 0x0D); 
    cc1101WriteReg(CC1101_FIFOTHR, 0x47);
    cc1101WriteReg(CC1101_PKTCTRL0, 0x32); 
    cc1101WriteReg(CC1101_MDMCFG4, 0x17); 
    cc1101WriteReg(CC1101_MDMCFG3, 0x83); 
    cc1101WriteReg(CC1101_MDMCFG2, 0x30); 
    cc1101WriteReg(CC1101_MDMCFG1, 0x22);
    cc1101WriteReg(CC1101_MDMCFG0, 0xF8);
    cc1101WriteReg(CC1101_DEVIATN, 0x15);
    cc1101WriteReg(CC1101_MCSM0, 0x18);
    cc1101WriteReg(CC1101_FOCCFG, 0x16);
    cc1101WriteReg(CC1101_BSCFG, 0x6C);
    cc1101WriteReg(CC1101_AGCCTRL2, 0x43);
    cc1101WriteReg(CC1101_AGCCTRL1, 0x40);
    cc1101WriteReg(CC1101_AGCCTRL0, 0x91);
    cc1101WriteReg(CC1101_FREND0, 0x11);
    cc1101WriteReg(CC1101_FSCAL3, 0xE9);
    cc1101WriteReg(CC1101_FSCAL2, 0x2A);
    cc1101WriteReg(CC1101_FSCAL1, 0x00);
    cc1101WriteReg(CC1101_FSCAL0, 0x1F);
    cc1101WriteReg(CC1101_TEST2, 0x81);
    cc1101WriteReg(CC1101_TEST1, 0x35);
    cc1101WriteReg(CC1101_TEST0, 0x09);

    spiCC1101.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    cc1101Select(); spiCC1101.transfer(CC1101_PATABLE | CC1101_WRITE_BURST);
    for (int i = 0; i < 8; i++) spiCC1101.transfer(0xC0); 
    cc1101Deselect(); spiCC1101.endTransaction();

    cc1101Initialized = true;
    Serial.println("[CC1101] Configurado com sucesso!");
    return true;
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
    capture_state = STATE_HOPPING;
    isr_count = 0;
    capture_started = false;
    // CORREÇÃO: IOCFG0=0x0D mantém GDO0 como saída de dados assíncronos.
    // A ISR é ativada desde o início para contar transições (mesmo no HOPPING).
    // Se isr_count > 5 em menos de 100ms, há sinal real (não é só ruído).
    cc1101WriteReg(CC1101_IOCFG0, 0x0D);
    pinMode(CC1101_GDO0, INPUT); 
    cc1101SetFrequency(currentCapture.frequency);
    cc1101SendCommand(CC1101_SIDLE); delay(1);
    cc1101SendCommand(CC1101_SRX); delay(10);
    // Habilita ISR desde o início — ela só conta transições, não causa bootloop
    isr_last_val = digitalRead(CC1101_GDO0);
    isr_last_change = micros();
    isr_enabled = true;
}

void cc1101CaptureLoop() {
    if (!cc1101CopyActive) return;
    unsigned long now = micros();
    unsigned long nowMs = millis();

    if (capture_state == STATE_HOPPING) {
        // CORREÇÃO: usa contagem de transições da ISR em vez de digitalRead(GDO0).
        // digitalRead(GDO0) só pega o nível atual — mas GDO0 com OOK oscila rapidamente.
        // Se a ISR capturou mais de 5 transições, há sinal real sendo recebido.
        // O ruído de fundo gera ~1-2 transições por segundo; um controle gera centenas.
        if (isr_count > 5) {
            capture_state = STATE_LOCKED;
        } 
        else if (nowMs - lastFreqSwitch > 1000) {
            // Sem sinal, troca de frequência e zera contador
            isr_enabled = false;
            isr_count = 0;
            capture_started = false;
            currentFreqIndex = (currentFreqIndex + 1) % 4;
            currentCapture.frequency = captureFreqs[currentFreqIndex];
            cc1101SetFrequency(currentCapture.frequency);
            cc1101SendCommand(CC1101_SIDLE); delay(1);
            cc1101SendCommand(CC1101_SRX); delay(5);
            isr_last_val = digitalRead(CC1101_GDO0);
            isr_last_change = micros();
            isr_enabled = true;
            lastFreqSwitch = nowMs;
        }
    } 
    else if (capture_state == STATE_LOCKED) {
        // IOCFG0 já está em 0x0D desde StartCapture (não precisa reescrever)
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
            // Mantém 0x0D (não volta para 0x06 que não detecta OOK)
            isr_count = 0;
            capture_started = false;
            capture_state = STATE_HOPPING;
            isr_last_change = micros();
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
                if (sig->length < 25 && totalDuration < 30000) snprintf(sig->name, 16, "Sensor %luM", sig->frequency / 1000000);
                else if (sig->length >= 24 && sig->length <= 50) snprintf(sig->name, 16, "Portao %luM", sig->frequency / 1000000);
                else if (sig->length > 50 || totalDuration > 70000) snprintf(sig->name, 16, "Carro %luM", sig->frequency / 1000000);
                else snprintf(sig->name, 16, "Sinal %luM", sig->frequency / 1000000);
                savedSignalCount++;
            }
        } 
        else if (totalTimeout) {
            isr_enabled = false;
            // Mantém 0x0D
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
            cc1101SendCommand(CC1101_SIDLE); delay(1);
            cc1101SendCommand(CC1101_SRX); delay(5);
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

void cc1101ReplaySignal(uint8_t index) {
    if (index >= savedSignalCount || !savedSignals[index].valid) return;
    if (!cc1101Initialized) return;
    isr_enabled = false; 
    SignalData* sig = &savedSignals[index];
    cc1101SetFrequency(sig->frequency); 
    cc1101WriteReg(CC1101_IOCFG0, 0x2E); 
    cc1101SendCommand(CC1101_SIDLE); delay(1);
    cc1101SendCommand(CC1101_STX); delay(1); 
    pinMode(CC1101_GDO0, OUTPUT);
    for (int i = 0; i < sig->length; i++) {
        digitalWrite(CC1101_GDO0, i % 2 == 0 ? HIGH : LOW);
        delayMicroseconds(sig->timings[i]);
    }
    digitalWrite(CC1101_GDO0, LOW);
    pinMode(CC1101_GDO0, INPUT);
    cc1101WriteReg(CC1101_IOCFG0, 0x0D); 
    cc1101SendCommand(CC1101_SIDLE);
}

void cc1101SendBruteForceCode(uint32_t code, uint32_t freq) {
    if (!cc1101Initialized) return;
    isr_enabled = false; 
    cc1101SetFrequency(freq);
    cc1101WriteReg(CC1101_IOCFG0, 0x2E); 
    cc1101SendCommand(CC1101_SIDLE); delay(1);
    cc1101SendCommand(CC1101_STX); delay(1);
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
    pinMode(CC1101_GDO0, INPUT);
    cc1101WriteReg(CC1101_IOCFG0, 0x0D);
    cc1101SendCommand(CC1101_SIDLE);
}

void cc1101StartSubGHzJammer() {
    if (!cc1101Initialized) return;
    isr_enabled = false;
    cc1101SetFrequency(433920000);
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
}

// ============================================================
// ROLLJAM AUTO (TÉCNICA CATCH AND JAM - ESTILO FLIPPER ZERO)
// ============================================================

void cc1101StartRollJam() {
    if (!cc1101Initialized) return;
    cc1101RollJamActive = true;
    rj_state = 0; 
    rj_timer = millis();
    currentCapture.frequency = 433920000; 
    
    isr_enabled = false;
    cc1101SetFrequency(currentCapture.frequency);
    
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
        }
    }
}

void cc1101StopRollJam() {
    cc1101RollJamActive = false;
    isr_enabled = false;
    digitalWrite(CC1101_GDO0, LOW);
    pinMode(CC1101_GDO0, INPUT);
    cc1101WriteReg(CC1101_IOCFG0, 0x0D);
    cc1101SendCommand(CC1101_SIDLE);
}

// ============================================================
// ANALISADOR DE ESPECTRO SUB-GHz
// ============================================================
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
    cc1101SetFrequency(freq);
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

// NOVA FUNÇÃO: Transmitir sinais gerados pelo Termux (Keeloq)
void cc1101TransmitRaw(uint32_t frequency, uint16_t* timings, uint8_t length) {
    if (!cc1101Initialized || length == 0 || length > 200) return;
    isr_enabled = false; 
    cc1101SetFrequency(frequency);
    cc1101WriteReg(CC1101_IOCFG0, 0x2E); 
    cc1101SendCommand(CC1101_SIDLE); delay(1);
    cc1101SendCommand(CC1101_STX); delay(1); 
    pinMode(CC1101_GDO0, OUTPUT);
    for (int i = 0; i < length; i++) {
        digitalWrite(CC1101_GDO0, i % 2 == 0 ? HIGH : LOW);
        delayMicroseconds(timings[i]);
    }
    digitalWrite(CC1101_GDO0, LOW);
    pinMode(CC1101_GDO0, INPUT);
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
        if(freq==0) cc1101SetFrequency(868000000);
        else cc1101SetFrequency(915000000);
        cc1101SendCommand(CC1101_SRX);
        delayMicroseconds(500); 
        for(int i=0; i<3; i++) {
            uint8_t rssiDec = cc1101ReadStatus(CC1101_RSSI);
            int rssi = (rssiDec >= 128) ? ((int)rssiDec - 256) / 2 - 74 : (int)rssiDec / 2 - 74;
            if (rssi > maxRssiDbm) maxRssiDbm = rssi;
            if (rssi > -70) persistentHits++; 
            delay(5); 
        }
    }
    cc1101SendCommand(CC1101_SIDLE);
    if (persistentHits < 3) return 0;
    if (maxRssiDbm < -65) return 0;
    if (maxRssiDbm > -30) return 100;
    return map(maxRssiDbm, -65, -30, 1, 100);
}
