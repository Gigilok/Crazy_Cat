#include <SPI.h>
#include <RF24.h>
#include "config.h"

RF24 radio(NRF_CE, NRF_CSN);

struct NRFDevice {
    uint8_t address[5];
    uint8_t channel;
    int8_t rssi;
};

NRFDevice nrfDevices[20];
uint8_t nrfDeviceCount = 0;

// Scanner antigo
static int8_t scanHistory[NRF_SCAN_HISTORY];
static int scanIndex = 0;
static bool scanning = false;
static unsigned long scanLastUpdate = 0;
static uint32_t scanTotalPackets = 0;
static int8_t scanBarData[16];

// Analyzer
struct DetectedSignal {
    uint8_t channel;
    uint8_t power; 
    unsigned long lastSeen;
    bool active;
};
static DetectedSignal detectedSignals[NRF_MAX_DETECTED];
static uint8_t detectedCount = 0;
static uint8_t analyzeSelectedIndex = 0;
static bool analyzing = false;

static int analyzeCurrentCh = 0;

// Jammer
static int jamChannel = 0;
static unsigned long jamLastSwitch = 0;
static uint32_t jamTotalPackets = 0;
static uint32_t jamChannelPackets = 0;
static int8_t jamHistory[16];
static int jamHistoryIndex = 0;

// Saved signals
static SignalData nrfSavedSignals[MAX_SAVED_SIGNALS];
static uint8_t nrfSavedCount = 0;

// ============================================================
// SCANNER SPECTRUM (ESTILO FLIPPER ZERO)
// ============================================================
#define SPEC_BARS_FLIPPER 64
#define SPEC_MAX_HEIGHT_FLIPPER 40
#define WATERFALL_ROWS 21

static uint8_t specCurrentHeights[SPEC_BARS_FLIPPER];
static uint8_t specPeakHeights[SPEC_BARS_FLIPPER];
static uint8_t waterfallData[WATERFALL_ROWS][SPEC_BARS_FLIPPER / 8];
static bool specRunning = false;
static uint32_t specFrames = 0;

const uint8_t dummyAddress[5] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE};

// Forward declarations (necessárias para nrf24Sleep)
void nrf24StopJammer();
void nrf24StopScan();
void nrf24StopAnalyze();

static void hardResetNRF24() {
    digitalWrite(NRF_CE, LOW);
    delay(150);
    digitalWrite(NRF_CE, HIGH);
    delay(150);
    digitalWrite(NRF_CSN, HIGH);
    delay(10);
    digitalWrite(NRF_CSN, LOW);
    delay(10);
    digitalWrite(NRF_CSN, HIGH);
    delay(50);
}

bool nrf24Init() {
    Serial.println(F("[NRF24] Inicializando..."));
    Serial.flush();

    // Configura pinos ANTES do SPI.begin para garantir estado deterministico
    pinMode(NRF_CE, OUTPUT);
    pinMode(NRF_CSN, OUTPUT);
    digitalWrite(NRF_CE, LOW);
    digitalWrite(NRF_CSN, HIGH);
    delay(10);

    // *** SPI já foi inicializado no main.cpp com os mesmos pinos ***
    // Não chamamos SPI.begin() novamente!

    // Reset fisico do modulo via pino CE/CSN
    hardResetNRF24();

    // Tenta inicializar o radio. Se o modulo nao estiver conectado,
    // radio.begin() retorna false e a init falha graciosamente.
    Serial.println(F("[NRF24] Chamando radio.begin()..."));
    Serial.flush();
    if (!radio.begin()) {
        Serial.println(F("[NRF24] FAIL: radio.begin() retornou false"));
        Serial.flush();
        return false;
    }
    Serial.println(F("[NRF24] radio.begin() OK"));
    Serial.flush();

    radio.setPALevel(RF24_PA_MAX, true);
    radio.setDataRate(RF24_1MBPS);
    radio.setAutoAck(false);
    radio.disableCRC();
    radio.setRetries(0, 0);
    radio.setCRCLength(RF24_CRC_DISABLED);
    radio.openReadingPipe(1, dummyAddress);
    Serial.println(F("[NRF24] Configurado com sucesso!"));
    Serial.flush();
    return true;
}

// ============================================================
// SLEEP – chamada por menu.cpp ao sair dos menus NRF24
// ============================================================
void nrf24Sleep() {
    if (nrf24JammerActive) { nrf24StopJammer(); }
    if (scanning) { nrf24StopScan(); }
    if (analyzing) { nrf24StopAnalyze(); }
    radio.powerDown();
    Serial.println(F("[NRF24] Modulo em POWER DOWN"));
    Serial.flush();
}

// SCANNER ANTIGO
void nrf24StartScan() {
    scanning = true;
    scanIndex = 0;
    scanTotalPackets = 0;
    for (int i = 0; i < NRF_SCAN_HISTORY; i++) scanHistory[i] = -100;
    for (int i = 0; i < 16; i++) scanBarData[i] = -100;
    radio.setAutoAck(false);
    radio.setCRCLength(RF24_CRC_DISABLED);
    radio.setPALevel(RF24_PA_MAX, true);
    radio.setDataRate(RF24_1MBPS);
    radio.openReadingPipe(1, dummyAddress);
}
void nrf24StopScan() { scanning = false; radio.stopListening(); }
bool nrf24IsScanning() { return scanning; }
const int8_t* nrf24GetScanHistory() { return scanHistory; }
int nrf24GetScanIndex() { return scanIndex; }
uint32_t nrf24GetScanTotalPackets() { return scanTotalPackets; }
const int8_t* nrf24GetScanBarData() { return scanBarData; }

void nrf24ScanLoop() {
    if (!scanning) return;
    unsigned long now = millis();
    if (now - scanLastUpdate < 15) return;
    scanLastUpdate = now;
    int ch = scanIndex % 125;
    radio.setChannel(ch);
    radio.startListening();
    delayMicroseconds(250);
    bool rpd = radio.testRPD();
    int8_t rssi = -100;
    if (rpd) { rssi = -64 - random(20); scanTotalPackets++; }
    else if (radio.testCarrier()) { rssi = -75 - random(10); }
    radio.stopListening();
    scanHistory[scanIndex] = rssi;
    int barIdx = (ch / 8) % 16;
    if (rssi > scanBarData[barIdx]) scanBarData[barIdx] = rssi;
    if (scanIndex % 125 == 0) {
        for (int i = 0; i < 16; i++) {
            if (scanBarData[i] > -100) scanBarData[i] -= 5;
            if (scanBarData[i] < -100) scanBarData[i] = -100;
        }
    }
    scanIndex++;
    if (scanIndex >= NRF_SCAN_HISTORY) scanIndex = 0;
}

// ============================================================
// ANALYZER (SNIFFER NÃO-BLOQUEANTE CORRIGIDO)
// ============================================================
void nrf24StartAnalyze() {
    analyzing = true;
    detectedCount = 0;
    analyzeSelectedIndex = 0;
    analyzeCurrentCh = 0;
    
    for (int i = 0; i < NRF_MAX_DETECTED; i++) detectedSignals[i].active = false;
    
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_1MBPS);
    radio.setAutoAck(false);
    radio.setCRCLength(RF24_CRC_DISABLED);
    radio.openReadingPipe(1, dummyAddress);
    radio.startListening();
}

void nrf24AnalyzeTick() {
    if (!analyzing) return;

    radio.stopListening(); 
    radio.setChannel(analyzeCurrentCh);
    radio.startListening(); 
    delayMicroseconds(200); 
    
    bool rpd = radio.testRPD();
    bool cd = radio.testCarrier();
    
    radio.stopListening(); 
    radio.flush_rx();

    if (rpd || cd) {
        uint8_t power = rpd ? 100 : 50; 
        bool found = false;
        
        for (int i = 0; i < detectedCount; i++) {
            if (detectedSignals[i].channel == analyzeCurrentCh) {
                detectedSignals[i].power = max(detectedSignals[i].power, power);
                detectedSignals[i].lastSeen = millis();
                detectedSignals[i].active = true;
                found = true;
                break;
            }
        }
        
        if (!found && detectedCount < NRF_MAX_DETECTED) {
            detectedSignals[detectedCount].channel = analyzeCurrentCh;
            detectedSignals[detectedCount].power = power;
            detectedSignals[detectedCount].lastSeen = millis();
            detectedSignals[detectedCount].active = true;
            detectedCount++;
        }
    }

    analyzeCurrentCh++;
    if (analyzeCurrentCh >= 125) analyzeCurrentCh = 0;
}

void nrf24StopAnalyze() {
    analyzing = false;
    radio.stopListening();
}

bool nrf24IsAnalyzing() { return analyzing; }
uint8_t nrf24GetDetectedCount() { return detectedCount; }
DetectedSignal* nrf24GetDetected(uint8_t index) { if (index < detectedCount) return &detectedSignals[index]; return nullptr; }
uint8_t nrf24GetAnalyzeSelected() { return analyzeSelectedIndex; }
void nrf24SetAnalyzeSelected(uint8_t idx) { if (idx < detectedCount) analyzeSelectedIndex = idx; }

bool nrf24SaveSignal(uint8_t detectedIdx) {
    if (detectedIdx >= detectedCount) return false;
    if (nrfSavedCount >= MAX_SAVED_SIGNALS) return false;
    DetectedSignal* sig = &detectedSignals[detectedIdx];
    nrfSavedSignals[nrfSavedCount].length = 1;
    nrfSavedSignals[nrfSavedCount].frequency = 2400000000UL + (sig->channel * 1000000UL);
    nrfSavedSignals[nrfSavedCount].modulation = 0;
    nrfSavedSignals[nrfSavedCount].valid = true;
    nrfSavedSignals[nrfSavedCount].timings[0] = sig->channel;
    snprintf(nrfSavedSignals[nrfSavedCount].name, 16, "NRF CH%d", sig->channel);
    nrfSavedCount++;
    return true;
}
uint8_t nrf24GetSavedCount() { return nrfSavedCount; }
SignalData* nrf24GetSavedSignal(uint8_t index) { if (index < nrfSavedCount) return &nrfSavedSignals[index]; return nullptr; }

// ============================================================
// SPECTRUM (FLIPPER ZERO STYLE)
// ============================================================
void nrf24SpecInit() {
    specFrames = 0;
    memset(specCurrentHeights, 0, sizeof(specCurrentHeights));
    memset(specPeakHeights, 0, sizeof(specPeakHeights));
    memset(waterfallData, 0, sizeof(waterfallData));
}

void nrf24SpecScan() {
    if (!specRunning) return;
    
    int signalsFoundThisScan = 0;

    for (int ch = 0; ch < 125; ch++) {
        radio.stopListening();
        radio.setChannel(ch);
        radio.startListening();
        delayMicroseconds(200);
        bool cd = radio.testCarrier();
        bool rpd = radio.testRPD();
        radio.stopListening();
        radio.flush_rx();

        int barIdx = map(ch, 0, 124, 0, SPEC_BARS_FLIPPER - 1);
        int targetHeight = 0;
        
        if (rpd) targetHeight = SPEC_MAX_HEIGHT_FLIPPER;       
        else if (cd) targetHeight = SPEC_MAX_HEIGHT_FLIPPER / 2; 

        if (targetHeight > 0) signalsFoundThisScan++;

        if (targetHeight > specCurrentHeights[barIdx]) {
            specCurrentHeights[barIdx] = targetHeight;
        } else {
            if (specCurrentHeights[barIdx] > 0) specCurrentHeights[barIdx]--;
        }

        if (specCurrentHeights[barIdx] > specPeakHeights[barIdx]) {
            specPeakHeights[barIdx] = specCurrentHeights[barIdx];
        } else {
            if (specPeakHeights[barIdx] > 0) specPeakHeights[barIdx]--;
        }
    }

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 1000) {
        lastPrint = millis();
        Serial.printf("[NRF24 SCANNER] Varredura completa. Sinais detectados: %d/125\n", signalsFoundThisScan);
    }

    for (int y = WATERFALL_ROWS - 1; y > 0; y--) {
        for (int x = 0; x < (SPEC_BARS_FLIPPER / 8); x++) {
            waterfallData[y][x] = waterfallData[y-1][x];
        }
    }
    memset(waterfallData[0], 0, SPEC_BARS_FLIPPER / 8);
    for (int i = 0; i < SPEC_BARS_FLIPPER; i++) {
        if (specCurrentHeights[i] > 0) {
            waterfallData[0][i / 8] |= (1 << (i % 8));
        }
    }
    specFrames++;
}

void nrf24SpecStart() {
    nrf24SpecInit();
    specRunning = true;
    radio.setAutoAck(false);
    radio.setCRCLength(RF24_CRC_DISABLED);
    radio.setRetries(0, 0);
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_1MBPS);
    radio.openReadingPipe(1, dummyAddress);
}

void nrf24SpecStop() { specRunning = false; radio.stopListening(); }
bool nrf24SpecIsRunning() { return specRunning; }
uint32_t nrf24SpecGetFrames() { return specFrames; }

int8_t nrf24SpecGetBarValue(int displayIdx) {
    if (displayIdx < 0 || displayIdx >= SPEC_BARS_FLIPPER) return 0;
    return specCurrentHeights[displayIdx];
}
int8_t nrf24SpecGetPeakValue(int displayIdx) {
    if (displayIdx < 0 || displayIdx >= SPEC_BARS_FLIPPER) return 0;
    return specPeakHeights[displayIdx];
}
bool nrf24SpecGetWaterfallPixel(int y, int x) {
    if (y < 0 || y >= WATERFALL_ROWS || x < 0 || x >= SPEC_BARS_FLIPPER) return false;
    return (waterfallData[y][x / 8] & (1 << (x % 8))) != 0;
}

int8_t nrf24SpecGetSelectedBar() { return 32; }
void nrf24SpecSetSelectedBar(int8_t bar) { (void)bar; }
int8_t nrf24SpecGetAnalysisChannel() { return 64; }
void nrf24SpecSetAnalysisChannel(int8_t ch) { (void)ch; }

// JAMMER
#define JAM_SWITCH_INTERVAL_US 200
void nrf24StartJammer() {
    if (nrf24JammerActive) return;
    if (!radio.isChipConnected()) {
        Serial.println(F("[NRF24] JAMMER: modulo nao conectado!"));
        return;
    }
    nrf24JammerActive = true;
    jamTotalPackets = 0;
    jamChannelPackets = 0;
    jamChannel = 0;
    jamLastSwitch = 0;
    for (int i = 0; i < 16; i++) jamHistory[i] = 0;
    radio.stopListening();
    radio.setAutoAck(false);
    radio.setRetries(0, 0);
    radio.setPALevel(RF24_PA_MAX, true);
    radio.setDataRate(RF24_1MBPS);
    radio.setCRCLength(RF24_CRC_DISABLED);
    radio.setChannel(0);
    radio.startConstCarrier(RF24_PA_MAX, 0);
    Serial.println(F("[NRF24] JAMMER: iniciado (1MBPS, channel hopping 0-125)"));
}
void nrf24StopJammer() {
    if (!nrf24JammerActive) return;
    radio.stopConstCarrier();
    radio.stopListening();
    radio.flush_tx();
    nrf24JammerActive = false;
}
int nrf24JammerLoop() {
    if (!nrf24JammerActive) return -1;
    jamTotalPackets++;
    jamChannelPackets++;
    int activeBar = (jamChannel / 8) % 16;
    for (int i = 0; i < 16; i++) {
        if (i == activeBar) jamHistory[i] = min(jamHistory[i] + 20, 50);
        else jamHistory[i] = max(jamHistory[i] - 2, 2);
    }
    unsigned long now = micros();
    if (now - jamLastSwitch >= JAM_SWITCH_INTERVAL_US) {
        jamLastSwitch = now;
        jamChannel++;
        jamChannelPackets = 0;
        if (jamChannel > 125) jamChannel = 0;
        radio.setChannel(jamChannel);
        yield();
    }
    return jamChannel;
}
const int8_t* nrf24GetJamHistory() { return jamHistory; }
uint32_t nrf24GetJamTotalPackets() { return jamTotalPackets; }
uint32_t nrf24GetJamChannelPackets() { return jamChannelPackets; }
uint8_t nrf24GetDeviceCount() { return nrfDeviceCount; }
NRFDevice* nrf24GetDevice(uint8_t index) { if (index < nrfDeviceCount) return &nrfDevices[index]; return nullptr; }
bool nrf24IsJammerActive() { return nrf24JammerActive; }
bool nrf24IsAvailable() { return radio.isChipConnected(); }
