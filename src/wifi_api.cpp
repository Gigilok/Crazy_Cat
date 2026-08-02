// ============================================================
// wifi_api.cpp - Servidor HTTP REST (Safe Action & PCAP Edition)
// ============================================================
#include "wifi_api.h"
#include "config.h"
#include "wifi_handshake.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

// ============================================================
// FORWARD DECLARATIONS
// ============================================================
extern MenuItem* currentMenuItems;
extern uint8_t currentMenuItemCount;
extern const char* currentMenuTitle;
extern int8_t menuIndex;
extern int8_t menuMaxIndex;
extern bool scannerRunning;
extern bool capturing;
extern unsigned long captureStartTime;
extern void enterMenu(MenuState state);
extern void goBack();

extern void scanNetworks();
extern uint8_t getNetworkCount();
extern NetworkInfo* getNetwork(uint8_t);
extern void startDeauth(uint8_t);
extern void stopDeauth();
extern void startEvilTwin(uint8_t);
extern void stopEvilTwin();
extern void startFakeAP(const char*);
extern void stopFakeAP();

extern bool nrf24JammerActive;
extern void nrf24StartJammer();
extern void nrf24StopJammer();
extern bool nrf24IsScanning();
extern void nrf24StartScan();
extern void nrf24StopScan();
extern void nrf24SpecStart();
extern void nrf24SpecStop();
extern bool nrf24SpecIsRunning();
extern uint32_t nrf24SpecGetFrames();
extern int8_t nrf24SpecGetBarValue(int);
extern int8_t nrf24SpecGetPeakValue(int);
extern bool nrf24SpecGetWaterfallPixel(int, int);
extern const int8_t* nrf24GetScanBarData();
extern uint32_t nrf24GetScanTotalPackets();
extern uint8_t nrf24GetSavedCount();
extern uint8_t nrf24GetDetectedCount();
extern uint8_t nrf24GetAnalyzeSelected();

extern bool cc1101CopyActive;
extern void cc1101StartCapture();
extern void cc1101StopCapture();
extern void cc1101ReplaySignal(uint8_t);
extern uint8_t cc1101GetSavedCount();
extern SignalData* cc1101GetSignal(uint8_t);
extern void cc1101TransmitRaw(uint32_t frequency, uint16_t* timings, uint8_t length);
extern void cc1101StartSubGHzJammer();
extern void cc1101StopSubGHzJammer();
extern void cc1101StartRollJam();
extern void cc1101StopRollJam();
extern void cc1101ClearSavedSignals();
extern void cc1101StartAnalyzer();
extern void cc1101StopAnalyzer();
extern bool cc1101AnalyzerIsRunning();
extern uint16_t cc1101GetAnalyzerValue(int idx);
extern uint32_t cc1101GetAnalyzerFreq(int idx);

extern uint8_t btDeviceCount;
extern void startBTScan();
extern void startBTJammer(uint8_t);
extern void stopBTJammer();
extern void stopBTScan();
extern uint8_t getBTDeviceCount();
extern BTDevice* getBTDevice(uint8_t);
extern bool isBLEAvailable();
extern bool isBTScanning();

extern bool bfRunning;
extern void startGateBruteForce();
extern void stopBruteForce();
extern void startCarBruteForce(uint8_t);
extern uint32_t getCurrentBFIndex();
extern uint32_t getTotalBFCount(uint8_t, uint8_t);
extern uint8_t getCarBrandCount();
extern bool bfIsGate;
extern uint8_t bfCarBrand;
extern const char* getCarBrandName(uint8_t);

extern bool droneJammerActive;
extern void startDroneJammer();
extern void stopDroneJammer();
extern bool cameraFreezeActive;
extern void startCameraFreeze();
extern void stopCameraFreeze();

extern void initConnection(int);
extern void setBrightness(uint8_t brightness);

extern uint8_t* getPcapData(size_t* outLen);

// ============================================================
// SERVER
// ============================================================
static WebServer apiServer(8080);
static bool apiRunning = false;

static volatile bool pendingDeauthStart = false;
static volatile bool pendingEvilTwinStart = false;
static volatile bool pendingDeauthStop = false;
static volatile bool pendingEvilTwinStop = false;
static volatile int pendingNetIdx = -1;

static void sendJSON(int code, const String& json) {
    apiServer.sendHeader("Access-Control-Allow-Origin", "*");
    apiServer.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    apiServer.sendHeader("Access-Control-Allow-Headers", "Content-Type");
    apiServer.sendHeader("Connection", "close");
    apiServer.send(code, "application/json", json);
}

static void sendOK(const String& msg) {
    DynamicJsonDocument doc(256);
    doc["status"] = "ok";
    doc["message"] = msg;
    String out;
    serializeJson(doc, out);
    sendJSON(200, out);
}

static void sendERR(const String& msg) {
    DynamicJsonDocument doc(256);
    doc["status"] = "error";
    doc["message"] = msg;
    String out;
    serializeJson(doc, out);
    sendJSON(400, out);
}

// ============================================================
// ENDPOINTS
// ============================================================

static void handleStatus() {
    DynamicJsonDocument doc(512);
    doc["menu"] = (int)currentMenu;
    doc["menu_name"] = currentMenuTitle ? currentMenuTitle : "";
    doc["wifi_enabled"] = wifiEnabled;
    doc["deauth_active"] = deauthActive;
    doc["eviltwin_active"] = evilTwinActive;
    doc["handshake_status"] = getHandshakeStatus();
    doc["handshake_complete"] = isHandshakeComplete();
    doc["nrf24_jammer"] = nrf24JammerActive;
    doc["nrf24_scanning"] = nrf24IsScanning();
    doc["cc1101_capturing"] = cc1101CopyActive;
    doc["drone_jammer"] = droneJammerActive;
    doc["camera_freeze"] = cameraFreezeActive;
    doc["bt_jammer"] = btJammerActive;
    doc["ble_available"] = isBLEAvailable();
    doc["ble_scanning"] = isBTScanning();
    doc["bruteforce"] = bfRunning;
    doc["network_count"] = networkCount;
    doc["btdevice_count"] = btDeviceCount;
    doc["cc1101_signals"] = cc1101GetSavedCount();
    doc["nrf24_signals"] = nrf24GetSavedCount();
    String out;
    serializeJson(doc, out);
    sendJSON(200, out);
}

static void handleNetworks() {
    DynamicJsonDocument doc(2048);
    JsonArray nets = doc.createNestedArray("networks");
    for (int i = 0; i < (int)networkCount; i++) {
        NetworkInfo* net = &scannedNetworks[i];
        JsonObject obj = nets.createNestedObject();
        obj["id"] = i;
        obj["ssid"] = net->ssid;
        obj["channel"] = net->channel;
        obj["rssi"] = net->rssi;
        obj["encrypted"] = net->encrypted;
        char bssid[18];
        snprintf(bssid, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
            net->bssid[0], net->bssid[1], net->bssid[2],
            net->bssid[3], net->bssid[4], net->bssid[5]);
        obj["bssid"] = bssid;
    }
    String out;
    serializeJson(doc, out);
    sendJSON(200, out);
}

static void handleScanNetworks() {
    yield();
    scanNetworks(); 
    yield();
    DynamicJsonDocument doc(2048);
    doc["status"] = "ok";
    doc["message"] = "Scan complete";
    doc["count"] = networkCount;
    JsonArray nets = doc.createNestedArray("networks");
    for (int i = 0; i < (int)networkCount; i++) {
        NetworkInfo* net = &scannedNetworks[i];
        JsonObject obj = nets.createNestedObject();
        obj["id"] = i;
        obj["ssid"] = net->ssid;
        obj["channel"] = net->channel;
        obj["rssi"] = net->rssi;
        obj["encrypted"] = net->encrypted;
        char bssid[18];
        snprintf(bssid, 18, "%02X:%02X:%02X:%02X:%02X:%02x",
            net->bssid[0], net->bssid[1], net->bssid[2],
            net->bssid[3], net->bssid[4], net->bssid[5]);
        obj["bssid"] = bssid;
    }
    String out;
    serializeJson(doc, out);
    sendJSON(200, out);
}

static void handleDeauthStart() {
    if (!apiServer.hasArg("id")) { sendERR("Missing id"); return; }
    int netIdx = apiServer.arg("id").toInt();
    if (netIdx < 0 || netIdx >= (int)networkCount) { sendERR("Invalid network id"); return; }
    pendingNetIdx = netIdx;
    pendingDeauthStart = true; 
    sendOK("Deauth started");
}

static void handleDeauthStop() {
    pendingDeauthStop = true;
    sendOK("Deauth stopped");
}

static void handleEvilTwinStart() {
    if (!apiServer.hasArg("id")) { sendERR("Missing id"); return; }
    int netIdx = apiServer.arg("id").toInt();
    if (netIdx < 0 || netIdx >= (int)networkCount) { sendERR("Invalid network id"); return; }
    pendingNetIdx = netIdx;
    pendingEvilTwinStart = true;
    sendOK("Evil Twin started");
}

static void handleEvilTwinStop() {
    pendingEvilTwinStop = true;
    sendOK("Evil Twin stopped");
}

static void handleHandshakeStatus() {
    DynamicJsonDocument doc(256);
    doc["capturing"] = isHandshakeCapturing();
    doc["complete"] = isHandshakeComplete();
    doc["frames"] = getHandshakeMessageCount();
    doc["status"] = getHandshakeStatus();
    String out;
    serializeJson(doc, out);
    sendJSON(200, out);
}

static void handleHandshakeDownload() {
    if (getHandshakeMessageCount() == 0) {
        sendERR("No handshake captured. Use Evil Twin first");
        return;
    }
    size_t pcapLen = 0;
    uint8_t* pcapData = getPcapData(&pcapLen);
    if (pcapData == nullptr || pcapLen == 0) {
        sendERR("Failed to build PCAP file.");
        return;
    }
    apiServer.sendHeader("Content-Disposition", "attachment; filename=handshake.pcap");
    apiServer.sendHeader("Content-Length", String(pcapLen));
    apiServer.sendHeader("Connection", "close");
    apiServer.send(200, "application/vnd.tcpdump.pcap", "");
    apiServer.sendContent((const char*)pcapData, pcapLen);
    free(pcapData);
}

static void handleNRF24JammerStart() { if (!nrf24JammerActive) nrf24StartJammer(); sendOK("NRF24 Jammer started"); }
static void handleNRF24JammerStop() { nrf24StopJammer(); sendOK("NRF24 Jammer stopped"); }
static void handleNRF24ScannerStart() { scannerRunning = true; nrf24SpecStart(); sendOK("NRF24 Scanner started"); }
static void handleNRF24ScannerStop() { scannerRunning = false; nrf24SpecStop(); sendOK("NRF24 Scanner stopped"); }

static void handleNRF24ScanData() {
    DynamicJsonDocument doc(512);
    const int8_t* bars = nrf24GetScanBarData();
    JsonArray arr = doc.createNestedArray("bars");
    for (int i = 0; i < 16; i++) arr.add(bars[i]);
    doc["packets"] = nrf24GetScanTotalPackets();
    String out;
    serializeJson(doc, out);
    sendJSON(200, out);
}

// NOVO: scanner estilo Flipper (64 barras + peaks)
static void handleNRF24SpecData() {
    DynamicJsonDocument doc(2048);
    JsonArray barsArr  = doc.createNestedArray("bars");
    JsonArray peaksArr = doc.createNestedArray("peaks");
    for (int i = 0; i < 64; i++) {
        barsArr.add((int)nrf24SpecGetBarValue(i));
        peaksArr.add((int)nrf24SpecGetPeakValue(i));
    }
    doc["frames"]     = nrf24SpecGetFrames();
    doc["running"]    = nrf24SpecIsRunning();
    doc["max_height"] = 40;
    doc["channels"]   = 64;
    String out;
    serializeJson(doc, out);
    sendJSON(200, out);
}

static void handleCC1101Copy() {
    yield();
    if (!capturing) {
        capturing = true; captureStartTime = millis(); cc1101StartCapture(); capturing = false;
    }
    yield();
    sendOK("CC1101 copy started");
}

static void handleCC1101Replay() {
    if (!apiServer.hasArg("id")) { sendERR("Missing id"); return; }
    int idx = apiServer.arg("id").toInt();
    if (idx < (int)cc1101GetSavedCount()) { cc1101ReplaySignal(idx); sendOK("CC1101 replay"); } 
    else { sendERR("Invalid signal id"); }
}

static void handleCC1101Signals() {
    DynamicJsonDocument doc(1024);
    JsonArray arr = doc.createNestedArray("signals");
    for (int i = 0; i < (int)cc1101GetSavedCount(); i++) {
        SignalData* sig = cc1101GetSignal(i);
        if (!sig || !sig->valid) continue;
        JsonObject obj = arr.createNestedObject();
        obj["id"] = i; obj["name"] = sig->name; obj["frequency"] = sig->frequency;
    }
    String out; serializeJson(doc, out); sendJSON(200, out);
}

static void handleCC1101GetRaw() {
    if (!apiServer.hasArg("id")) { sendERR("Missing id"); return; }
    int idx = apiServer.arg("id").toInt();
    SignalData* sig = cc1101GetSignal(idx);
    if (!sig || !sig->valid) { sendERR("Invalid signal id"); return; }
    
    DynamicJsonDocument doc(4096);
    doc["frequency"] = sig->frequency;
    doc["length"] = sig->length;
    JsonArray timings = doc.createNestedArray("timings");
    for (int i = 0; i < sig->length; i++) {
        timings.add(sig->timings[i]);
    }
    String out;
    serializeJson(doc, out);
    sendJSON(200, out);
}

static void handleCC1101TransmitRaw() {
    if (!apiServer.hasArg("plain")) { sendERR("Missing body"); return; }
    DynamicJsonDocument doc(4096);
    DeserializationError error = deserializeJson(doc, apiServer.arg("plain"));
    if (error) { sendERR("Invalid JSON"); return; }
    
    uint32_t freq = doc["frequency"];
    JsonArray timings = doc["timings"];
    
    uint8_t length = timings.size();
    if (length == 0 || length > 200) { sendERR("Invalid timings length"); return; }
    
    uint16_t buf[200];
    for (int i = 0; i < length; i++) {
        buf[i] = timings[i];
    }
    
    cc1101TransmitRaw(freq, buf, length);
    sendOK("Raw signal transmitted");
}

// NOVOS: CC1101 Jammer, RollJam, Clear, Analyzer
static void handleCC1101JammerStart() { cc1101StartSubGHzJammer(); sendOK("CC1101 Jammer started"); }
static void handleCC1101JammerStop() { cc1101StopSubGHzJammer(); sendOK("CC1101 Jammer stopped"); }
static void handleCC1101RollJamStart() { cc1101StartRollJam(); sendOK("CC1101 RollJam started"); }
static void handleCC1101RollJamStop() { cc1101StopRollJam(); sendOK("CC1101 RollJam stopped"); }
static void handleCC1101Clear() { cc1101ClearSavedSignals(); sendOK("CC1101 signals cleared"); }
static void handleCC1101AnalyzerStart() { cc1101StartAnalyzer(); sendOK("CC1101 Analyzer started"); }
static void handleCC1101AnalyzerStop() { cc1101StopAnalyzer(); sendOK("CC1101 Analyzer stopped"); }
static void handleCC1101AnalyzerData() {
    DynamicJsonDocument doc(4096);
    JsonArray barsArr = doc.createNestedArray("bars");
    JsonArray freqsArr = doc.createNestedArray("freqs");
    for (int i = 0; i < 64; i++) {
        barsArr.add(cc1101GetAnalyzerValue(i));
        freqsArr.add((uint32_t)cc1101GetAnalyzerFreq(i));
    }
    doc["running"] = cc1101AnalyzerIsRunning();
    String out;
    serializeJson(doc, out);
    sendJSON(200, out);
}

static void handleDroneJammerStart() { if (!droneJammerActive) startDroneJammer(); sendOK("Drone jammer started"); }
static void handleDroneJammerStop() { stopDroneJammer(); sendOK("Drone jammer stopped"); }
static void handleCameraFreezeStart() { if (!cameraFreezeActive) startCameraFreeze(); sendOK("Camera freeze started"); }
static void handleCameraFreezeStop() { stopCameraFreeze(); sendOK("Camera freeze stopped"); }

static void handleBTScan() {
    if (!isBLEAvailable()) { sendERR("BLE not available"); return; }
    if (isBTScanning()) { sendERR("Scan already in progress"); return; }
    btDeviceCount = 0; startBTScan(); sendOK("BT scan started (15s async)");
}

static void handleBTDevices() {
    DynamicJsonDocument doc(1024);
    JsonArray arr = doc.createNestedArray("devices");
    for (int i = 0; i < (int)btDeviceCount; i++) {
        BTDevice* dev = getBTDevice(i);
        if (!dev) continue;
        JsonObject obj = arr.createNestedObject();
        obj["id"] = i; obj["name"] = dev->name; obj["rssi"] = dev->rssi;
    }
    String out; serializeJson(doc, out); sendJSON(200, out);
}

static void handleBTJammerStart() {
    if (!isBLEAvailable()) { sendERR("BLE not available"); return; }
    if (!apiServer.hasArg("id")) { sendERR("Missing id"); return; }
    int idx = apiServer.arg("id").toInt();
    if (idx < (int)btDeviceCount) { startBTJammer(idx); sendOK("BT jammer started"); } 
    else { sendERR("Invalid device id"); }
}
static void handleBTJammerStop() { stopBTJammer(); sendOK("BT jammer stopped"); }

static void handleBTScanStatus() {
    DynamicJsonDocument doc(256);
    doc["ble_available"] = isBLEAvailable();
    doc["scanning"] = isBTScanning();
    doc["device_count"] = btDeviceCount;
    String out; serializeJson(doc, out); sendJSON(200, out);
}

static void handleBFGateStart() { if (!bfRunning) startGateBruteForce(); sendOK("BF Gate started"); }
static void handleBFGateStop() { stopBruteForce(); sendOK("BF Gate stopped"); }
static void handleBFCarStart() {
    if (!apiServer.hasArg("brand")) { sendERR("Missing brand"); return; }
    int brand = apiServer.arg("brand").toInt();
    if (brand < getCarBrandCount()) { if (!bfRunning) startCarBruteForce(brand); sendOK("BF Car started"); } 
    else { sendERR("Invalid brand"); }
}
static void handleBFCarStop() { stopBruteForce(); sendOK("BF Car stopped"); }

static void handleBFStatus() {
    DynamicJsonDocument doc(512);
    doc["running"] = bfRunning;
    doc["current_index"] = getCurrentBFIndex();
    doc["gate_total"] = getTotalBFCount(0, 0);
    doc["brand_count"] = getCarBrandCount();
    if (bfRunning) {
        if (bfIsGate) {
            doc["mode"] = "gate";
            doc["total"] = getTotalBFCount(0, 0);
        } else {
            doc["mode"] = "car";
            doc["brand"] = bfCarBrand;
            doc["brand_name"] = getCarBrandName(bfCarBrand);
            doc["total"] = getTotalBFCount(1, bfCarBrand);
        }
    } else {
        doc["mode"] = "idle";
        doc["total"] = 0;
    }
    String out; serializeJson(doc, out); sendJSON(200, out);
}

// NOVO: lista de marcas de carro
static void handleBFCarBrands() {
    DynamicJsonDocument doc(1024);
    JsonArray arr = doc.createNestedArray("brands");
    for (int i = 0; i < getCarBrandCount(); i++) {
        JsonObject obj = arr.createNestedObject();
        obj["id"] = i;
        obj["name"] = getCarBrandName(i);
        obj["total"] = getTotalBFCount(1, i);
    }
    String out;
    serializeJson(doc, out);
    sendJSON(200, out);
}

static void handleSetBrightness() {
    if (!apiServer.hasArg("value")) { sendERR("Missing value"); return; }
    int val = apiServer.arg("value").toInt();
    if (val >= 0 && val <= 255) { screenBrightness = val; setBrightness(screenBrightness); sendOK("Brightness set"); } 
    else { sendERR("Invalid value (0-255)"); }
}

static void handleWiFiToggle() {
    if (!wifiEnabled && !isBLEAvailable()) {
        // Turning WiFi ON, BLE might need re-init later
    }
    toggleWiFi();
    DynamicJsonDocument doc(256);
    doc["status"] = "ok";
    doc["wifi_enabled"] = wifiEnabled;
    String out;
    serializeJson(doc, out);
    sendJSON(200, out);
}

static void handleMenuNavigate() {
    if (!apiServer.hasArg("to")) { sendERR("Missing to"); return; }
    String target = apiServer.arg("to");
    if (target == "MAIN") enterMenu(MENU_MAIN);
    else if (target == "NRF24") enterMenu(MENU_NRF24);
    else if (target == "CC1101") enterMenu(MENU_CC1101);
    else if (target == "ATTACKS") enterMenu(MENU_ATTACKS);
    else if (target == "NETWORKS") enterMenu(MENU_NETWORKS);
    else if (target == "SETTINGS") enterMenu(MENU_SETTINGS);
    else { sendERR("Unknown menu"); return; }
    sendOK("Menu changed");
}

static void handleButton() {
    if (!apiServer.hasArg("action")) { sendERR("Missing action"); return; }
    String action = apiServer.arg("action");
    if (action == "UP") { if (menuIndex > 0) menuIndex--; } 
    else if (action == "DOWN") { if (menuIndex < menuMaxIndex) menuIndex++; } 
    else if (action == "SELECT") { if (currentMenuItems && menuIndex < (int)currentMenuItemCount) enterMenu(currentMenuItems[menuIndex].state); } 
    else if (action == "BACK") { goBack(); } 
    else { sendERR("Unknown action"); return; }
    sendOK("Button " + action + " pressed");
}

// ============================================================
// SETUP
// ============================================================
void startAPIServer() {
    if (apiRunning) return;

    apiServer.onNotFound([]() {
        if (apiServer.method() == HTTP_OPTIONS) {
            apiServer.sendHeader("Access-Control-Allow-Origin", "*");
            apiServer.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
            apiServer.sendHeader("Access-Control-Allow-Headers", "Content-Type");
            apiServer.send(204);
            return;
        }
        sendERR("Endpoint not found: " + apiServer.uri());
    });

    apiServer.on("/api/status", HTTP_GET, handleStatus);
    apiServer.on("/api/networks", HTTP_GET, handleNetworks);
    apiServer.on("/api/networks/scan", HTTP_POST, handleScanNetworks);
    apiServer.on("/api/deauth/start", HTTP_POST, handleDeauthStart);
    apiServer.on("/api/deauth/stop", HTTP_POST, handleDeauthStop);
    apiServer.on("/api/eviltwin/start", HTTP_POST, handleEvilTwinStart);
    apiServer.on("/api/eviltwin/stop", HTTP_POST, handleEvilTwinStop);
    apiServer.on("/api/handshake", HTTP_GET, handleHandshakeStatus);
    apiServer.on("/api/handshake/download", HTTP_GET, handleHandshakeDownload);
    apiServer.on("/api/nrf24/jammer/start", HTTP_POST, handleNRF24JammerStart);
    apiServer.on("/api/nrf24/jammer/stop", HTTP_POST, handleNRF24JammerStop);
    apiServer.on("/api/nrf24/scanner/start", HTTP_POST, handleNRF24ScannerStart);
    apiServer.on("/api/nrf24/scanner/stop", HTTP_POST, handleNRF24ScannerStop);
    apiServer.on("/api/nrf24/scan", HTTP_GET, handleNRF24ScanData);
    apiServer.on("/api/nrf24/spec", HTTP_GET, handleNRF24SpecData);
    apiServer.on("/api/cc1101/copy", HTTP_POST, handleCC1101Copy);
    apiServer.on("/api/cc1101/replay", HTTP_POST, handleCC1101Replay);
    apiServer.on("/api/cc1101/signals", HTTP_GET, handleCC1101Signals);
    apiServer.on("/api/cc1101/raw", HTTP_GET, handleCC1101GetRaw);
    apiServer.on("/api/cc1101/transmit_raw", HTTP_POST, handleCC1101TransmitRaw);
    apiServer.on("/api/cc1101/jammer/start", HTTP_POST, handleCC1101JammerStart);
    apiServer.on("/api/cc1101/jammer/stop", HTTP_POST, handleCC1101JammerStop);
    apiServer.on("/api/cc1101/rolljam/start", HTTP_POST, handleCC1101RollJamStart);
    apiServer.on("/api/cc1101/rolljam/stop", HTTP_POST, handleCC1101RollJamStop);
    apiServer.on("/api/cc1101/clear", HTTP_POST, handleCC1101Clear);
    apiServer.on("/api/cc1101/analyzer/start", HTTP_POST, handleCC1101AnalyzerStart);
    apiServer.on("/api/cc1101/analyzer/stop", HTTP_POST, handleCC1101AnalyzerStop);
    apiServer.on("/api/cc1101/analyzer/data", HTTP_GET, handleCC1101AnalyzerData);
    apiServer.on("/api/attack/drone/jammer/start", HTTP_POST, handleDroneJammerStart);
    apiServer.on("/api/attack/drone/jammer/stop", HTTP_POST, handleDroneJammerStop);
    apiServer.on("/api/attack/camera/freeze/start", HTTP_POST, handleCameraFreezeStart);
    apiServer.on("/api/attack/camera/freeze/stop", HTTP_POST, handleCameraFreezeStop);
    apiServer.on("/api/attack/bt/scan", HTTP_POST, handleBTScan);
    apiServer.on("/api/attack/bt/devices", HTTP_GET, handleBTDevices);
    apiServer.on("/api/attack/bt/jammer/start", HTTP_POST, handleBTJammerStart);
    apiServer.on("/api/attack/bt/jammer/stop", HTTP_POST, handleBTJammerStop);
    apiServer.on("/api/attack/bt/status", HTTP_GET, handleBTScanStatus);
    apiServer.on("/api/attack/bf/gate/start", HTTP_POST, handleBFGateStart);
    apiServer.on("/api/attack/bf/gate/stop", HTTP_POST, handleBFGateStop);
    apiServer.on("/api/attack/bf/car/start", HTTP_POST, handleBFCarStart);
    apiServer.on("/api/attack/bf/car/stop", HTTP_POST, handleBFCarStop);
    apiServer.on("/api/attack/bf/status", HTTP_GET, handleBFStatus);
    apiServer.on("/api/attack/bf/brands", HTTP_GET, handleBFCarBrands);
    apiServer.on("/api/settings/brightness", HTTP_POST, handleSetBrightness);
    apiServer.on("/api/settings/wifi/toggle", HTTP_POST, handleWiFiToggle);
    apiServer.on("/api/menu/navigate", HTTP_POST, handleMenuNavigate);
    apiServer.on("/api/btn", HTTP_POST, handleButton);

    apiServer.begin();
    apiRunning = true;
    Serial.println("[API] HTTP Server started on :8080");
}

void stopAPIServer() {
    if (!apiRunning) return;
    apiServer.stop();
    apiRunning = false;
    Serial.println("[API] HTTP Server stopped");
}

// ============================================================
// LOOP - adições mínimas para ferramentas funcionarem via APK
// ============================================================

// Forward declarations necessárias para os loops
extern void cc1101CaptureLoop();
extern void cc1101RollJamLoop();
extern bool cc1101RollJamActive;
extern void cc1101AnalyzerLoop();
extern bool cc1101AnalyzerIsRunning();
// NRF24 loops - ANTES faltavam, scanner não atualizava via APK
extern int  nrf24JammerLoop();
extern bool nrf24JammerActive;
extern void nrf24SpecScan();
extern void nrf24ScanLoop();
extern bool nrf24IsScanning();
extern void nrf24AnalyzeTick();
extern bool nrf24IsAnalyzing();
extern bool scannerRunning;
// BruteForce loop
extern void bfLoop();
extern bool bfRunning;

void apiLoop() {
    if (apiRunning) apiServer.handleClient();

    if (pendingDeauthStart) {
        pendingDeauthStart = false;
        startDeauth(pendingNetIdx);
    }
    if (pendingEvilTwinStart) {
        pendingEvilTwinStart = false;
        startEvilTwin(pendingNetIdx);
    }
    if (pendingDeauthStop) {
        pendingDeauthStop = false;
        stopDeauth();
    }
    if (pendingEvilTwinStop) {
        pendingEvilTwinStop = false;
        stopEvilTwin();
    }

    // TOOL LOOPS: CC1101 + NRF24 + BruteForce
    // CORREÇÃO CRÍTICA: nrf24SpecScan() e nrf24ScanLoop() ANTES não eram
    // chamados via API. Só rodavam dentro de renderNRF24Scanner() no menu.
    // Quando o APK iniciava o scanner, os dados ficavam sempre zerados.
    if (cc1101CopyActive)              cc1101CaptureLoop();
    if (cc1101RollJamActive)           cc1101RollJamLoop();
    if (cc1101AnalyzerIsRunning())     cc1101AnalyzerLoop();
    if (nrf24JammerActive)             nrf24JammerLoop();
    if (scannerRunning)                nrf24SpecScan();
    if (nrf24IsScanning())             nrf24ScanLoop();
    if (nrf24IsAnalyzing())            nrf24AnalyzeTick();
    if (bfRunning)                     bfLoop();
}

bool isAPIServerRunning() { return apiRunning; }
