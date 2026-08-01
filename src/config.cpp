#include "config.h"
#include <WiFi.h>

SignalData savedSignals[MAX_SAVED_SIGNALS];
NetworkInfo scannedNetworks[MAX_NETWORKS];
BTDevice btDevices[MAX_BT_DEVICES];
RemoteDevice remoteDevices[10];

uint8_t savedSignalCount = 0;
uint8_t networkCount = 0;
uint8_t btDeviceCount = 0;
uint8_t remoteDeviceCount = 0;

MenuState currentMenu = MENU_MAIN;
MenuState previousMenu = MENU_MAIN;
int8_t menuIndex = 0;
int8_t menuMaxIndex = 0;
bool menuRunning = true;
uint8_t screenBrightness = 255;

bool nrf24JammerActive = false;
bool cc1101CopyActive = false;
bool cc1101RollJamActive = false;
bool deauthActive = false;
bool droneJammerActive = false;
bool cameraFreezeActive = false;
bool btJammerActive = false;
bool bfRunning = false;

bool wifiEnabled = true;

char capturedPassword[64] = {0};
bool passwordCaptured = false;
bool fakeAPEnabled = false;
bool evilTwinActive = false;

void toggleWiFi() {
    if (wifiEnabled) {
        Serial.println("[WiFi] Turning OFF...");
        extern void stopAPIServer();
        stopAPIServer();
        WiFi.softAPdisconnect(true);
        WiFi.disconnect(true, true);
        delay(300);
        WiFi.mode(WIFI_OFF);
        delay(1500);
        wifiEnabled = false;
        Serial.println("[WiFi] OFF");
    } else {
        Serial.println("[WiFi] Turning ON...");
        WiFi.mode(WIFI_AP_STA);
        delay(500);
        WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
        delay(200);
        WiFi.softAP("TP_Link", "crazycat123", 6, 0, 4);
        delay(500);
        extern void startAPIServer();
        startAPIServer();
        wifiEnabled = true;
        Serial.println("[WiFi] ON");
    }
}

bool isWiFiEnabled() { return wifiEnabled; }
