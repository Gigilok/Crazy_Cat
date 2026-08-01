#include "config.h"

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
bool deauthActive = false;
bool droneJammerActive = false;
bool cameraFreezeActive = false;
bool btJammerActive = false;
bool bfRunning = false;

char capturedPassword[64] = {0};
bool passwordCaptured = false;
bool fakeAPEnabled = false;
bool evilTwinActive = false;
