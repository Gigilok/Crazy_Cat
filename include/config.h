#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================
// PINOS - Crazy Cat v3.1 (CC1101 agora no VSPI)
// ============================================================
#define OLED_SCK    22
#define OLED_SDA    21

#define BTN_UP      5
#define BTN_DOWN    27
#define BTN_SELECT  32
#define BTN_BACK    33

#define NRF_CE      26
#define NRF_CSN     25
#define NRF_SCK     18
#define NRF_MOSI    23
#define NRF_MISO    19

// CC1101 agora no mesmo barramento VSPI do NRF24
#define CC1101_GDO0 12
#define CC1101_CSN  14
#define CC1101_SCK  18    // VSPI SCK
#define CC1101_MOSI 23    // VSPI MOSI
#define CC1101_MISO 19    // VSPI MISO
#define CC1101_GDO2 13

#define BUZZER_PIN  4

// ============================================================
// BATERIA - medidor de carga
// ============================================================
#define BATTERY_PIN         34
#define BATTERY_DIVIDER     2.0
#define BATTERY_FULL_MV     4200
#define BATTERY_EMPTY_MV    3000

// ============================================================
// CONFIGURACOES
// ============================================================
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_ADDRESS    0x3C

#define MAX_SAVED_SIGNALS   5
#define CAPTURE_DURATION    8000
#define MAX_NETWORKS        20
#define MAX_BT_DEVICES      15

// ============================================================
// ESTRUTURAS
// ============================================================
struct SignalData {
    uint16_t timings[200];
    uint8_t length;
    uint32_t frequency;
    uint8_t modulation;
    char name[16];
    bool valid;
};

struct NetworkInfo {
    char ssid[33];
    uint8_t bssid[6];
    int8_t rssi;
    uint8_t channel;
    bool encrypted;
};

struct BTDevice {
    char name[32];
    uint8_t address[6];
    int8_t rssi;
};

struct RemoteDevice {
    char ip[16];
    char name[32];
    uint8_t mac[6];
    uint16_t port;
};

// ============================================================
// ESTADOS
// ============================================================
enum MenuState {
    MENU_MAIN,
    MENU_NRF24, MENU_NRF24_JAMMER, MENU_NRF24_SCANNER, MENU_NRF24_ANALYZE, MENU_NRF24_ANALYZE_DETAIL,
    MENU_CC1101, MENU_CC1101_COPY, MENU_CC1101_REPLAY, MENU_CC1101_JAMMER, MENU_CC1101_ROLLJAM, MENU_CC1101_ANALYZER, MENU_CC1101_CLEAR,
    MENU_ATTACKS,
    MENU_ATTACK_DRONE, MENU_ATTACK_DRONE_JAMMER, MENU_ATTACK_DRONE_REMOTE, MENU_ATTACK_DRONE_LOCATE,
    MENU_ATTACK_DEAUTH,
    MENU_ATTACK_CAMERA, MENU_ATTACK_CAMERA_FREEZE,
    MENU_ATTACK_BLUETOOTH,
    MENU_ATTACK_BRUTEFORCE, MENU_ATTACK_BF_GATE, MENU_ATTACK_BF_CAR,
    MENU_NETWORKS, MENU_NET_PASSWORD, MENU_NET_DEAUTH, MENU_NET_REMOTE,
    MENU_SETTINGS, MENU_SETTINGS_PINS, MENU_SETTINGS_MODULES, MENU_SETTINGS_BRIGHTNESS,
    MENU_SETTINGS_RECORDS, MENU_SETTINGS_RECORDS_DETAIL, MENU_SETTINGS_RECORDS_DELETE,
    MENU_SETTINGS_WIFI, MENU_SETTINGS_CONNECTION
};

enum ButtonState { BTN_NONE, BTN_PRESSED_UP, BTN_PRESSED_DOWN, BTN_PRESSED_SELECT, BTN_PRESSED_BACK };

// ============================================================
// VARIAVEIS EXTERNAS
// ============================================================
extern SignalData savedSignals[MAX_SAVED_SIGNALS];
extern NetworkInfo scannedNetworks[MAX_NETWORKS];
extern BTDevice btDevices[MAX_BT_DEVICES];
extern RemoteDevice remoteDevices[10];

extern uint8_t savedSignalCount;
extern uint8_t networkCount;
extern uint8_t btDeviceCount;
extern uint8_t remoteDeviceCount;

struct MenuItem { const char* label; MenuState state; };

extern MenuState currentMenu;
extern MenuState previousMenu;
extern int8_t menuIndex;
extern int8_t menuMaxIndex;
extern bool menuRunning;
extern uint8_t screenBrightness;

extern bool nrf24JammerActive;
extern bool cc1101CopyActive;
extern bool cc1101RollJamActive;
extern bool deauthActive;
extern bool droneJammerActive;
extern bool cameraFreezeActive;
extern bool btJammerActive;
extern bool bfRunning;

extern bool wifiEnabled;

extern char capturedPassword[64];
extern bool passwordCaptured;
extern bool fakeAPEnabled;
extern bool evilTwinActive;

#define NRF_SCAN_HISTORY    64
#define NRF_SCAN_BARS       16
#define NRF_MAX_DETECTED    10

extern void startHandshakeCapture();
extern void stopHandshakeCapture();
extern bool isHandshakeCapturing();
extern const char* getHandshakeStatus();
extern uint8_t getHandshakeMessageCount();
extern bool isHandshakeComplete();
extern bool saveHandshakeToFile(const char* filename);
extern bool sendHandshakeViaBluetooth(const char* filename);
extern size_t getHandshakeFileSize(const char* filename);
extern void serveHandshakeHTTP();

extern void startAPIServer();
extern void stopAPIServer();
extern void apiLoop();
extern bool isAPIServerRunning();

// ============================================================
// BLUETOOTH - Interface publica
// ============================================================
extern bool bluetoothInit();
extern bool isBLEAvailable();
extern bool isBTScanning();
extern void stopBTScan();

// ============================================================
// CC1101 - Raw Transmit (Para Termux Keeloq)
// ============================================================
extern void cc1101TransmitRaw(uint32_t frequency, uint16_t* timings, uint8_t length);

// ============================================================
// WIFI TOGGLE
// ============================================================
extern void toggleWiFi();
extern bool isWiFiEnabled();

// ============================================================
// BRUTEFORCE - expostos para wifi_api.cpp
// ============================================================
extern bool bfIsGate;
extern uint8_t bfCarBrand;
extern uint32_t getCurrentBFIndex();
extern const char* getCarBrandName(uint8_t index);
extern uint32_t getTotalBFCount(uint8_t type, uint8_t brandIndex);
extern uint8_t getCarBrandCount();

// ============================================================
// CC1101 - expostos para wifi_attacks.cpp (drone jammer)
// ============================================================
extern bool cc1101Initialized;
extern void cc1101SetFrequency(uint32_t freqHz);
extern void cc1101WriteReg(uint8_t reg, uint8_t value);
extern void cc1101SendCommand(uint8_t cmd);

// ============================================================
// NRF24 - expostos para menu.cpp (powerDown ao sair do menu)
// ============================================================
extern void nrf24Sleep();

#endif
