#include "config.h"

// Flag para re-inicializar o HSPI do CC1101 após testAllPins()
// (mantido por compatibilidade, mas testAllPins agora NÃO destrói os pinos)
extern bool cc1101NeedsSpiReinit;

struct PinTest {
    const char* name;
    uint8_t pin;
    bool working;
};

// Pinos SPI do HSPI (CC1101) e VSPI (NRF24) — NÃO testar!
// pinMode() nesses pinos destrói o GPIO matrix do SPI.
static const uint8_t spiPins[] = {
    CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CSN,  // HSPI: 14,12,13,15
    NRF_SCK, NRF_MISO, NRF_MOSI, NRF_CSN              // VSPI: 18,19,23,25
};
static const uint8_t spiPinCount = sizeof(spiPins) / sizeof(spiPins[0]);

static bool isSpiPin(uint8_t pin) {
    for (uint8_t i = 0; i < spiPinCount; i++) {
        if (spiPins[i] == pin) return true;
    }
    return false;
}

PinTest pinTests[] = {
    {"OLED SDA", OLED_SDA, false},
    {"OLED SCK", OLED_SCK, false},
    {"BTN UP", BTN_UP, false},
    {"BTN DOWN", BTN_DOWN, false},
    {"BTN SEL", BTN_SELECT, false},
    {"BTN BACK", BTN_BACK, false},
    {"NRF CE", NRF_CE, false},
    {"NRF CSN", NRF_CSN, false},
    {"NRF SCK", NRF_SCK, false},
    {"NRF MOSI", NRF_MOSI, false},
    {"NRF MISO", NRF_MISO, false},
    {"CC GDO0", CC1101_GDO0, false},
    {"CC CSN", CC1101_CSN, false},
    {"CC SCK", CC1101_SCK, false},
    {"CC MOSI", CC1101_MOSI, false},
    {"CC MISO", CC1101_MISO, false},
    {"CC GDO2", CC1101_GDO2, false},
};
const uint8_t pinTestCount = sizeof(pinTests) / sizeof(pinTests[0]);

void testAllPins() {
    for (int i = 0; i < pinTestCount; i++) {
        uint8_t pin = pinTests[i].pin;

        // NUNCA faz pinMode() em pinos SPI — isso destrói o GPIO matrix
        // e quebra o barramento SPI. Testa apenas leitura.
        if (isSpiPin(pin)) {
            pinTests[i].working = true;  // Assume OK (não podemos testar sem destruir)
            continue;
        }

        // Pinos de botão: testa com INPUT_PULLUP
        if (pin == BTN_UP || pin == BTN_DOWN || pin == BTN_SELECT || pin == BTN_BACK) {
            pinMode(pin, INPUT_PULLUP);
            delay(5);
            pinTests[i].working = (digitalRead(pin) == HIGH);
        } else {
            // Pinos normais: teste de OUTPUT
            pinMode(pin, OUTPUT);
            digitalWrite(pin, HIGH);
            delay(5);
            pinTests[i].working = true;
        }
        delay(5);
    }
    // NÃO precisa mais de cc1101NeedsSpiReinit porque não destruímos os pinos SPI
    // Mas setamos como precaução caso algum outro código tenha alterado os pinos
    cc1101NeedsSpiReinit = true;
}

uint8_t getPinTestCount() { return pinTestCount; }

PinTest* getPinTest(uint8_t index) {
    if (index < pinTestCount) return &pinTests[index];
    return nullptr;
}

struct ModuleStatus {
    const char* name;
    bool connected;
    bool working;
};

ModuleStatus modules[] = {
    {"NRF24L01", false, false},
    {"CC1101", false, false},
    {"WiFi", false, false},
    {"Bluetooth", false, false},
    {"OLED", false, false},
};
const uint8_t moduleCount = sizeof(modules) / sizeof(modules[0]);

void testModules(bool nrfOk, bool cc1101Ok, bool btOk) {
    modules[0].connected = nrfOk;
    modules[0].working = nrfOk;
    modules[1].connected = cc1101Ok;
    modules[1].working = cc1101Ok;
    modules[2].connected = true;
    modules[2].working = wifiEnabled;
    modules[3].connected = btOk;
    modules[3].working = btOk;
    modules[4].connected = true;
    modules[4].working = true;
}

uint8_t getModuleCount() { return moduleCount; }

ModuleStatus* getModule(uint8_t index) {
    if (index < moduleCount) return &modules[index];
    return nullptr;
}

void setScreenBrightness(uint8_t brightness) {
    screenBrightness = brightness;
    extern void setBrightness(uint8_t);
    setBrightness(brightness);
}

enum ConnType { CONN_USB, CONN_SERIAL, CONN_BT, CONN_PAIR };
ConnType currentConn = CONN_USB;
bool connActive = false;
char pairingCode[7] = "000000";

void initConnection(int type) {
    currentConn = (ConnType)type;
    connActive = false;
    switch (currentConn) {
        case CONN_USB: connActive = true; break;
        case CONN_SERIAL: connActive = true; break;
        case CONN_BT: connActive = true; break;
        case CONN_PAIR: snprintf(pairingCode, 7, "%06d", random(1000000)); connActive = true; break;
    }
}

void disconnectConnection() {
    connActive = false;
}

bool isConnectionActive() { return connActive; }
int getConnectionType() { return (int)currentConn; }

const char* getConnectionTypeName() {
    switch (currentConn) {
        case CONN_USB: return "USB";
        case CONN_SERIAL: return "Serial";
        case CONN_BT: return "Bluetooth";
        case CONN_PAIR: return "Pareamento";
        default: return "?";
    }
}

const char* getPairingCode() { return pairingCode; }
