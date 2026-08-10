#include "config.h"

extern bool cc1101NeedsSpiReinit;

struct PinTest { const char* name; uint8_t pin; bool working; };

// Todos os pinos que NÃO devem ser alterados
static const uint8_t protectedPins[] = {
    CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CSN,
    NRF_SCK, NRF_MISO, NRF_MOSI, NRF_CSN,
    CC1101_GDO0, CC1101_GDO2,
    NRF_CE
};
static const uint8_t protectedCount = sizeof(protectedPins) / sizeof(protectedPins[0]);

static bool isProtectedPin(uint8_t pin) {
    for (uint8_t i = 0; i < protectedCount; i++)
        if (protectedPins[i] == pin) return true;
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
        if (isProtectedPin(pin)) {
            pinMode(pin, INPUT);
            delay(1);
            pinTests[i].working = true;
            continue;
        }
        if (pin == BTN_UP || pin == BTN_DOWN || pin == BTN_SELECT || pin == BTN_BACK) {
            pinMode(pin, INPUT_PULLUP);
            delay(5);
            pinTests[i].working = (digitalRead(pin) == HIGH);
        } else {
            pinMode(pin, OUTPUT);
            digitalWrite(pin, HIGH);
            delay(5);
            pinTests[i].working = true;
        }
        delay(5);
    }
    cc1101NeedsSpiReinit = true;
}

uint8_t getPinTestCount() { return pinTestCount; }
PinTest* getPinTest(uint8_t index) { if (index < pinTestCount) return &pinTests[index]; return nullptr; }

// ... (resto do arquivo permanece igual)
