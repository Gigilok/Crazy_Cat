#include "config.h"

unsigned long lastButtonPress = 0;
const unsigned long DEBOUNCE_DELAY = 150; // Reduzi de 200 para 150 para ficar mais responsivo

bool inputInit() {
    pinMode(BTN_UP, INPUT_PULLUP);
    pinMode(BTN_DOWN, INPUT_PULLUP);
    pinMode(BTN_SELECT, INPUT_PULLUP);
    pinMode(BTN_BACK, INPUT_PULLUP);
    return true;
}

ButtonState readButtons() {
    if (millis() - lastButtonPress < DEBOUNCE_DELAY) {
        return BTN_NONE;
    }

    if (digitalRead(BTN_UP) == LOW) {
        lastButtonPress = millis();
        return BTN_PRESSED_UP;
    }
    if (digitalRead(BTN_DOWN) == LOW) {
        lastButtonPress = millis();
        return BTN_PRESSED_DOWN;
    }
    if (digitalRead(BTN_SELECT) == LOW) {
        lastButtonPress = millis();
        return BTN_PRESSED_SELECT;
    }
    if (digitalRead(BTN_BACK) == LOW) {
        lastButtonPress = millis();
        return BTN_PRESSED_BACK;
    }

    return BTN_NONE;
}
