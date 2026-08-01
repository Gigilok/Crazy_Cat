#include "config.h"

void buzzerInit() {
    // Inicia em alta impedância para não interferir no boot do ESP32
    pinMode(BUZZER_PIN, INPUT);
}

void beep(int durationMs = 50) {
    // Só configura como saída no momento do bip
    pinMode(BUZZER_PIN, OUTPUT);
    
    // Gera onda quadrada manual (2000Hz)
    int cycles = durationMs * 2; 
    for (int i = 0; i < cycles; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delayMicroseconds(250);
        digitalWrite(BUZZER_PIN, LOW);
        delayMicroseconds(250);
    }
    
    // Volta para INPUT imediatamente após o bip
    pinMode(BUZZER_PIN, INPUT);
}

void doubleBeep() {
    beep(40);
    delay(50);
    beep(40);
}
