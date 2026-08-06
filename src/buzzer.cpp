#include "config.h"

// ============================================================
// BUZZER - PWM hardware (muito mais alto que digitalWrite)
// Usa LEDC do ESP32 para gerar onda quadrada estavel
// ============================================================

#define BUZZER_PWM_CHANNEL  7       // canal LEDC (0-15, 7 evita conflito com display)
#define BUZZER_PWM_FREQ     2730    // 2730 Hz = frequencia de ressonancia do piezo
#define BUZZER_PWM_RES      8       // 8 bits = 0-255

static bool buzzerReady = false;

void buzzerInit() {
    // Configura PWM hardware no pino do buzzer
    ledcAttachPin(BUZZER_PIN, BUZZER_PWM_CHANNEL);
    ledcSetup(BUZZER_PWM_CHANNEL, BUZZER_PWM_FREQ, BUZZER_PWM_RES);
    ledcWrite(BUZZER_PWM_CHANNEL, 0);  // comeca desligado
    buzzerReady = true;
}

void beep(int durationMs = 50) {
    if (!buzzerReady) {
        buzzerInit();
    }
    // 50% duty cycle = volume MAXIMO do piezo
    ledcWrite(BUZZER_PWM_CHANNEL, 128);
    delay(durationMs);
    ledcWrite(BUZZER_PWM_CHANNEL, 0);
    // Pequeno delay apos o bip para evitar ruido de transicao
    delayMicroseconds(200);
}

void doubleBeep() {
    beep(60);
    delay(60);
    beep(60);
}
