#include "config.h"
#include "wifi_api.h"
#include <WiFi.h>

#define LED_PIN 2

extern bool displayInit();
extern bool inputInit();
extern bool nrf24Init();
extern bool cc1101Init();
extern void menuInit();
extern void menuLoop();
extern void showLoading(const char* text, int percent);
extern void startAPIServer();
extern void apiLoop();

// Buzzer
extern void buzzerInit();
extern void beep(int durationMs);
extern void doubleBeep();

extern bool deauthActive;
extern bool deauthLoop();

bool nrf24OK = false;
bool cc1101OK = false;
bool displayOK = false;

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial.begin(115200);
    Serial.setDebugOutput(true);
    delay(3000);

    Serial.println("\n=============================");
    Serial.println("[SETUP] Iniciando Crazy Cat...");
    Serial.println("=============================");
    Serial.flush();

    buzzerInit();
    beep(100);

    // 1. Display
    Serial.println("[SETUP] 1. Display...");
    Serial.flush();
    displayOK = displayInit();
    if (displayOK) {
        Serial.println("[SETUP] OK: Display.");
        showLoading("Iniciando...", 10);
    } else {
        Serial.println("[SETUP] ERRO: Display falhou!");
    }

    // 2. Botoes
    Serial.println("[SETUP] 2. Botoes...");
    Serial.flush();
    inputInit();
    if (displayOK) showLoading("Botoes OK", 25);

    // 3. NRF24
    Serial.println("[SETUP] 3. NRF24...");
    Serial.flush();
    nrf24OK = nrf24Init();
    Serial.printf("[SETUP] NRF24: %s\n", nrf24OK ? "OK" : "FAIL");
    Serial.flush();
    if (displayOK) showLoading(nrf24OK ? "NRF24 OK" : "NRF24 FAIL", 40);

    delay(100);

    // 4. CC1101
    Serial.println("[SETUP] 4. CC1101...");
    Serial.flush();
    cc1101OK = cc1101Init();
    Serial.printf("[SETUP] CC1101: %s\n", cc1101OK ? "OK" : "FAIL");
    Serial.flush();
    if (displayOK) showLoading(cc1101OK ? "CC1101 OK" : "CC1101 FAIL", 60);

    // 5. WiFi
    Serial.println("[SETUP] 5. WiFi...");
    Serial.flush();
    WiFi.mode(WIFI_AP_STA);
    delay(100);
    WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
    delay(50);
    WiFi.softAP("TP_Link", "crazycat123", 6, 0, 4);
    Serial.println("[SETUP] OK: WiFi AP.");
    if (displayOK) showLoading("WiFi OK", 80);

    // 6. API Server
    Serial.println("[SETUP] 6. API Server...");
    Serial.flush();
    startAPIServer();
    Serial.println("[SETUP] OK: API :8080.");
    if (displayOK) showLoading("Pronto!", 100);

    // 7. Menu
    Serial.println("[SETUP] 7. Menu...");
    Serial.flush();
    if (displayOK) {
        menuInit();
    }
    
    // BLE NAO e iniciado aqui - e iniciado sob demanda
    // quando o usuario entra no menu Bluetooth
    // Isso evita crash por conflito WiFi+BLE no boot
    Serial.println("[SETUP] BLE: lazy init (sob demanda)");
    
    Serial.println("[SETUP] Setup completo!");
    digitalWrite(LED_PIN, HIGH);
    doubleBeep();
}

void loop() {
    apiLoop();
    if (deauthActive) deauthLoop();
    if (displayOK) {
        menuLoop();
    } else {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(200);
    }
}
