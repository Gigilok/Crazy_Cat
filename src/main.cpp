#include "config.h"
#include "wifi_api.h"
#include <WiFi.h>
#include <SPI.h>

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
    // PROTEÇÃO CRÍTICA: GPIO15 (CC1101_CSN) é strapping pin.
    // Configurar como OUTPUT HIGH ANTES de qualquer coisa para garantir boot estável.
    pinMode(15, OUTPUT); digitalWrite(15, HIGH);
    
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

    // 3. Inicializa o barramento SPI compartilhado (VSPI) para NRF24 e CC1101
    Serial.println("[SETUP] Inicializando SPI compartilhado (VSPI)...");
    SPI.begin(NRF_SCK, NRF_MISO, NRF_MOSI, -1);  // CS controlado manualmente
    SPI.setFrequency(8000000);
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    delay(10);
    Serial.println("[SETUP] SPI VSPI OK.");

    // 4. NRF24
    Serial.println("[SETUP] 4. NRF24...");
    Serial.flush();
    nrf24OK = nrf24Init();
    Serial.printf("[SETUP] NRF24: %s\n", nrf24OK ? "OK" : "FAIL");
    Serial.flush();
    if (displayOK) showLoading(nrf24OK ? "NRF24 OK" : "NRF24 FAIL", 40);

    delay(100);

    // 5. CC1101
    // IMPORTANTE: o CC1101 e o NRF24 compartilham o mesmo barramento SPI
    // (VSPI: SCK=18, MOSI=23, MISO=19). O driver do CC1101 agora usa
    // SPI.beginTransaction()/endTransaction() em cada acesso e sobe o CSN
    // do NRF24 antes de tocar no barramento, evitando o conflito que
    // fazia o CC1101 retornar PARTNUM=0xFF ("modulo nao responde").
    Serial.println("[SETUP] 5. CC1101...");
    Serial.flush();
    cc1101OK = cc1101Init();
    Serial.printf("[SETUP] CC1101: %s\n", cc1101OK ? "OK" : "FAIL");
    Serial.flush();
    if (displayOK) showLoading(cc1101OK ? "CC1101 OK" : "CC1101 FAIL", 60);

    // 5b. Re-inicializa o NRF24 apos o CC1101 ter mexido nos pinos CE/CSN
    // do NRF24 (durante nrf24_release_bus). Isso garante que o NRF24
    // continua funcional apos a inicializacao do CC1101.
    if (nrf24OK) {
        Serial.println("[SETUP] 5b. Re-init NRF24 (apos CC1101)...");
        Serial.flush();
        nrf24OK = nrf24Init();
        Serial.printf("[SETUP] NRF24 re-init: %s\n", nrf24OK ? "OK" : "FAIL");
        Serial.flush();
    }

    // 6. WiFi
    Serial.println("[SETUP] 6. WiFi...");
    Serial.flush();
    WiFi.mode(WIFI_AP_STA);
    delay(100);
    WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
    delay(50);
    WiFi.softAP("TP_Link", "crazycat123", 6, 0, 4);
    Serial.println("[SETUP] OK: WiFi AP.");
    if (displayOK) showLoading("WiFi OK", 80);

    // 7. API Server
    Serial.println("[SETUP] 7. API Server...");
    Serial.flush();
    startAPIServer();
    Serial.println("[SETUP] OK: API :8080.");
    if (displayOK) showLoading("Pronto!", 100);

    // 8. Menu
    Serial.println("[SETUP] 8. Menu...");
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
