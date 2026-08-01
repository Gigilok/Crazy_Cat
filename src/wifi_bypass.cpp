// ============================================================
// wifi_bypass.cpp
// BYPASS do ieee80211_raw_frame_sanity_check para Arduino-ESP32
// 
// NO ARDUINO-ESP32, o --wrap do linker NAO funciona em libs pre-compiladas.
// A solucao eh definir a funcao DIRETAMENTE e usar -zmuldefs no linker
// para permitir que esta definicao sobrescreva a da libnet80211.a.
//
// Assinatura para Arduino-ESP32 v2.x (ESP-IDF v4.4): 3 x int32_t
// ============================================================
#include <Arduino.h>
#include <stdbool.h>

// Funcao definida diretamente - sobrescreve a da libnet80211.a via -zmuldefs
extern "C" int ieee80211_raw_frame_sanity_check(int32_t arg1, int32_t arg2, int32_t arg3) {
    (void)arg1;
    (void)arg2;
    (void)arg3;
    return 0;
}
