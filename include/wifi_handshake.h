#ifndef WIFI_HANDSHAKE_H
#define WIFI_HANDSHAKE_H

#include <Arduino.h>

void startHandshakeCapture();
void stopHandshakeCapture();
bool isHandshakeCapturing();
const char* getHandshakeStatus();
uint8_t getHandshakeMessageCount();
bool isHandshakeComplete();
void clearHandshakeBuffer();

// Nova função usando buffer binário para não corromper o arquivo PCAP
uint8_t* getPcapData(size_t* outLen);

#endif
