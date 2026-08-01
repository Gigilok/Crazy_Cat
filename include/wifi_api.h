#ifndef WIFI_API_H
#define WIFI_API_H

#include <Arduino.h>

void startAPIServer();
void stopAPIServer();
void apiLoop();
bool isAPIServerRunning();

#endif
