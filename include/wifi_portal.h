#ifndef WIFI_PORTAL_H
#define WIFI_PORTAL_H

#include <Arduino.h>

void startPortal(const char* ssid);
void stopPortal();
void portalLoop();
bool isPortalActive();
const char* getCapturedPassword();
const char* getPortalStatus();

#endif
