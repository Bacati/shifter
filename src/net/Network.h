#pragma once
#include <Arduino.h>

// ============================================================================
//  Network : point d'acces WiFi durci + latch de redemarrage.
//
//   - WPA2, SSID suffixe par la MAC, canal fixe, 2 clients max.
//   - Mot de passe PAR APPAREIL : genere aleatoirement au 1er boot,
//     stocke en NVS, imprime sur le port serie. Rotatif via /api/wifi.
//   - Le reboot demande par un handler async est differe puis execute
//     depuis loop() (jamais xTaskCreate depuis le contexte AsyncTCP).
// ============================================================================

namespace net {

void   begin();
String ssid();
String apPassword();
String hwid();

void rotateApPassword(String& newPasswordOut);  // genere + persiste (reboot requis)

void requestReboot(uint32_t delayMs = 400);
bool rebootPending();
void serviceReboot();  // appele par loop()

}  // namespace net
