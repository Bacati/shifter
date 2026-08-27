#include "Network.h"
#include "../core/Config.h"
#include "../persistence/Storage.h"
#include "../sim/SimEngine.h"
#include <WiFi.h>
#include <string.h>

#if __has_include(<esp_random.h>)
  #include <esp_random.h>
#else
  #include <esp_system.h>
#endif

namespace {
String   s_ssid;
String   s_apPw;
String   s_hwid;
volatile bool s_reboot = false;
uint32_t s_rebootAt = 0;

String genPassword(int n) {
  // Jeu de caracteres sans ambiguite visuelle (pas de 0/O/1/l/I).
  static const char CS[] = "ABCDEFGHJKLMNPQRSTUVWXYZabcdefghijkmnpqrstuvwxyz23456789";
  const int L = sizeof(CS) - 1;
  String s;
  s.reserve(n);
  for (int i = 0; i < n; ++i) s += CS[esp_random() % L];
  return s;
}
}  // namespace

void net::begin() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);

  s_hwid = WiFi.macAddress();
  String sfx = s_hwid;
  sfx.replace(":", "");
  s_ssid = "AptatioECU-" + sfx.substring(6);  // 6 derniers octets hex

  s_apPw = sim::apPasswordOverride();          // fixe en mode simulation, "" sinon
  if (s_apPw.length() < 8) {
    s_apPw = storage::loadWifiPassword();
    if (s_apPw.length() < 8) {
      s_apPw = genPassword(12);
      storage::saveWifiPassword(s_apPw);
      Serial.println("[NET] mot de passe AP genere (1er boot)");
    }
  }

  const bool ok = WiFi.softAP(s_ssid.c_str(), s_apPw.c_str(),
                              cfg::AP_CHANNEL, 0 /*ssid visible*/, cfg::AP_MAX_CLIENTS);
  WiFi.setSleep(false);

  Serial.println("=================================================");
  Serial.printf ("  WiFi SSID : %s\n", s_ssid.c_str());
  Serial.printf ("  WiFi PASS : %s\n", s_apPw.c_str());
  Serial.printf ("  IP        : %s\n", WiFi.softAPIP().toString().c_str());
  Serial.printf ("  AP init   : %s\n", ok ? "OK" : "ECHEC");
  Serial.println("=================================================");
}

String net::ssid()       { return s_ssid; }
String net::apPassword() { return s_apPw; }
String net::hwid()       { return s_hwid; }

void net::rotateApPassword(String& out) {
  out = genPassword(12);
  storage::saveWifiPassword(out);
  Serial.println("[NET] mot de passe AP renouvele (reboot en attente)");
}

void net::requestReboot(uint32_t delayMs) {
  s_rebootAt = millis() + delayMs;
  s_reboot   = true;
}

bool net::rebootPending() { return s_reboot; }

void net::serviceReboot() {
  if (s_reboot && static_cast<int32_t>(millis() - s_rebootAt) >= 0) {
    Serial.println("[NET] redemarrage...");
    Serial.flush();
    delay(20);
    ESP.restart();
  }
}
