#pragma once
#include "Arduino.h"
#define WIFI_AP 2
struct IPAddress { String toString() const { return "192.168.4.1"; } };
class WiFiClass {
 public:
  String lastSsid, lastPass; bool apUp = false;
  void persistent(bool) {}
  bool mode(int) { return true; }
  String macAddress() { return String(host::wifiMac); }
  bool softAP(const char* ssid, const char* pass, int, int, int) {
    lastSsid = ssid; lastPass = pass; apUp = strlen(pass) >= 8; return apUp;
  }
  IPAddress softAPIP() { return IPAddress(); }
  void setSleep(bool) {}
};
extern WiFiClass WiFi;
