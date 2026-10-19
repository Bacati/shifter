#pragma once
#include "OneWire.h"
typedef uint8_t DeviceAddress[8];
#define DEVICE_DISCONNECTED_C (-127.0f)

// Faux DS18B20 : modele fidele aux pieges reels (85.0 C d'usine tant qu'aucune conversion
// n'est terminee, requestTemperatures() bloquant par defaut, conversion 12 bits = 750 ms).
class DallasTemperature {
 public:
  explicit DallasTemperature(OneWire*) {}
  void begin() {}
  uint8_t getDeviceCount() { return host::ds.present ? 1 : 0; }
  bool getAddress(uint8_t* a, uint8_t idx) {
    if (!host::ds.present || idx != 0) return false;
    for (int i = 0; i < 8; ++i) a[i] = (uint8_t)(0x28 + i);
    return true;
  }
  bool setResolution(const uint8_t*, uint8_t, bool = false) { return host::ds.present; }
  void setResolution(uint8_t) {}
  void setWaitForConversion(bool w) { host::ds.waitConv = w; }
  void requestTemperatures();
  float getTempC(const uint8_t*, byte = 0);
  float getTempCByIndex(uint8_t idx) { uint8_t a[8]; if (!getAddress(a, idx)) return DEVICE_DISCONNECTED_C; return getTempC(a); }
};
