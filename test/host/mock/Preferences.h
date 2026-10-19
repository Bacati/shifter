#pragma once
#include "Arduino.h"

// Faux NVS (host::nvs survit aux "reboots" d'un test ; coupure de courant simulable).
class Preferences {
  std::string ns_; bool ro_ = false, started_ = false;
  std::string key(const char* k) const { return ns_ + "/" + k; }
  void put(const char* k, const std::string& v) {
    auto it = host::nvs.find(key(k));
    if (it != host::nvs.end() && it->second == v) return;     // inchange : pas d'ecriture flash
    if (host::nvsFailAfter >= 0 && host::nvsWrites >= host::nvsFailAfter) throw host::PowerLoss{};
    host::nvsWrites++;
    host::nvs[key(k)] = v;
  }
 public:
  bool begin(const char* name, bool readOnly = false) {
    ns_ = name; ro_ = readOnly;
    const bool exists = host::nvs.count(ns_ + "/#ns") != 0;
    if (ro_ && !exists) { started_ = false; return false; }
    if (!ro_ && !exists) host::nvs[ns_ + "/#ns"] = "1";
    started_ = true; return true;
  }
  void end() { started_ = false; }
  int getInt(const char* k, int d = 0) {
    if (!started_) return d;
    auto it = host::nvs.find(key(k)); return it == host::nvs.end() ? d : atoi(it->second.c_str());
  }
  size_t putInt(const char* k, int v) { if (!started_ || ro_) return 0; put(k, std::to_string(v)); return 4; }
  String getString(const char* k, String d = String()) {
    if (!started_) return d;
    auto it = host::nvs.find(key(k)); return it == host::nvs.end() ? d : String(it->second);
  }
  size_t putString(const char* k, const String& v) { if (!started_ || ro_) return 0; put(k, v.c_str()); return v.length(); }
  size_t putBytes(const char* k, const void* v, size_t n) {
    if (!started_ || ro_) return 0;
    std::string h; static const char* x = "0123456789abcdef";
    for (size_t i = 0; i < n; ++i) { h += x[((const uint8_t*)v)[i] >> 4]; h += x[((const uint8_t*)v)[i] & 15]; }
    put((std::string("#b/") + k).c_str(), h); return n;
  }
  size_t getBytesLength(const char* k) {
    if (!started_) return 0; auto it = host::nvs.find(key((std::string("#b/") + k).c_str()));
    return it == host::nvs.end() ? 0 : it->second.size() / 2;
  }
  size_t getBytes(const char* k, void* buf, size_t max) {
    size_t n = getBytesLength(k); if (!n || n > max) return 0;
    const std::string& h = host::nvs[key((std::string("#b/") + k).c_str())];
    for (size_t i = 0; i < n; ++i) ((uint8_t*)buf)[i] = (uint8_t)std::stoi(h.substr(2 * i, 2), nullptr, 16);
    return n;
  }
};
