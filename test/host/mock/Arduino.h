#pragma once
// Faux Arduino.h pour compiler le vrai code de src/ sur PC (voir test/host/host.h).
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdarg>
#include <cmath>
#include <string>
#include <algorithm>
#include <atomic>
#include "../host.h"
#include "freertos/FreeRTOS.h"

#define IRAM_ATTR
#define PROGMEM
#define strlen_P strlen
#define DRAM_ATTR

typedef uint8_t byte;

#define HIGH 1
#define LOW  0
#define INPUT          0
#define OUTPUT         1
#define INPUT_PULLUP   2
#define RISING   1
#define FALLING  2
#define CHANGE   3

#ifndef constrain
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif

uint32_t millis();
uint32_t micros();
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);
void pinMode(int pin, int mode);
void digitalWrite(int pin, int level);
int  digitalRead(int pin);
void attachInterrupt(int pin, void (*fn)(), int mode);
inline int digitalPinToInterrupt(int pin) { return pin; }

// ---- String (sous-ensemble utilise par le firmware) -------------------------
class String {
  std::string s;
 public:
  String() {}
  String(const char* c) : s(c ? c : "") {}
  String(const std::string& x) : s(x) {}
  String(char c) : s(1, c) {}
  String(int v) : s(std::to_string(v)) {}
  String(unsigned v) : s(std::to_string(v)) {}
  String(long v) : s(std::to_string(v)) {}
  String(unsigned long v) : s(std::to_string(v)) {}
  size_t length() const { return s.size(); }
  const char* c_str() const { return s.c_str(); }
  char operator[](size_t i) const { return s[i]; }
  void reserve(size_t n) { s.reserve(n); }
  String& operator+=(char c) { s += c; return *this; }
  String& operator+=(const char* c) { s += c; return *this; }
  String& operator+=(const String& o) { s += o.s; return *this; }
  friend String operator+(const String& a, const String& b) { return String(a.s + b.s); }
  friend String operator+(const String& a, const char* b) { return String(a.s + b); }
  friend String operator+(const char* a, const String& b) { return String(std::string(a) + b.s); }
  bool operator==(const char* o) const { return s == o; }
  bool operator==(const String& o) const { return s == o.s; }
  bool operator!=(const char* o) const { return s != o; }
  bool operator!=(const String& o) const { return s != o.s; }
  int indexOf(char c) const { size_t p = s.find(c); return p == std::string::npos ? -1 : (int)p; }
  String substring(size_t from) const { return String(from >= s.size() ? std::string() : s.substr(from)); }
  String substring(size_t from, size_t to) const { return String(from >= s.size() ? std::string() : s.substr(from, to - from)); }
  long toInt() const { return strtol(s.c_str(), nullptr, 10); }
  void trim() {
    size_t a = s.find_first_not_of(" \t\r\n"); if (a == std::string::npos) { s.clear(); return; }
    size_t b = s.find_last_not_of(" \t\r\n"); s = s.substr(a, b - a + 1);
  }
  void toUpperCase() { for (auto& c : s) c = (char)toupper((unsigned char)c); }
  bool startsWith(const char* p) const { return s.rfind(p, 0) == 0; }
  void replace(const char* from, const char* to) {
    std::string f = from, t = to; size_t p = 0;
    while ((p = s.find(f, p)) != std::string::npos) { s.replace(p, f.size(), t); p += t.size(); }
  }
};

// ---- Serial ----------------------------------------------------------------
class SerialMock {
 public:
  void begin(unsigned long) {}
  void flush() {}
  int  available();
  int  read();
  void write(const std::string& out);
  void printf(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
  void println() { write("\n"); }
  void println(const char* s) { write(std::string(s) + "\n"); }
  void println(const String& s) { write(std::string(s.c_str()) + "\n"); }
  void print(const char* s) { write(s); }
};
extern SerialMock Serial;

struct EspClass { void restart(); };
extern EspClass ESP;
