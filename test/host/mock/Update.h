#pragma once
#include "Arduino.h"
#define UPDATE_SIZE_UNKNOWN 0xFFFFFFFFu
class UpdateClass {
 public:
  bool running = false, error = false, ended = false; size_t written = 0; int begins = 0;
  uint8_t first = 0;
  bool begin(size_t) { running = true; error = false; ended = false; written = 0; ++begins; first = 0; return true; }
  size_t write(uint8_t* d, size_t n) {
    if (!running) return 0;
    if (written == 0 && n > 0) { first = d[0]; if (d[0] != 0xE9) { error = true; return 0; } }   // octet magique ESP32
    written += n; return n;
  }
  bool end(bool) { ended = true; running = false; return !error && written > 0; }
  bool hasError() const { return error; }
  void printError(SerialMock&) {}
};
extern UpdateClass Update;
