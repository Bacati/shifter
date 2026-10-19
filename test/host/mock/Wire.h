#pragma once
#include "Arduino.h"

// Faux bus I2C relie au MPU6050 simule (host::mpu).
class TwoWire {
  uint8_t  addr_ = 0;
  uint8_t  tx_[16]; size_t txn_ = 0;
  uint8_t  rx_[32]; size_t rxn_ = 0, rxi_ = 0;
 public:
  bool begin(int sda = -1, int scl = -1, uint32_t hz = 0);
  void beginTransmission(uint8_t a) { addr_ = a; txn_ = 0; }
  size_t write(uint8_t b) { if (txn_ < sizeof(tx_)) tx_[txn_++] = b; return 1; }
  uint8_t endTransmission(bool stop = true);
  size_t requestFrom(int addr, int n);
  int read() { return rxi_ < rxn_ ? rx_[rxi_++] : -1; }
};
extern TwoWire Wire;
