#pragma once
#include <cstdint>
#include "../../host.h"

// GPIO.out_w1ts / out_w1tc : l'ecriture d'un masque pilote les broches de sortie.
struct GpioW1 {
  bool set;
  GpioW1& operator=(uint32_t mask) {
    for (int p = 0; p < 32; ++p) if (mask & (1u << p)) host::driveOut(p, set ? 1 : 0);
    return *this;
  }
};
struct gpio_dev_t {
  GpioW1 out_w1ts{true};
  GpioW1 out_w1tc{false};
};
extern gpio_dev_t GPIO;
