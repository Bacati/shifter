#pragma once
#include <cstdint>
#include "host.h"
inline void esp_restart() { host::restartCount++; host::restartAtUs = host::nowUs; throw host::RestartException{}; }
inline uint32_t esp_random() { return (uint32_t)(host::rnd() * 4294967296.0); }
