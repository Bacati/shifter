#include "IgnitionCut.h"
#include "../core/Config.h"
#include "../core/SharedState.h"
#include "../sim/SimEngine.h"   // IGNITION_SIM_SCALE
#include <soc/gpio_struct.h>

namespace {
portMUX_TYPE      s_mux       = portMUX_INITIALIZER_UNLOCKED;
volatile bool     s_active    = false;
volatile uint32_t s_endMicros = 0;
volatile uint8_t  s_mode      = static_cast<uint8_t>(AssistMode::Idle);

inline void cutOn()  { GPIO.out_w1ts = (1u << pins::CUT); }
inline void cutOff() { GPIO.out_w1tc = (1u << pins::CUT); }
}  // namespace

void ignition::begin() {
  pinMode(pins::CUT, OUTPUT);
  cutOff();
  portENTER_CRITICAL(&s_mux);
  s_active = false;
  s_mode   = static_cast<uint8_t>(AssistMode::Idle);
  portEXIT_CRITICAL(&s_mux);
}

void ignition::request(uint16_t durationMs, AssistMode mode) {
#ifdef WOKWI_SIM
  const uint32_t scaledMs = static_cast<uint32_t>(durationMs) * sim::timeScale();
#else
  const uint32_t scaledMs = durationMs;   // build normal : aucune mise a l'echelle
#endif
  const uint32_t proposedEnd = micros() + scaledMs * 1000UL;
  const uint8_t  m           = static_cast<uint8_t>(mode);
  bool notify = false;

  portENTER_CRITICAL(&s_mux);
  const bool wasActive = s_active;
  if (!wasActive || m >= s_mode) {
    // prolonge seulement (ne raccourcit jamais une coupure deja programmee)
    if (!wasActive || static_cast<int32_t>(proposedEnd - s_endMicros) > 0)
      s_endMicros = proposedEnd;
    if (!wasActive || s_mode != m) notify = true;
    s_mode   = m;
    s_active = true;
    cutOn();
  }
  portEXIT_CRITICAL(&s_mux);

  if (notify) state::setMode(mode);
}

void ignition::service() {
  bool cleared = false;
  portENTER_CRITICAL(&s_mux);
  if (s_active && static_cast<int32_t>(micros() - s_endMicros) >= 0) {
    cutOff();
    s_active = false;
    s_mode   = static_cast<uint8_t>(AssistMode::Idle);
    cleared  = true;
  }
  portEXIT_CRITICAL(&s_mux);

  if (cleared) state::setMode(AssistMode::Idle);
}

bool ignition::isActive() {
  portENTER_CRITICAL(&s_mux);
  const bool a = s_active;
  portEXIT_CRITICAL(&s_mux);
  return a;
}

AssistMode ignition::activeMode() {
  portENTER_CRITICAL(&s_mux);
  const uint8_t m = s_mode;
  portEXIT_CRITICAL(&s_mux);
  return static_cast<AssistMode>(m);
}

void ignition::forceClear() {
  portENTER_CRITICAL(&s_mux);
  cutOff();
  s_active = false;
  s_mode   = static_cast<uint8_t>(AssistMode::Idle);
  portEXIT_CRITICAL(&s_mux);
  state::setMode(AssistMode::Idle);
}
