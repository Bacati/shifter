#pragma once
#include <Arduino.h>

// ============================================================================
//  Sensors : possede les ISR et leurs variables brutes.
//
//   - RPM      : 1 impulsion / tour. Filtre EMI 2 etages + EWMA + peak instant.
//   - Roues    : comptage d'impulsions (anti-rebond optionnel).
//   - Shifter  : latch d'evenement avec horodatage (fenetre analysee par l'arbitre).
//   - Pit / embrayage : entrees digitales debouncees.
//
//   Toute la synchro ISR <-> tache passe par un spinlock dedie (pris aussi
//   dans les ISR via portENTER_CRITICAL_ISR).
// ============================================================================

namespace sensors {

void begin();
void update();  // appele chaque tour de loop() (Core 1)

struct ShiftRequest {
  bool     pending;
  uint32_t triggeredMicros;
};

ShiftRequest peekShift();     // lit le latch sans le consommer
void         consumeShift();  // efface le latch (apres action OU expiration de fenetre)

bool pitButton();     // debounce
bool clutchPulled();  // debounce
bool engineStalled();

}  // namespace sensors
