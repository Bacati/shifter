#pragma once
#include <Arduino.h>

// ============================================================================
//  Types partages : modes de l'arbitre, tiers de licence, config carto.
// ============================================================================

// L'ordre = la priorite stricte de l'arbitre. Valeur haute = prioritaire.
// Overrev (survie moteur) > Pit limiter (voie des stands : prend le dessus sur toutes les
// assistances) > Quickshifter > Launch > Slip.
enum class AssistMode : uint8_t {
  Idle     = 0,
  Slip     = 1,
  Launch   = 2,
  Shift    = 3,
  PitLimit = 4,
  Overrev  = 5,
};
static constexpr uint8_t kAssistModeCount = 6;

enum class LicenseTier : uint8_t {
  Basic = 0,
  Track = 1,
  Pro   = 2,
};
static constexpr uint8_t kLicenseTierCount = 3;

inline constexpr bool operator>=(LicenseTier a, LicenseTier b) {
  return static_cast<uint8_t>(a) >= static_cast<uint8_t>(b);
}
inline constexpr bool operator<(LicenseTier a, LicenseTier b) {
  return static_cast<uint8_t>(a) < static_cast<uint8_t>(b);
}

// Cartographie modifiable a chaud (web) et persistee en NVS.
struct EcuConfig {
  int shiftCutMs;      // duree de coupure quickshifter
  int minShiftRpm;     // regime mini pour armer le quickshifter
  int launchRpm;       // regime cible du launch control
  int maxSlipPercent;  // tolerance de glisse du slip control
  int wheelCircumMm;   // circonference de roue (capteurs de vitesse)
  int pulsesPerRev;    // impulsions par tour des capteurs de roue
  int pitRpm;          // plafond regime en voie des stands (repli si les capteurs de roue sont muets)
  int pitKmh;          // vitesse maxi en voie des stands (km/h) : limiteur principal
};

inline const char* toString(AssistMode m) {
  switch (m) {
    case AssistMode::Idle:     return "IDLE";
    case AssistMode::PitLimit: return "PIT LIMIT";
    case AssistMode::Launch:   return "LAUNCH";
    case AssistMode::Slip:     return "SLIP CTL";
    case AssistMode::Shift:    return "SHIFTING";
    case AssistMode::Overrev:  return "OVERREV";
  }
  return "IDLE";
}

inline const char* toString(LicenseTier t) {
  switch (t) {
    case LicenseTier::Basic: return "BASIC";
    case LicenseTier::Track: return "TRACK";
    case LicenseTier::Pro:   return "PRO (MotoGP)";
  }
  return "BASIC";
}
