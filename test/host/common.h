#pragma once
// Outils communs aux tests : boucle ECU, generateurs de signaux, profil de mouvement moto.
#include "host.h"
#include "mock/Arduino.h"
#include <memory>

#include "src/core/Config.h"
#include "src/core/SharedState.h"
#include "src/sensors/Sensors.h"
#include "src/actuator/IgnitionCut.h"
#include "src/control/Arbiter.h"
#include "src/persistence/License.h"

// ---- boucle ECU identique a loop() de shifter.ino (sans reseau) ------------------------
inline void ecuStep() {
  sensors::update();
  ignition::service();
  arbiter::update();
  ignition::service();
  state::beatEngineLoop();
}
// periodUs : duree d'un tour de loop() (reel : quelques dizaines de us).
inline void startEcuLoop(uint64_t periodUs = 100) {
  host::every(host::nowUs, periodUs, [] { ecuStep(); return true; });
}

// ---- generateurs d'impulsions ----------------------------------------------------------
struct Gen { bool stop = false; uint32_t count = 0; };
// Train d'impulsions : hz(t) frequence (<=0 : pause), jitter relatif gaussien, miss = proba d'impulsion manquee.
inline std::shared_ptr<Gen> train(int pin, std::function<double(uint64_t)> hz, uint32_t widthUs = 100,
                                  double jitter = 0.0, double miss = 0.0, uint64_t start = 0) {
  auto g = std::make_shared<Gen>();
  auto step = std::make_shared<std::function<void()>>();
  *step = [=]() {
    if (g->stop) return;
    const double f = hz(host::nowUs);
    if (f <= 0.0) { host::at(host::nowUs + 1000, *step); return; }
    if (miss <= 0.0 || host::rnd() >= miss) { host::pulse(pin, host::nowUs + 1, widthUs); g->count++; }
    double per = 1e6 / f; if (jitter > 0) per *= 1.0 + jitter * host::gauss();
    if (per < widthUs + 2) per = widthUs + 2;
    host::at(host::nowUs + (uint64_t)per, *step);
  };
  host::at(start ? start : host::nowUs, *step);
  return g;
}
// vitesse roue (km/h) -> frequence capteur (Hz) pour circonference/ppr donnes
inline double wheelHz(double kmh, double circMm, int ppr) { return (kmh / 3.6) / (circMm / 1000.0 / ppr); }
inline double rpmHz(double rpm) { return rpm / 60.0; }

// Active le tier voulu avec une vraie cle (HMAC) calculee par le firmware.
inline void setTier(LicenseTier t) {
  uint32_t retry = 0;
  if (t == LicenseTier::Basic) return;
  bool ok = license::tryUnlock(license::expectedKey(t), retry);
  if (!ok) host::fail(__FILE__, __LINE__, "setTier: cle refusee");
}

// Entrees TOR
inline void pressShifter(uint64_t t, uint32_t holdUs = 80000) {
  host::at(t, [] { host::setPin(pins::SHIFTER, 0); });
  host::at(t + holdUs, [] { host::setPin(pins::SHIFTER, 1); });
}
inline void setClutch(uint64_t t, bool pulled) { host::at(t, [=] { host::setPin(pins::CLUTCH_HALL, pulled ? 0 : 1); }); }
inline void setPit(uint64_t t, bool pressed)   { host::at(t, [=] { host::setPin(pins::PIT_BTN, pressed ? 0 : 1); }); }

#include "common_ride.h"
