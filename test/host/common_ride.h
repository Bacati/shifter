#pragma once
#include "host.h"
#include "mock/Arduino.h"
#include <functional>
#include <memory>

// ---- profil de mouvement moto (pour le faux MPU6050) --------------------------------------
//  Repere corps : X avant, Y gauche, Z haut. roll>0 = couche a DROITE. Lacet monde psi' = -g tan(phi)/v.
struct Ride {
  std::function<double(uint64_t)> rollDeg;     // angle reel
  std::function<double(uint64_t)> speedMs;     // vitesse (m/s) pour le lacet
  std::function<double(uint64_t)> longG;       // accel longitudinale (g)
};
inline void installRide(const Ride& r) {
  // Vitesse de roulis = derivee ARRIERE entre deux appels successifs (le MPU est echantillonne a 200 Hz) :
  // coherente meme si l'angle depend de l'etat de la moto (ex. amplitude fonction de la vitesse).
  auto last = std::make_shared<std::pair<uint64_t, double>>(std::make_pair<uint64_t, double>(~0ull, 0.0));
  host::mpu.motion = [r, last](uint64_t t, float* a, float* g) {
    const double phi = r.rollDeg(t) * M_PI / 180.0;
    double phid = 0.0;
    if (last->first != ~0ull && t > last->first) phid = (phi - last->second) / ((t - last->first) * 1e-6);
    last->first = t; last->second = phi;
    const double v = std::max(1.0, r.speedMs(t));
    const double psid = -9.81 * std::tan(phi) / v;              // lacet monde (rad/s)
    a[0] = (float)r.longG(t);
    a[1] = 0.0f;                                                // virage stabilise : Y corps ~ 0
    a[2] = (float)(1.0 / std::cos(phi));
    g[0] = (float)(phid * 180.0 / M_PI);
    g[1] = (float)(psid * std::sin(phi) * 180.0 / M_PI);
    g[2] = (float)(psid * std::cos(phi) * 180.0 / M_PI);
  };
}
