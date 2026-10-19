#pragma once
// Moto virtuelle pour les tests de stress : boite 6 rapports, embrayage, couple moteur, traction/patinage,
// freinage, roue avant, + pilote qui enchaine departs, passages de vitesse, retrogradages, virages.
// Branchee sur le VRAI firmware (shifter.ino) via les broches, les impulsions RPM/roues et l'IMU simule.
#include "host.h"
#include "mock/Arduino.h"
#include "common.h"
#include <algorithm>
#include <deque>

struct Bike {
  // ---- commandes du pilote ----
  double thr = 0;               // 0..1
  double brake = 0;             // km/h/s de freinage
  int    gear = 0;              // 0 = point mort, 1..6
  double engage = 1.0;          // embrayage 0 (tire) .. 1 (relache)
  bool   engineOn = true;
  // ---- etat ----
  double rpm = 1500, vg = 0, ex = 0;     // regime, vitesse sol (km/h), survitesse de la roue arriere (patinage)
  double frontFactor = 1.0;              // < 1 : roue avant en l'air (wheelie)
  double mu = 1.0;                       // adherence
  // ---- statistiques ----
  double spinIntegral = 0;               // somme(ex*dt) : mesure du patinage
  double peakSpin = 0;
  double peakRpm = 0, maxRpmEver = 0, spinTotal = 0;
  long   stalls = 0;
  double aDriveLast = 0;

  static double ratio(int g) { static const double r[7] = {0, 40, 30, 24, 20, 17.5, 15.5}; return r[g < 0 ? 0 : (g > 6 ? 6 : g)]; }
  static double torque(double rpm) { double x = (rpm - 11500.0) / 8000.0; return std::max(0.0, 1.0 - x * x); }
  static constexpr double kRpmPerKmh = 9.259;       // roue de 1,8 m

  double lockRpm() const { return gear > 0 ? (vg + ex) * kRpmPerKmh * ratio(gear) : 0.0; }

  void step(double dt, bool cut) {
    const double R = ratio(gear);
    double aDrive = 0.0;
    const bool coupled = gear > 0 && engage > 0.02;
    if (engineOn) {
      if (!coupled) {
        double a = thr * 16000.0 * (1.0 - rpm / 22000.0) - 1500.0 * rpm / 10000.0;
        if (rpm < 1400.0 && rpm > 300.0) a += 800.0 * (1400.0 - rpm) / 1400.0 + 400.0;     // regulation de ralenti
        if (cut) a = -6000.0;
        rpm = std::max(0.0, rpm + a * dt);
      } else {
        const double lock = lockRpm();
        const double sl = rpm - lock;                          // glissement d'embrayage (tr/min)
        const double aFree = thr * 16000.0 * (1.0 - rpm / 22000.0) - 1500.0 * rpm / 10000.0;
        const double eb = 8.0 * R / 40.0 * (rpm / 10000.0);
        if (sl > 150.0) {                                      // l'embrayage patine : le moteur est plus vite que la roue
          const double aCap = engage * engage * 70.0 * R / 40.0;
          const double aTr = std::min(aCap, sl * 0.01 * R / 40.0);
          aDrive = cut ? -eb * 0.3 : aTr;
          double a = cut ? -6000.0 : aFree - aTr * kRpmPerKmh * R * 0.3;
          rpm = std::min(21000.0, std::max(lock, rpm + a * dt));
          if (engage > 0.4 && rpm < 1250.0 && thr < 0.5) { engineOn = false; ++stalls; rpm = 0; }   // cale : embrayage lache sans gaz
        } else if (sl < -150.0) {                              // la roue entraine un moteur plus lent (relache brutal, retro)
          aDrive = -std::min(engage * 25.0 * R / 40.0, -sl * 0.01 * R / 40.0);
          rpm += (lock - rpm) * std::min(1.0, dt / 0.05);
        } else {                                               // verrouille
          rpm = std::min(21000.0, lock + 0.0);
          double want = thr * std::max(0.12, torque(rpm)) * R;
          if (cut) want = -eb; else want -= eb * (1.0 - thr);
          aDrive = want * engage;
        }
        if (engage > 0.9 && lock < 900.0 && thr < 0.35 && rpm < 900.0) { engineOn = false; ++stalls; rpm = 0; }
      }
    } else {
      rpm = std::max(0.0, rpm - 3000.0 * dt);
      if (coupled && engage > 0.5) aDrive = -3.0 * R / 40.0;     // moteur cale, vitesse engagee : frein moteur
    }
    // ---- sol ----
    const double aTrac = mu * 35.0;
    double aGround = aDrive;
    if (aDrive > aTrac) { ex += (aDrive - aTrac) * dt * 3.0; aGround = aTrac; }
    else                { ex = std::max(0.0, ex - 60.0 * dt); }
    const double drag = 0.0006 * vg * vg + (vg > 0.1 ? 1.0 : 0.0);
    vg = std::max(0.0, vg + (aGround - drag - brake) * dt);
    if (vg <= 0.0) ex = std::min(ex, 40.0);
    spinIntegral += ex * dt;
    peakSpin = std::max(peakSpin, ex);
    peakRpm = std::max(peakRpm, rpm); maxRpmEver = std::max(maxRpmEver, rpm); spinTotal += ex * dt;
    aDriveLast = aDrive;
  }
  double front() const { return vg * frontFactor; }
  double rear() const { return vg + ex; }
};

// ------------------------------------------------------------------------------------------------------------
struct EdgeRec { uint64_t t; AssistMode m; };
struct PressRec { uint64_t t; uint32_t holdUs; int rpmF; bool cutActive; };

struct Stats {
  long cuts = 0, violations = 0;
  long byMode[6] = {0, 0, 0, 0, 0, 0};
  std::string firstViolation;
  std::vector<EdgeRec> edges;
  std::vector<PressRec> presses;
  uint64_t maxCutRun = 0, maxCutRunNoPit = 0;   // plus longues coupures continues (hors pit limiter et launch : ils coupent en continu par conception)
};

struct World {
  Bike B;
  Stats S;
  std::function<double(uint64_t)> leanFn;     // angle de roulis vrai (deg)
  bool checkInvariants = true;
  int  minShift = 6000;                       // reglage courant du regime mini shifter (pour les invariants)
  uint64_t rpmDropUntil = 0;                  // panne du capteur RPM jusqu'a cet instant
  double lastShiftEdge = -1;
  long unassistedShifts = 0;

  std::deque<std::string> hist;               // historique 100 ms (vrai angle / estime / vitesses)
  std::vector<std::string> snap;
  void violation(const std::string& m) { if (!S.violations++) { S.firstViolation = m; snap.assign(hist.begin(), hist.end()); } }
};

static World W;

static inline double leanAt(uint64_t t) { return W.leanFn ? W.leanFn(t) : 0.0; }

// Branche la moto sur le firmware : integration 1 ms, generateurs d'impulsions, IMU, surveillance des coupures.
static inline void wireBike() {
  host::every(host::nowUs, 1000, [] { W.B.step(0.001, host::cutHigh()); return true; });
  train(pins::RPM, [](uint64_t t) { return (W.B.engineOn && W.B.rpm > 400 && t >= W.rpmDropUntil) ? rpmHz(W.B.rpm) : 0.0; });
  train(pins::WHEEL_FRONT, [](uint64_t) { return W.B.front() > 0.5 ? wheelHz(W.B.front(), 1800, 3) : 0.0; });
  train(pins::WHEEL_REAR,  [](uint64_t) { return W.B.rear()  > 0.5 ? wheelHz(W.B.rear(),  1800, 3) : 0.0; });
  Ride r;
  r.speedMs = [](uint64_t) { return std::max(1.0, W.B.vg / 3.6); };
  r.longG   = [](uint64_t) { return 0.0; };
  r.rollDeg = [](uint64_t t) { return leanAt(t); };
  installRide(r);
}

// Echantillonne la broche CUT (200 us) : enregistre chaque debut de coupure et verifie les invariants.
static inline void watchCuts(bool pitLimitEnabledCheck = true) {
  static bool last = false;
  static uint64_t runStart = 0;
  static bool runPit = false;
  static AssistMode lastMode = AssistMode::Idle;
  last = false; lastMode = AssistMode::Idle;
  host::every(host::nowUs, 200, [pitLimitEnabledCheck] {
    const bool now = host::cutHigh();
    const AssistMode curMode = now ? ignition::activeMode() : AssistMode::Idle;
    if (now && !last) { runStart = host::nowUs; runPit = false; }
    if (now && (sensors::pitButton() || ignition::activeMode() == AssistMode::Launch)) runPit = true;   // coupures continues voulues : pit et 2-step
    if (now && (!last || curMode != lastMode)) {       // debut de coupure OU changement de mode en cours de coupure
      const AssistMode m = curMode;
      W.S.cuts++; W.S.byMode[(int)m]++;
      W.S.edges.push_back({host::nowUs, m});
      if (m == AssistMode::Shift) W.lastShiftEdge = host::nowUs / 1000.0;
      if (W.checkInvariants) {
        int roll, lat; bool imuOk; state::getLean(roll, lat, imuOk);
        const bool imuFresh = imuOk && (millis() - state::lastLeanMs() < cfg::IMU_STALE_MS);
        int f, rr; state::getSpeeds(f, rr);
        const double truth = leanAt(host::nowUs);
        char b[240];
        const double ts = host::nowUs / 1e6;
        if (m == AssistMode::Slip) {
          if (license::tier() < LicenseTier::Pro) { snprintf(b, sizeof b, "slip hors licence t=%.2f", ts); W.violation(b); }
          if (!imuFresh) { snprintf(b, sizeof b, "slip sans IMU fiable t=%.2f", ts); W.violation(b); }
          if (std::fabs(truth) > cfg::LEAN_CUTOFF_DEG + 7.0) { snprintf(b, sizeof b, "slip a %.1f deg vrai (estime %.1f) t=%.2f", truth, roll / 10.0, ts); W.violation(b); }
          if (lat > cfg::LEAN_LATERAL_G_HARD * 100 + 3) { snprintf(b, sizeof b, "slip a %.2f g t=%.2f", lat / 100.0, ts); W.violation(b); }
          if (f / 10.0 <= cfg::SLIP_MIN_SPEED_KMH) { snprintf(b, sizeof b, "slip a %.1f km/h t=%.2f", f / 10.0, ts); W.violation(b); }
        } else if (m == AssistMode::Launch) {
          if (license::tier() < LicenseTier::Track) { snprintf(b, sizeof b, "launch hors licence t=%.2f", ts); W.violation(b); }
          if (!sensors::clutchPulled()) { snprintf(b, sizeof b, "launch sans embrayage t=%.2f", ts); W.violation(b); }
          if (f / 10.0 >= cfg::LAUNCH_SPEED_MAX_KMH + 0.5 || rr / 10.0 >= cfg::LAUNCH_SPEED_MAX_KMH + 0.5) { snprintf(b, sizeof b, "launch a f=%.1f r=%.1f t=%.2f", f / 10.0, rr / 10.0, ts); W.violation(b); }
        } else if (m == AssistMode::Shift) {
          if (state::getRpmFiltered() < W.minShift - 400) { snprintf(b, sizeof b, "shift a %d tr/min t=%.2f", state::getRpmFiltered(), ts); W.violation(b); }
        } else if (m == AssistMode::Overrev) {
          if (state::getRpmInstant() < cfg::OVERREV_RPM - 300) { snprintf(b, sizeof b, "overrev a %d tr/min t=%.2f", state::getRpmInstant(), ts); W.violation(b); }
        } else if (m == AssistMode::PitLimit) {
          if (!sensors::pitButton()) { snprintf(b, sizeof b, "pit sans bouton t=%.2f", ts); W.violation(b); }
        } else { snprintf(b, sizeof b, "coupure en mode Idle t=%.2f", ts); W.violation(b); }
      }
    }
    if (!now && last) {
      W.S.maxCutRun = std::max<uint64_t>(W.S.maxCutRun, host::nowUs - runStart);
      if (!runPit) W.S.maxCutRunNoPit = std::max<uint64_t>(W.S.maxCutRunNoPit, host::nowUs - runStart);
    }
    last = now; lastMode = curMode;
    return true; });
}

// ---- pilote : actions elementaires ------------------------------------------------------------------------
static inline void pinClutch(bool pulled) { host::setPin(pins::CLUTCH_HALL, pulled ? 0 : 1); }

// Appui sur le selecteur enregistre pour la verification : l'etat du regime est lu a l'instant de l'appui.
static inline void riderPressShifter(uint64_t t, uint32_t holdUs = 60000) {
  host::at(t, [holdUs] {
    W.S.presses.push_back({host::nowUs, holdUs, state::getRpmFiltered(),
                            host::cutHigh() || sensors::pitButton() || state::getRpmInstant() > cfg::OVERREV_RPM - 800});
    host::setPin(pins::SHIFTER, 0);
  });
  host::at(t + holdUs, [] { host::setPin(pins::SHIFTER, 1); });
}

// Verifie la correspondance appuis <-> coupures shifter (fenetre de tolerance sur les cas limites).
static inline void verifyShiftAccounting(long* mustCutOut = nullptr, long* missedOut = nullptr, long* spuriousOut = nullptr) {
  long mustCut = 0, missed = 0, mustNot = 0, spurious = 0;
  double lastEdge = -1e9;
  size_t ei = 0;
  std::vector<uint64_t> shiftEdges;
  for (auto& e : W.S.edges) if (e.m == AssistMode::Shift) shiftEdges.push_back(e.t);
  uint64_t lastExpected = 0; bool haveExpected = false;
  for (auto& p : W.S.presses) {
    // dernier front de coupure shift avant cet appui
    uint64_t lastE = 0; bool haveE = false;
    for (uint64_t te : shiftEdges) { if (te <= p.t) { lastE = te; haveE = true; } else break; }
    const bool lockClear = !haveE || (p.t - lastE) > 320000;
    const bool must = p.rpmF >= W.minShift + 500 && lockClear && p.holdUs >= 10000 && !p.cutActive;
    const bool mustNotCut = p.rpmF < W.minShift - 500 && lockClear;
    bool got = false;
    for (uint64_t te : shiftEdges) if (te >= p.t && te <= p.t + 40000) { got = true; break; }
    // Un cut plus prioritaire (pit / overrev) qui demarre dans la fenetre explique legitimement l'absence de cut shift.
    bool preempted = false;
    for (auto& e : W.S.edges) if (e.t >= p.t && e.t <= p.t + 40000 && (e.m == AssistMode::PitLimit || e.m == AssistMode::Overrev)) { preempted = true; break; }
    if (must) { ++mustCut; if (!got && !preempted) { ++missed; if (missed == 1) W.violation("appui shifter valide sans coupure t=" + std::to_string(p.t / 1e6)); } }
    if (mustNotCut) { ++mustNot; if (got) { ++spurious; if (spurious == 1) W.violation("coupure shifter sous le regime mini t=" + std::to_string(p.t / 1e6)); } }
    (void)lastEdge; (void)ei; (void)lastExpected; (void)haveExpected;
  }
  if (mustCutOut) *mustCutOut = mustCut;
  if (missedOut) *missedOut = missed;
  if (spuriousOut) *spuriousOut = spurious;
}

// Boot du firmware avec la moto branchee.
static inline void bootWorld(LicenseTier t, double leanAmp = 0.0, bool checks = true) {
  W = World();
  W.minShift = cfg::DEFAULTS.minShiftRpm;
  W.checkInvariants = checks;
  host::nowUs = SEC(1);
  host::ds.temp = [](uint64_t) { return 78.0f; };
  W.leanFn = [leanAmp](uint64_t tt) {
    const double s = tt / 1e6, k = std::min(1.0, std::max(0.0, (W.B.vg - 8.0) / 25.0));
    return k * leanAmp * std::sin(s / 4.1) * std::sin(s / 13.0); };
  wireBike();
  setup();
  setTier(t);
  host::every(host::nowUs, 100, [] { loop(); return true; });
  watchCuts();
  host::every(host::nowUs, 100000, [] {
    int rl, lt; bool ok; state::getLean(rl, lt, ok); char s[200];
    snprintf(s, sizeof s, "t=%.1f vrai=%.1f est=%.1f ok=%d vg=%.0f ex=%.0f gear=%d eng=%d rpm=%.0f", host::nowUs / 1e6, leanAt(host::nowUs), rl / 10.0, ok, W.B.vg, W.B.ex, W.B.gear, W.B.engage > 0.5, W.B.rpm);
    W.hist.push_back(s); if (W.hist.size() > 80) W.hist.pop_front(); return true; });
}
