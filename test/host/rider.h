#pragma once
// Pilote virtuel : enchaine departs, plein gaz avec passages de vitesse au selecteur, freinage avec retrogradages.
#include "bike.h"

struct RiderOpt {
  bool   wot = true;                     // plein gaz au depart (sinon le pilote tient le regime de depart)
  double stageRpmMin = 7000, stageRpmMax = 12000;
  double stageSecMin = 1.2, stageSecMax = 2.5;
  double releaseMsMin = 80, releaseMsMax = 700;
  int    topGearMin = 3, topGearMax = 6;
  double topHoldMin = 1.0, topHoldMax = 3.0;
  double upRpmMin = 12500, upRpmMax = 15200;
  double pauseMin = 1.0, pauseMax = 3.0;
  // episodes etranges (probabilite par cycle)
  double pMuDrop = 0, pWheelie = 0, pDoublePress = 0, pMissedShift = 0, pShortPress = 0;
  double muMin = 1.0, muMax = 1.0;       // adherence de base, tiree a chaque depart
  double pChatter = 0, pBadDump = 0, pClutchlessDown = 0, pStallRelease = 0, pPitInAccel = 0;
};

struct CycleStat { double peakStageRpm = 0, spin = 0, peakSpin = 0, t40 = -1; };

struct Rider {
  enum Ph { PAUSE, STAGE, RELEASE, ACCEL, BRAKE } ph = PAUSE;
  RiderOpt o;
  uint64_t phStart = 0;
  double phDurS = 1.0, stageDurS = 2.0, stageRpm = 9000, releaseMs = 300, upRpm = 14000, brakeRate = 25, topHold = 2;
  int topGear = 5;
  double baseMu = 1.0;
  uint64_t lastPress = 0, topSince = 0, muEnd = 0, wheelieEnd = 0, chatterEnd = 0, pitEnd = 0;
  int ds = 0; uint64_t dsT = 0;
  bool wheelieDone = false, muDone = false, pitDone = false;
  bool wantMuDrop = false, wantWheelie = false, wantPit = false;
  uint64_t muAt = 0, wheelieAt = 0, pitAt = 0;
  long cycles = 0, ups = 0, downs = 0, launches = 0, wheelies = 0, muDrops = 0, chatters = 0, doubles = 0, clutchlessDowns = 0;
  bool inCycle = false;
  std::vector<CycleStat> stats;
  CycleStat cur;

  static double uni(double a, double b) { return a + (b - a) * host::rnd(); }
  void enter(Ph p) { ph = p; phStart = host::nowUs; }
  double since() const { return (host::nowUs - phStart) / 1e6; }

  void press(uint64_t now) {
    uint32_t hold = (uint32_t)uni(40000, 90000);
    if (host::rnd() < o.pShortPress) hold = (uint32_t)uni(10000, 14000);
    riderPressShifter(now, hold);
    lastPress = now;
    if (host::rnd() >= o.pMissedShift) host::at(now + 15000, [] { if (W.B.gear < 6) W.B.gear++; });
    ++ups;
    if (host::rnd() < o.pDoublePress) { riderPressShifter(now + (uint64_t)uni(60000, 200000), 40000); ++doubles; }
  }

  void tick() {
    Bike& B = W.B;
    const uint64_t now = host::nowUs;
    if (muEnd && now > muEnd) { B.mu = baseMu; muEnd = 0; }
    if (wheelieEnd && now > wheelieEnd) { B.frontFactor = 1.0; wheelieEnd = 0; }
    if (pitEnd && now > pitEnd) { host::setPin(pins::PIT_BTN, 1); pitEnd = 0; }

    switch (ph) {
      case PAUSE: {
        B.thr = 0; B.brake = (B.vg > 0.2) ? 20 : 0;
        if (!B.engineOn && B.vg < 0.5) { B.engineOn = true; B.rpm = 1400; }      // demarreur
        if (since() > phDurS && B.vg < 0.5) {
          enter(STAGE);
          pinClutch(true); B.engage = 0; B.gear = 0; baseMu = uni(o.muMin, o.muMax); B.mu = baseMu;
          host::at(now + 200000, [] { W.B.gear = 1; });
          stageRpm = uni(o.stageRpmMin, o.stageRpmMax); stageDurS = uni(o.stageSecMin, o.stageSecMax);
          releaseMs = uni(o.releaseMsMin, o.releaseMsMax);
          if (host::rnd() < o.pBadDump) releaseMs = 40;
          if (host::rnd() < o.pStallRelease) { stageRpm = 2500; B.thr = 0; }
          topGear = o.topGearMin + (int)(host::rnd() * (o.topGearMax - o.topGearMin + 1)); if (topGear > o.topGearMax) topGear = o.topGearMax;
          topHold = uni(o.topHoldMin, o.topHoldMax); upRpm = uni(o.upRpmMin, o.upRpmMax);
          wantMuDrop = host::rnd() < o.pMuDrop; muAt = now + (uint64_t)uni(2.0e6, 8.0e6); muDone = false;
          wantWheelie = host::rnd() < o.pWheelie; wheelieAt = now + (uint64_t)uni(2.3e6, 4.0e6); wheelieDone = false;
          wantPit = host::rnd() < o.pPitInAccel; pitAt = now + (uint64_t)uni(4.0e6, 9.0e6); pitDone = false;
          cur = CycleStat(); B.spinIntegral = 0; B.peakSpin = 0; B.peakRpm = 0;
          ++launches; inCycle = true;
        }
        break;
      }
      case STAGE: {
        B.thr = o.wot ? 1.0 : ((B.rpm < stageRpm) ? 1.0 : 0.3);
        if (stageRpm < 3000) B.thr = 0;
        cur.peakStageRpm = std::max(cur.peakStageRpm, B.rpm);
        if (since() > stageDurS) { enter(RELEASE); B.peakSpin = 0; B.spinIntegral = 0; }
        break;
      }
      case RELEASE: {
        const double e = since() * 1000.0 / releaseMs;
        B.engage = std::min(1.0, e);
        if (e > 0.15) pinClutch(false);
        B.thr = (stageRpm < 3000) ? 0.0 : 1.0;
        if (stageRpm < 3000 && e > 0.4 && B.engineOn) { B.engineOn = false; B.rpm = 0; ++B.stalls; }   // lache sans gaz : cale
        if (e >= 1.0) { enter(ACCEL); topSince = 0; }
        break;
      }
      case ACCEL: {
        B.thr = 1.0; B.brake = 0;
        if (!B.engineOn) { enter(BRAKE); break; }
        if (cur.t40 < 0 && B.vg >= 40.0) { cur.t40 = since(); cur.spin = B.spinIntegral; cur.peakSpin = B.peakSpin; }
        if (wantMuDrop && !muDone && now > muAt) { B.mu = 0.3; muEnd = now + (uint64_t)uni(0.5e6, 2.0e6); muDone = true; ++muDrops; }
        if (wantWheelie && !wheelieDone && now > wheelieAt && B.gear <= 2) { B.frontFactor = 0.65; wheelieEnd = now + 600000; wheelieDone = true; ++wheelies; }
        if (wantPit && !pitDone && now > pitAt) { host::setPin(pins::PIT_BTN, 0); pitEnd = now + (uint64_t)uni(0.5e6, 1.5e6); pitDone = true; }
        if (chatterEnd && now > chatterEnd) { chatterEnd = 0; pinClutch(false); }
        if (!chatterEnd && host::rnd() < o.pChatter * 0.0004) {                        // coup d'embrayage parasite
          pinClutch(true); B.engage = 0.0; chatterEnd = now + (uint64_t)uni(30000, 120000); ++chatters;
          host::at(chatterEnd, [] { W.B.engage = 1.0; });
        }
        if (B.gear < topGear && B.rpm > upRpm && now - lastPress > 350000 && B.engage >= 1.0) press(now);
        if (B.gear >= topGear) {
          if (!topSince) topSince = now;
          if ((now - topSince) / 1e6 > topHold || B.vg > 118) enter(BRAKE);
        }
        break;
      }
      case BRAKE: {
        B.brake = brakeRate;
        if (ds == 0) B.thr = 0;
        if (!B.engineOn) B.thr = 0;
        // retrogradage : si le regime apres le rapport inferieur reste raisonnable
        if (ds == 0 && B.gear > 1) {
          const double rpmAfter = (B.vg + B.ex) * Bike::kRpmPerKmh * Bike::ratio(B.gear - 1);
          if (rpmAfter < 12800) {
            if (host::rnd() < o.pClutchlessDown) {                                      // sans embrayage : selecteur + gaz
              riderPressShifter(now, 50000); lastPress = now; ++clutchlessDowns;
              host::at(now + 15000, [] { if (W.B.gear > 1) W.B.gear--; });
              ds = 5; dsT = now; B.thr = 1.0;
            } else { ds = 1; dsT = now; B.engage = 0; pinClutch(true); }
          }
        } else if (ds == 1 && now - dsT >= 40000) { if (B.gear > 1) B.gear--; B.thr = 1.0; ds = 2;
        } else if (ds == 2 && now - dsT >= 110000) { B.thr = 0; ds = 3;
        } else if (ds == 3) {
          const double e = (now - dsT - 110000) / 200000.0;
          B.engage = std::min(1.0, std::max(0.0, e));
          if (e > 0.3) pinClutch(false);
          if (e >= 1.0) { ds = 0; ++downs; }
        } else if (ds == 5 && now - dsT >= 120000) { B.thr = 0; ds = 0; ++downs; }
        if (ds == 0 && B.gear == 1 && B.vg < 7.0) { B.engage = 0; pinClutch(true); B.gear = 0; }
        if (B.vg < 0.3) { cycles++; inCycle = false; stats.push_back(cur); enter(PAUSE); phDurS = uni(o.pauseMin, o.pauseMax); B.brake = 0; B.thr = 0; ds = 0; B.engage = 0; }
        break;
      }
    }
  }
};

static Rider R;

// Execute jusqu'a `cycles` cycles (ou limite de temps). Echantillonne le tas pour detecter les fuites.
struct RunResult { long heapGrowthBytes = 0, allocGrowth = 0; double simSeconds = 0; bool finished = false; };

static inline RunResult runRider(long cycles, const RiderOpt& opt, double maxSimSeconds = 20000) {
  R = Rider(); R.o = opt;
  R.stats.reserve(1 << 17); W.S.edges.reserve(1 << 20); W.S.presses.reserve(1 << 17); host::cutEdges.reserve(1 << 21); host::serialOut.reserve(1 << 20);
  R.enter(Rider::PAUSE); R.phDurS = 1.0;
  host::every(host::nowUs, 2000, [] { R.tick(); return true; });
  RunResult res;
  const uint64_t t0 = host::nowUs;
  long baseBytes = -1, baseAllocs = -1; long lastBytes = 0, lastAllocs = 0;
  uint64_t nextSample = t0 + SEC(180);
  while (R.cycles < cycles && (host::nowUs - t0) < SEC(maxSimSeconds)) {
    host::runFor(SEC(30));
    host::serialOut.clear();
    if (host::nowUs >= nextSample) {
      if (baseBytes < 0) { baseBytes = host::liveBytes(); baseAllocs = host::liveAllocs(); }
      lastBytes = host::liveBytes(); lastAllocs = host::liveAllocs();
      nextSample += SEC(120);
    }
  }
  res.heapGrowthBytes = (baseBytes < 0) ? 0 : lastBytes - baseBytes;
  res.allocGrowth = (baseAllocs < 0) ? 0 : lastAllocs - baseAllocs;
  res.simSeconds = (host::nowUs - t0) / 1e6;
  res.finished = R.cycles >= cycles;
  return res;
}
