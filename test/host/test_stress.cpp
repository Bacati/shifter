// SRC: core/SharedState.cpp sensors/Sensors.cpp sensors/Imu.cpp sensors/WaterTemp.cpp actuator/IgnitionCut.cpp control/Arbiter.cpp persistence/License.cpp persistence/Storage.cpp net/Network.cpp net/Telemetry.cpp sim/SimEngine.cpp factory/Factory.cpp
//
// STRESS : le vrai firmware (production) sur une moto virtuelle complete -- boite 6 rapports, embrayage, couple,
// patinage, wheelie, virages -- pilotee par un pilote qui enchaine des centaines de departs, un millier de passages
// de vitesse au selecteur, des retrogradages avec/sans embrayage, avec des episodes etranges melanges.
#include "host.h"
#include "mock/Arduino.h"
#include "src/core/Config.h"
#include "src/core/SharedState.h"
#include "src/actuator/IgnitionCut.h"
#include "src/persistence/License.h"
#include "src/net/WebServer.h"
#include "common.h"
#include "shifter.ino"
#include "rider.h"
#include <fstream>

size_t web::sseClients() { return 1; }
void web::sseSend(const char*, const char*) {}
void web::begin() {}

// Controles communs a tous les scenarios de stress.
static void commonChecks(const RunResult& rr, bool accounting = true, double heapBytes = 2048, long heapAllocs = 24) {
  INFO("%.0f s simulees, %ld cycles, %ld montees (%ld retrogradages), %ld coupures [overrev %ld pit %ld shift %ld launch %ld slip %ld]",
       rr.simSeconds, R.cycles, R.ups, R.downs, W.S.cuts, W.S.byMode[5], W.S.byMode[4], W.S.byMode[3], W.S.byMode[2], W.S.byMode[1]);
  INFO("regime moteur max atteint %.0f ; plus longue coupure %.0f ms ; deriv. tas %+ld o / %+ld allocs", W.B.maxRpmEver, W.S.maxCutRun / 1000.0, rr.heapGrowthBytes, rr.allocGrowth);
  if (W.S.violations) for (auto& l : W.snap) INFO("%s", l.c_str());
  CHECK_MSG(W.S.violations == 0, "%ld violations d'invariants ; 1re : %s", W.S.violations, W.S.firstViolation.c_str());
  if (accounting) {
    long must = 0, missed = 0, spurious = 0;
    verifyShiftAccounting(&must, &missed, &spurious);
    INFO("appuis selecteur : %zu, devant couper : %ld, manques : %ld, coupures indues : %ld", W.S.presses.size(), must, missed, spurious);
    if (missed > 0) {
      const char* nm[6] = {"idle", "slip", "launch", "shift", "pit", "overrev"};
      for (auto& pr : W.S.presses) {
        if (std::fabs(pr.t / 1e6 - atof(W.S.firstViolation.substr(W.S.firstViolation.rfind('=') + 1).c_str())) < 0.0015) {
          INFO("appui manque : t=%.6f maintenu=%u us rpmF=%d busy=%d", pr.t / 1e6, pr.holdUs, pr.rpmF, pr.cutActive);
          for (auto& e : W.S.edges) if (e.t + 600000 > pr.t && e.t < pr.t + 200000) INFO("  front/mode %.4f s -> %s", e.t / 1e6, nm[(int)e.m]);
          for (auto& q : W.S.presses) if (q.t + 600000 > pr.t && q.t < pr.t + 200000) INFO("  appui %.4f s rpmF=%d hold=%u", q.t / 1e6, q.rpmF, q.holdUs);
        }
      }
    }
    CHECK_MSG(missed == 0, "%ld appuis valides sans coupure ; 1er : %s", missed, W.S.firstViolation.c_str());
    CHECK_MSG(spurious == 0, "%ld coupures shifter sous le regime mini", spurious);
    CHECK(must > 0);
  }
  CHECK(host::restartCount == 0);
  CHECK(host::tasksAlive() == host::taskCount());
  CHECK(host::cutHigh() == ignition::isActive());
  CHECK_MSG(W.B.maxRpmEver < cfg::OVERREV_RPM + 2000, "moteur a %.0f tr/min : limiteur depasse", W.B.maxRpmEver);
  CHECK_MSG(W.S.maxCutRunNoPit < 1600000, "coupure continue de %.0f ms (hors pit/launch)", W.S.maxCutRunNoPit / 1000.0);
  CHECK_MSG(W.S.maxCutRun < 4000000, "coupure continue de %.0f ms", W.S.maxCutRun / 1000.0);
  CHECK_MSG(rr.heapGrowthBytes <= heapBytes && rr.allocGrowth <= heapAllocs, "FUITE ? tas %+ld octets, %+ld allocations", rr.heapGrowthBytes, rr.allocGrowth);
  CHECK(host::cutEdges.size() < host::cutEdges.capacity());          // le test n'a pas realloue (mesure du tas propre)
}

static RiderOpt weird() {
  RiderOpt o;
  o.pMuDrop = 0.35; o.pWheelie = 0.12; o.pDoublePress = 0.06; o.pMissedShift = 0.03; o.pShortPress = 0.06;
  o.pChatter = 2; o.pBadDump = 0.15; o.pClutchlessDown = 0.3; o.pStallRelease = 0.02; o.pPitInAccel = 0.1;
  o.muMin = 0.5; o.muMax = 1.05; o.wot = true;
  return o;
}

// ============================ Courses completes, par licence ==================================================
static void courses(LicenseTier t, long cycles) {
  bootWorld(t, 30.0);
  RunResult rr = runRider(cycles, weird());
  CHECK(rr.finished);
  commonChecks(rr);
  CHECK_MSG(R.ups >= cycles * 3, "peu de passages de vitesse (%ld)", R.ups);
}
TEST(stress_courses_basic_150_cycles) { courses(LicenseTier::Basic, 150); }
TEST(stress_courses_track_150_cycles) { courses(LicenseTier::Track, 150); }
TEST(stress_courses_pro_150_cycles)   { courses(LicenseTier::Pro, 150); }

// ============================ 1000+ passages de vitesse =======================================================
TEST(stress_1000_passages_de_vitesse_pro) {
  bootWorld(LicenseTier::Pro, 25.0);
  RiderOpt o; o.topGearMin = 6; o.topGearMax = 6; o.topHoldMin = 0.5; o.topHoldMax = 1.0; o.pauseMin = 0.5; o.pauseMax = 1.0;
  RunResult rr = runRider(215, o);
  CHECK(rr.finished);
  commonChecks(rr);
  CHECK_MSG(R.ups >= 1000, "seulement %ld passages", R.ups);
  INFO("passages de vitesse au selecteur : %ld", R.ups);
}

// ============================ Retrogradages ===================================================================
TEST(stress_1250_retrogradages_avec_et_sans_embrayage) {
  bootWorld(LicenseTier::Pro, 25.0);
  RiderOpt o; o.topGearMin = 6; o.topGearMax = 6; o.topHoldMin = 0.2; o.topHoldMax = 0.6; o.pClutchlessDown = 0.35; o.pauseMin = 0.3; o.pauseMax = 0.8;
  RunResult rr = runRider(260, o);
  CHECK(rr.finished);
  commonChecks(rr);
  CHECK_MSG(R.downs >= 1000, "seulement %ld retrogradages", R.downs);
  INFO("retrogradages : %ld dont sans embrayage : %ld", R.downs, R.clutchlessDowns);
}

// ============================ Departs : launch / sans launch ==================================================
static void departs(LicenseTier t, double* meanStageOut = nullptr) {
  bootWorld(t, 0.0);
  RiderOpt o; o.topGearMin = 2; o.topGearMax = 2; o.topHoldMin = 0.1; o.topHoldMax = 0.3; o.pauseMin = 0.3; o.pauseMax = 0.6;
  o.muMin = 0.4; o.muMax = 1.1; o.releaseMsMin = 40; o.releaseMsMax = 900; o.stageSecMin = 1.5; o.stageSecMax = 3.0;
  RunResult rr = runRider(300, o);
  CHECK(rr.finished);
  commonChecks(rr);
  long nLim = 0, nHigh = 0; double mean = 0, spin = 0, t40 = 0; long nT40 = 0;
  for (auto& s : R.stats) {
    if (s.peakStageRpm <= cfg::DEFAULTS.launchRpm + 1000) ++nLim;
    if (s.peakStageRpm > 13000) ++nHigh;
    mean += s.peakStageRpm; spin += s.spin; if (s.t40 > 0) { t40 += s.t40; ++nT40; }
  }
  const double n = (double)R.stats.size();
  INFO("%s : %ld departs ; regime de depart max moyen %.0f ; sous cible+1000 : %ld/%.0f ; au-dessus de 13000 : %ld ; patinage moyen %.1f ; t(0-40) moyen %.2f s",
       toString(t), R.launches, mean / n, nLim, n, nHigh, spin / n, nT40 ? t40 / nT40 : -1.0);
  if (meanStageOut) *meanStageOut = mean / n;
  if (t >= LicenseTier::Track) CHECK_MSG(nLim >= n * 0.97, "launch: regime tenu seulement %ld/%.0f fois", nLim, n);
  else                         CHECK_MSG(nHigh >= n * 0.90, "BASIC : le moteur devrait monter librement (%ld/%.0f)", nHigh, n);
}
TEST(stress_departs_basic_300)  { departs(LicenseTier::Basic); }
TEST(stress_departs_track_300)  { departs(LicenseTier::Track); }
TEST(stress_departs_pro_300)    { departs(LicenseTier::Pro); }

// ============================ Patinage ========================================================================
static void patinage(LicenseTier t, const char* tag) {
  bootWorld(t, 0.0);
  RiderOpt o; o.topGearMin = 3; o.topGearMax = 4; o.topHoldMin = 0.5; o.topHoldMax = 1.0; o.pauseMin = 0.3; o.pauseMax = 0.6;
  o.pMuDrop = 1.0; o.muMin = 0.9; o.muMax = 1.05;
  RunResult rr = runRider(250, o);
  CHECK(rr.finished);
  commonChecks(rr);
  const double perCycle = W.B.spinTotal / R.cycles;
  INFO("%s : patinage cumule par cycle = %.1f km/h.s ; episodes %ld", tag, perCycle, R.muDrops);
  std::ofstream f(std::string("test/host/build/spin_") + tag + ".txt"); f << perCycle << "\n";
}
TEST(stress_patinage_basic_250) { patinage(LicenseTier::Basic, "basic"); }
TEST(stress_patinage_pro_250)   { patinage(LicenseTier::Pro, "pro"); }
TEST(stress_patinage_comparaison_pro_vs_basic) {
  double a = -1, b = -1;
  { std::ifstream f("test/host/build/spin_basic.txt"); f >> a; }
  { std::ifstream f("test/host/build/spin_pro.txt"); f >> b; }
  CHECK_MSG(a >= 0 && b >= 0, "metriques absentes (lancer stress_patinage_basic/pro avant)");
  INFO("patinage par cycle : BASIC %.1f ; PRO %.1f (%.0f %%)", a, b, 100.0 * b / std::max(1e-9, a));
  CHECK_MSG(b < a * 0.95, "le slip control PRO ne reduit pas le patinage (%.1f vs %.1f)", b, a);
}

// Patinage EN VIRAGE profond : le slip control ne doit jamais couper quand la moto est couchee a plus de ~35 deg reels.
TEST(stress_patinage_en_virage_profond_pro) {
  bootWorld(LicenseTier::Pro, 46.0);
  RiderOpt o; o.topGearMin = 3; o.topGearMax = 5; o.topHoldMin = 3; o.topHoldMax = 8; o.pMuDrop = 1.0; o.muMin = 0.5; o.muMax = 1.0;
  o.pauseMin = 0.5; o.pauseMax = 1.0;
  RunResult rr = runRider(220, o);
  CHECK(rr.finished);
  commonChecks(rr);
  INFO("coupures slip : %ld", W.S.byMode[1]);
  CHECK(W.S.byMode[1] > 20);                       // le slip fonctionne bien en ligne droite / petit angle
}

// ============================ Cas bizarres, repetes ===========================================================
TEST(stress_embrayage_hache_pendant_plein_gaz) {
  bootWorld(LicenseTier::Pro, 20.0);
  RiderOpt o = weird(); o.pChatter = 12; o.topGearMin = 4; o.topGearMax = 6;
  RunResult rr = runRider(90, o);
  CHECK(rr.finished);
  INFO("coups d'embrayage parasites : %ld", R.chatters);
  CHECK(R.chatters > 100);
  commonChecks(rr);
}
TEST(stress_calage_moteur_et_redemarrage_300) {
  bootWorld(LicenseTier::Track, 0.0);
  RiderOpt o; o.pStallRelease = 0.5; o.topGearMin = 2; o.topGearMax = 3; o.pauseMin = 0.3; o.pauseMax = 0.8; o.releaseMsMin = 30; o.releaseMsMax = 150;
  RunResult rr = runRider(300, o);
  CHECK(rr.finished);
  INFO("calages : %ld", W.B.stalls);
  CHECK(W.B.stalls > 50);
  commonChecks(rr, false);
  CHECK_MSG(state::getRpmFiltered() >= 0, "rpm");
}
TEST(stress_capteur_rpm_muet_pendant_plein_gaz) {
  bootWorld(LicenseTier::Pro, 0.0);
  RiderOpt o; o.topGearMin = 4; o.topGearMax = 6;
  host::every(host::nowUs + SEC(5), SEC(3), [] { W.rpmDropUntil = host::nowUs + (uint64_t)(50000 + 350000 * host::rnd()); return true; });
  RunResult rr = runRider(100, o);
  CHECK(rr.finished);
  commonChecks(rr, false);
}
TEST(stress_selecteur_maintenu_10s_et_rafales) {
  bootWorld(LicenseTier::Basic, 0.0);
  W.B.gear = 3; W.B.engage = 1; W.B.vg = 60; W.B.thr = 0.45;
  host::runFor(SEC(3));
  // 1) maintenu 10 s
  const uint64_t t0 = host::nowUs;
  riderPressShifter(t0 + MS(100), 10000000);
  host::runFor(SEC(12));
  long held = 0; for (auto& e : W.S.edges) if (e.m == AssistMode::Shift && e.t >= t0) ++held;
  INFO("coupures shifter pendant un appui maintenu 10 s : %ld", held);
  CHECK_MSG(held <= 1, "%ld coupures pour UN appui maintenu", held);
  // 2) rafales : 2000 appuis a intervalles aleatoires 5..400 ms
  const uint64_t t1 = host::nowUs; uint64_t t = t1 + MS(100);
  for (int i = 0; i < 2000; ++i) { riderPressShifter(t, (uint32_t)(3000 + host::rnd() * 60000)); t += (uint64_t)(5000 + host::rnd() * 395000); }
  host::runUntil(t + SEC(1));
  long n = 0; for (auto& e : W.S.edges) if (e.m == AssistMode::Shift && e.t >= t1) ++n;
  const double dur = (t - t1) / 1e6;
  INFO("rafale : 2000 appuis en %.0f s -> %ld coupures (max theorique %.0f)", dur, n, dur / 0.25 + 1);
  CHECK_MSG(n <= dur / 0.25 + 2, "verrou 250 ms non respecte : %ld coupures en %.0f s", n, dur);
  CHECK_MSG(W.S.violations == 0, "%s", W.S.firstViolation.c_str());
  CHECK(host::cutHigh() == ignition::isActive());
}
TEST(stress_selecteur_a_l_arret_point_mort) {
  bootWorld(LicenseTier::Basic, 0.0);
  W.B.gear = 0; W.B.vg = 0; W.B.thr = 0;
  host::runFor(SEC(2));
  uint64_t t = host::nowUs + MS(100);
  int atIdle = 0;
  for (int i = 0; i < 1500; ++i) {
    const double thr = (host::rnd() < 0.5) ? 0.0 : (0.1 + 0.9 * host::rnd());
    host::at(t, [thr] { W.B.thr = thr; });
    riderPressShifter(t + MS(300), 40000);
    t += MS(900);
    if (thr == 0.0) ++atIdle;
  }
  host::runUntil(t + SEC(1));
  long viol = W.S.violations; long must = 0, missed = 0, spur = 0; verifyShiftAccounting(&must, &missed, &spur);
  INFO("1500 appuis a l'arret (%d au ralenti) : %ld devant couper, %ld manques, %ld indus", atIdle, must, missed, spur);
  CHECK_MSG(viol == 0, "%s", W.S.firstViolation.c_str());
}
TEST(stress_vitesse_oscillant_autour_de_5kmh_launch_et_slip) {
  bootWorld(LicenseTier::Pro, 0.0);
  // moto en roue libre au point mort, vitesse sol imposee autour des seuils launch / slip (5 km/h)
  W.B.gear = 0; W.B.engage = 0;
  host::every(host::nowUs, 5000, [] { const double s = host::nowUs / 1e6; W.B.vg = 5.0 + 1.3 * std::sin(s * 2 * M_PI / 0.45) + 0.4 * std::sin(s * 2 * M_PI / 0.11);
                                      W.B.ex = (std::sin(s * 3.1) > 0.3) ? 0.35 * W.B.vg : 0.0; W.B.thr = (W.B.rpm < 8500) ? 1.0 : 0.2; return true; });
  host::every(host::nowUs + 200, 37000, [] { pinClutch(host::rnd() < 0.7); return true; });
  host::runFor(SEC(600));
  INFO("coupures : launch %ld, slip %ld", W.S.byMode[2], W.S.byMode[1]);
  CHECK_MSG(W.S.violations == 0, "%ld violations ; 1re : %s", W.S.violations, W.S.firstViolation.c_str());
  CHECK(host::restartCount == 0 && host::tasksAlive() == host::taskCount());
}
TEST(stress_entrees_simultanees_embrayage_pit_selecteur) {
  bootWorld(LicenseTier::Pro, 25.0);
  RiderOpt o = weird();
  host::every(host::nowUs + SEC(3), 300000, [] {
    const uint64_t t = host::nowUs + 1;
    const int mask = (int)(host::rnd() * 8);
    host::at(t, [mask] { if (mask & 1) host::setPin(pins::CLUTCH_HALL, 0); if (mask & 2) host::setPin(pins::PIT_BTN, 0); if (mask & 4) host::setPin(pins::SHIFTER, 0); });
    host::at(t + (uint64_t)(20000 + host::rnd() * 200000), [] { host::setPin(pins::CLUTCH_HALL, 1); host::setPin(pins::PIT_BTN, 1); host::setPin(pins::SHIFTER, 1); });
    return true; });
  RunResult rr = runRider(80, o);
  CHECK(rr.finished);
  commonChecks(rr, false);
}
TEST(stress_debordement_micros_et_millis_90_minutes) {
  host::baseUs = 0xFFFFFFFFull - SEC(35 * 60);           // micros() deborde dans 35 min
  host::baseMs = 0xFFFFFFFFull - 50ull * 60 * 1000;      // millis() deborde dans 50 min
  bootWorld(LicenseTier::Pro, 28.0);
  RunResult rr = runRider(1000000, weird(), 90 * 60);
  INFO("duree simulee %.0f min", rr.simSeconds / 60.0);
  CHECK(rr.simSeconds > 85 * 60);
  CHECK(host::nowUs + host::baseUs > 0xFFFFFFFFull);      // le debordement micros a bien eu lieu
  commonChecks(rr);
}
TEST(stress_configs_extremes_400_combinaisons) {
  bootWorld(LicenseTier::Pro, 25.0);
  RiderOpt o; o.topGearMin = 2; o.topGearMax = 4; o.topHoldMin = 0.1; o.topHoldMax = 0.4; o.pauseMin = 0.2; o.pauseMax = 0.4;
  R = Rider(); R.o = o; W.S.edges.reserve(1 << 20); W.S.presses.reserve(1 << 17); host::cutEdges.reserve(1 << 21);
  R.enter(Rider::PAUSE); R.phDurS = 0.5;
  host::every(host::nowUs, 2000, [] { R.tick(); return true; });
  auto pick = [](int lo, int hi) { double r = host::rnd(); return r < 0.3 ? lo : (r < 0.6 ? hi : lo + (int)(host::rnd() * (hi - lo))); };
  for (int i = 0; i < 400; ++i) {
    EcuConfig c = { pick(cfg::SHIFT_CUT_MIN, cfg::SHIFT_CUT_MAX), pick(cfg::MIN_RPM_MIN, cfg::MIN_RPM_MAX), pick(cfg::LAUNCH_MIN, cfg::LAUNCH_MAX),
                    pick(cfg::SLIP_MIN, cfg::SLIP_MAX), pick(cfg::WHEEL_MIN, cfg::WHEEL_MAX), pick(cfg::PULSES_MIN, cfg::PULSES_MAX),
                    pick(cfg::PIT_MIN, cfg::PIT_MAX), pick(cfg::PITKMH_MIN, cfg::PITKMH_MAX) };
    state::setConfig(c);                       // le reglage est applique a chaud, comme par la page web
    W.minShift = c.minShiftRpm;
    const long target = R.cycles + 1;
    int guard = 0;
    while (R.cycles < target && guard++ < 40) host::runFor(SEC(5));
    host::serialOut.clear();
  }
  INFO("400 configurations, %ld cycles, %ld coupures", R.cycles, W.S.cuts);
  CHECK_MSG(W.S.violations == 0, "%ld violations ; 1re : %s", W.S.violations, W.S.firstViolation.c_str());
  CHECK(host::restartCount == 0 && host::tasksAlive() == host::taskCount());
  CHECK(host::cutHigh() == ignition::isActive());
  CHECK_MSG(W.S.maxCutRunNoPit < 1600000, "coupure continue de %.0f ms (hors pit/launch)", W.S.maxCutRunNoPit / 1000.0);
}
TEST(stress_changements_de_licence_en_roulant) {
  bootWorld(LicenseTier::Basic, 20.0);
  RiderOpt o = weird();
  long unlocks = 0, ok = 0;
  host::every(host::nowUs + SEC(7), SEC(11), [&] {
    uint32_t retry = 0; const int pick = (int)(host::rnd() * 5);
    String key = (pick < 3) ? license::expectedKey((LicenseTier)pick) : String("DEADBEEFDEADBEEF");
    ++unlocks; if (license::tryUnlock(key, retry)) ++ok;
    return true; });
  RunResult rr = runRider(120, o, 30000);
  INFO("%ld tentatives de licence, %ld reussies, ecritures NVS %d", unlocks, ok, host::nvsWrites);
  CHECK(host::nvsWrites <= ok * 2 + 4);                                    // pas d'ecriture flash parasite
  commonChecks(rr, false);
}
TEST(stress_pit_limiter_spam_et_rapport_haut) {
  bootWorld(LicenseTier::Basic, 0.0);
  RiderOpt o; o.pPitInAccel = 1.0; o.topGearMin = 3; o.topGearMax = 5; o.pauseMin = 0.3; o.pauseMax = 0.7;
  host::every(host::nowUs + SEC(4), 200000, [] { if (host::rnd() < 0.4) host::setPin(pins::PIT_BTN, host::rnd() < 0.5 ? 0 : 1); return true; });
  RunResult rr = runRider(120, o);
  CHECK(rr.finished);
  commonChecks(rr, false);
}

// ============================ Fuite memoire sur longue duree ==================================================
TEST(stress_fuite_memoire_3_heures_de_course) {
  bootWorld(LicenseTier::Pro, 28.0);
  RunResult rr = runRider(1000000, weird(), 3 * 3600);
  INFO("duree simulee : %.0f min", rr.simSeconds / 60.0);
  CHECK(rr.simSeconds >= 2.5 * 3600);
  commonChecks(rr, true, 512, 8);
}
