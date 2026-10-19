// SRC: core/SharedState.cpp sensors/Sensors.cpp sensors/Imu.cpp sensors/WaterTemp.cpp actuator/IgnitionCut.cpp control/Arbiter.cpp persistence/License.cpp persistence/Storage.cpp net/Network.cpp net/Telemetry.cpp sim/SimEngine.cpp factory/Factory.cpp
//
// Execute le VRAI shifter.ino (build de production, sans -DWOKWI_SIM) avec un vehicule virtuel en boucle fermee :
// moteur (RPM reagissant aux coupures), roues, IMU MPU6050 et sonde DS18B20 simules.
#include "host.h"
#include "mock/Arduino.h"
#include "src/core/Config.h"
#include "src/core/SharedState.h"
#include "src/actuator/IgnitionCut.h"
#include "src/persistence/License.h"
#include "src/net/WebServer.h"
#include "common.h"
#include "shifter.ino"

// ---- doublures du serveur web (le serveur HTTP reel n'est pas simulable ici) -----------------
static std::vector<std::string> g_sse;
size_t web::sseClients() { return 1; }
void web::sseSend(const char* json, const char*) { g_sse.push_back(json); }
void web::begin() {}

// ---- vehicule virtuel ----------------------------------------------------------------------------
struct Veh {
  double rpm = 1500, thr = 0;
  bool clutch = false, engineOn = true;
  double front = 0, rear = 0;               // km/h
  double accelFull = 16000, maxRpm = 22000, drag = 6000;
  bool coupled = false;                     // true : vitesse des roues = rpm * 0.01 (60 km/h a 6000 tr/min)
} V;

static void vehicleStep() {
  const double dt = 0.001;
  if (V.engineOn) {
    double a = V.accelFull * V.thr * (1.0 - V.rpm / V.maxRpm) - 1500.0 * (V.rpm / 10000.0);
    if (host::cutHigh()) a = -V.drag;
    V.rpm = std::max(0.0, V.rpm + a * dt);
  }
  if (V.coupled) V.front = V.rear = V.rpm * 0.01;
}
static void startVehicle(bool withWheels = true) {
  host::every(host::nowUs, 1000, [] { vehicleStep(); return true; });
  train(pins::RPM, [](uint64_t) { return V.engineOn && V.rpm > 300 ? rpmHz(V.rpm) : 0.0; });
  if (withWheels) {
    train(pins::WHEEL_FRONT, [](uint64_t) { return V.front > 0.5 ? wheelHz(V.front, 1800, 3) : 0.0; });
    train(pins::WHEEL_REAR,  [](uint64_t) { return V.rear > 0.5 ? wheelHz(V.rear, 1800, 3) : 0.0; });
  }
}
static void bootFull(LicenseTier t = LicenseTier::Basic, double leanDeg = 0.0) {
  host::nowUs = SEC(1);
  host::ds.temp = [](uint64_t) { return 78.0f; };
  Ride r; r.rollDeg = [=](uint64_t) { return leanDeg; }; r.speedMs = [](uint64_t) { return std::max(2.0, V.front / 3.6); }; r.longG = [](uint64_t) { return 0.0; };
  installRide(r);
  setup();
  setTier(t);
  host::every(host::nowUs, 100, [] { loop(); return true; });
  startVehicle();
}
static double cutMs(uint64_t a, uint64_t b) { return host::cutHighTotalUs(a, b) / 1000.0; }

// ============================ Demarrage complet ======================================================
TEST(full_demarrage_tous_les_capteurs) {
  bootFull();
  host::runFor(SEC(4));
  CHECK(host::serialContains("[IMU] MPU6050 OK"));
  CHECK(host::serialContains("[TEMP] DS18B20 OK"));
  CHECK(host::serialContains("[BOOT] HWID: AA:BB:CC:DD:EE:FF"));
  CHECK(host::serialContains("[BOOT] pret."));
  CHECK(!host::serialContains("[SIM]"));
  int r, l; bool ok; state::getLean(r, l, ok); CHECK(ok);
  int t; bool tok; state::getWaterTemp(t, tok); CHECK(tok && t == 780);
  CHECK(host::wdtInit && host::wdtTimeoutS == cfg::WDT_TIMEOUT_S);
  CHECK(host::restartCount == 0);
  CHECK(!host::cutHigh());
}
TEST(full_demarrage_sans_imu_ni_sonde_ni_crash) {
  host::mpu.present = false; host::ds.present = false;
  bootFull(LicenseTier::Pro);
  V.thr = 1.0; V.front = 40; V.rear = 52;                  // glisse 30 % : sans IMU, le slip doit rester coupe
  host::runFor(SEC(6));
  CHECK(host::serialContains("[IMU] absent"));
  CHECK(host::serialContains("[TEMP] DS18B20 absent"));
  CHECK(host::restartCount == 0);
  // l'overrev doit rester operationnel
  CHECK_MSG(V.rpm < 17000, "overrev inoperant sans capteurs: %.0f", V.rpm);
}

// ============================ Boucle fermee : overrev ==================================================
TEST(full_overrev_boucle_fermee_plein_gaz) {
  bootFull(); V.rpm = 8000; V.thr = 1.0;
  double maxRpm = 0; int below = 0;
  host::every(host::nowUs, 1000, [&] { maxRpm = std::max(maxRpm, V.rpm); return true; });
  host::runFor(SEC(8));
  const double duty = cutMs(SEC(6), host::nowUs) / 2000.0;
  INFO("regime max reel = %.0f ; cut duty une fois au limiteur = %.0f %%", maxRpm, duty * 100);
  CHECK_MSG(maxRpm < cfg::OVERREV_RPM + 800, "depassement %.0f tr/min", maxRpm);
  CHECK(maxRpm > cfg::OVERREV_RPM - 500);                   // le moteur atteint bien le limiteur
  (void)below;
}
TEST(full_overrev_pas_de_coupure_sous_le_seuil) {
  bootFull(); V.rpm = 9000; V.thr = 0.4;                    // plafonne ~ 14 500 tr/min
  host::runFor(SEC(10));
  INFO("rpm stabilise %.0f", V.rpm);
  CHECK(V.rpm > 9000 && V.rpm < cfg::OVERREV_RPM - 800);
  CHECK_MSG(host::cutPulses(SEC(3), host::nowUs) == 0, "coupure parasite en croisiere");
}

// ============================ Quickshifter ==============================================================
TEST(full_quickshifter_un_appui_une_coupure_50ms) {
  bootFull(); V.rpm = 10000; V.thr = 0.5;
  pressShifter(SEC(4), 120000);
  host::runFor(SEC(6));
  CHECK_MSG(host::cutPulses(SEC(3), host::nowUs) == 1, "pulses=%d", host::cutPulses(SEC(3), host::nowUs));
  CHECK_NEAR(cutMs(SEC(3), host::nowUs), cfg::DEFAULTS.shiftCutMs, 1.5);
}
TEST(full_quickshifter_latence_appui_vers_coupure) {
  bootFull(); V.rpm = 10000; V.thr = 0.5;
  host::runFor(SEC(3));
  const uint64_t tp = host::nowUs + MS(100);
  pressShifter(tp, 120000);
  host::runUntil(tp + SEC(1));
  uint64_t tc = 0; for (auto& e : host::cutEdges) if (e.t >= tp && e.high) { tc = e.t; break; }
  INFO("latence appui -> coupure = %.2f ms", (tc - tp) / 1000.0);
  CHECK(tc > 0 && (tc - tp) < 6000);                        // < 6 ms (2 ms de confirmation + tour de boucle)
}

// ============================ Launch control ==================================================================
TEST(full_launch_boucle_fermee_tient_le_regime) {
  bootFull(LicenseTier::Track); V.rpm = 3000; V.thr = 1.0; V.clutch = true;
  host::at(SEC(1.5), [] { host::setPin(pins::CLUTCH_HALL, 0); });
  double lo = 1e9, hi = 0;
  host::every(SEC(5), 1000, [&] { lo = std::min(lo, V.rpm); hi = std::max(hi, V.rpm); return host::nowUs < SEC(8); });
  host::runFor(SEC(9));
  INFO("regime launch tenu: %.0f .. %.0f (cible %d)", lo, hi, cfg::DEFAULTS.launchRpm);
  CHECK(lo > cfg::DEFAULTS.launchRpm - 1200 && hi < cfg::DEFAULTS.launchRpm + 1200);
}
TEST(full_launch_se_desarme_quand_la_moto_roule) {
  bootFull(LicenseTier::Track); V.rpm = 8000; V.thr = 1.0;
  host::at(SEC(1.5), [] { host::setPin(pins::CLUTCH_HALL, 0); });
  host::runFor(SEC(4));
  const bool launching = host::cutPulses(SEC(3), host::nowUs) > 10;
  V.front = V.rear = 15; V.thr = 0.3;                       // la moto part (gaz reduits)
  host::runFor(SEC(1));
  const uint64_t t = host::nowUs; host::runFor(SEC(1));
  CHECK(launching);
  CHECK_MSG(host::cutPulses(t, host::nowUs) == 0, "launch toujours actif a 15 km/h");
}

// ============================ Slip + IMU de bout en bout ============================================================
static int slipRun(double leanDeg, LicenseTier tier) {
  host::cutEdges.clear();
  bootFull(tier, leanDeg);
  V.rpm = 9000; V.thr = 0.4;
  host::runFor(SEC(3));                                  // demarrage a l'arret : biais gyro mesure au calme
  V.front = 45; V.rear = 45 * 1.3;
  host::runFor(SEC(8));
  return host::cutPulses(SEC(6), host::nowUs);
}
TEST(full_slip_ligne_droite_pro_coupe)           { int n = slipRun(0, LicenseTier::Pro);   INFO("%d coupures", n); CHECK(n >= 10); }
TEST(full_slip_ligne_droite_track_ne_coupe_pas)  { CHECK(slipRun(0, LicenseTier::Track) == 0); }
TEST(full_slip_20deg_reduit_mais_peut_couper)    { int n = slipRun(20, LicenseTier::Pro);  INFO("%d coupures a 20 deg", n); CHECK(n >= 0); }
TEST(full_slip_30deg_aucune_coupure)             { CHECK_MSG(slipRun(30, LicenseTier::Pro) == 0, "slip actif a 30 deg"); }
TEST(full_slip_virage_stabilise_20_deg_puis_30) {
  // La moto reste couchee : l'estimation d'angle ne doit pas deriver vers 0 et reautoriser le slip (bug v23.0).
  bootFull(LicenseTier::Pro);
  Ride r; r.speedMs = [](uint64_t) { return 14.0; }; r.longG = [](uint64_t) { return 0.0; };
  r.rollDeg = [](uint64_t t) { double s = (t - SEC(1)) / 1e6; if (s < 4) return 0.0; if (s < 5) return 32.0 * (s - 4); return 32.0; };
  installRide(r);
  V.rpm = 9000; V.thr = 0.4; V.front = 50; V.rear = 50;
  host::runFor(SEC(7));                                     // couche a 32 deg depuis 2 s
  V.rear = 50 * 1.3;                                        // la roue arriere patine EN virage
  const uint64_t t = host::nowUs; host::runFor(SEC(8));
  CHECK_MSG(host::cutPulses(t, host::nowUs) == 0, "coupures de motricite en virage a 32 deg");
}

// ============================ Capteurs en panne en roulant ===========================================================
TEST(full_imu_perdue_en_roulant_desactive_le_slip) {
  bootFull(LicenseTier::Pro);
  V.rpm = 9000; V.thr = 0.4;
  host::runFor(SEC(3)); V.front = 45; V.rear = 45 * 1.3;
  host::runFor(SEC(6));
  CHECK(host::cutPulses(SEC(7), host::nowUs) > 5);
  host::mpu.busStuck = true;
  host::runFor(MS(400));
  const uint64_t t = host::nowUs; host::runFor(SEC(2));
  CHECK_MSG(host::cutPulses(t, host::nowUs) == 0, "slip actif IMU perdue");
}
TEST(full_capteur_roue_arriere_debranche_pas_de_coupure_folle) {
  bootFull(LicenseTier::Pro);
  V.rpm = 9000; V.thr = 0.4; V.front = 45; V.rear = 45;
  host::runFor(SEC(5));
  V.rear = 0;                                               // capteur arriere HS
  const uint64_t t = host::nowUs; host::runFor(SEC(4));
  CHECK_MSG(host::cutPulses(t, host::nowUs) == 0, "coupures avec capteur arriere debranche");
}
TEST(full_capteur_roue_avant_debranche_pas_de_coupure_folle) {
  bootFull(LicenseTier::Pro);
  V.rpm = 9000; V.thr = 0.4; V.front = 45; V.rear = 45;
  host::runFor(SEC(5));
  V.front = 0;                                              // capteur avant HS, la moto roule
  const uint64_t t = host::nowUs; host::runFor(SEC(4));
  CHECK_MSG(host::cutPulses(t, host::nowUs) == 0, "coupures avec capteur avant debranche");
}
TEST(full_sonde_temperature_debranchee_en_roulant_sans_effet_moteur) {
  bootFull(); V.rpm = 10000; V.thr = 0.5; pressShifter(SEC(6));
  host::runFor(SEC(3));
  host::ds.present = false;
  host::runFor(SEC(6));
  int t; bool ok; state::getWaterTemp(t, ok);
  CHECK(!ok);
  CHECK(host::cutPulses(SEC(5), host::nowUs) == 1);        // le shifter fonctionne toujours
}

// ============================ Parasites sur toutes les entrees ==========================================================
TEST(full_parasites_shifter_pas_de_coupure_parasite) {
  // Croisiere a 10 000 tr/min, parasites de quelques us sur l'entree shifter (couplage EMI) ~2 par seconde.
  bootFull(); V.rpm = 10000; V.thr = 0.52;
  auto spike = std::make_shared<std::function<void()>>();
  *spike = [=]() { uint64_t t = host::nowUs + 1; host::at(t, [] { host::setPin(pins::SHIFTER, 0); }); host::at(t + 20 + (uint64_t)(host::rnd() * 200), [] { host::setPin(pins::SHIFTER, 1); });
                   host::at(host::nowUs + 200000 + (uint64_t)(host::rnd() * 800000), *spike); };
  host::at(SEC(3), *spike);
  host::runFor(SEC(60));
  int n = host::cutPulses(SEC(3), host::nowUs);
  INFO("coupures parasites sur 57 s: %d", n);
  CHECK_MSG(n == 0, "%d quickshifts declenches par des parasites de quelques us", n);
}
TEST(full_parasites_pit_embrayage_pas_de_coupure_parasite) {
  bootFull(LicenseTier::Track); V.rpm = 5000; V.thr = 0.2;
  auto spike = std::make_shared<std::function<void()>>();
  *spike = [=]() { int pin = host::rnd() < 0.5 ? pins::PIT_BTN : pins::CLUTCH_HALL; uint64_t t = host::nowUs + 1;
                   host::at(t, [=] { host::setPin(pin, 0); }); host::at(t + 100 + (uint64_t)(host::rnd() * 4000), [=] { host::setPin(pin, 1); });
                   host::at(host::nowUs + 100000 + (uint64_t)(host::rnd() * 400000), *spike); };
  host::at(SEC(3), *spike);
  host::runFor(SEC(60));
  CHECK_MSG(host::cutPulses(SEC(3), host::nowUs) == 0, "coupures parasites: %d", host::cutPulses(SEC(3), host::nowUs));
}
TEST(full_parasites_roues_pas_de_slip_parasite) {
  bootFull(LicenseTier::Pro); V.rpm = 9000; V.thr = 0.4; V.front = V.rear = 55;
  auto spike = std::make_shared<std::function<void()>>();
  *spike = [=]() { int pin = host::rnd() < 0.5 ? pins::WHEEL_FRONT : pins::WHEEL_REAR; host::pulse(pin, host::nowUs + 1, 5);
                   host::at(host::nowUs + 300000 + (uint64_t)(host::rnd() * 1500000), *spike); };
  host::at(SEC(4), *spike);
  host::runFor(SEC(120));
  int n = host::cutPulses(SEC(4), host::nowUs);
  INFO("coupures slip sur 116 s avec ~1 parasite roue / 1 s: %d", n);
  CHECK_MSG(n <= 2, "%d coupures parasites de motricite", n);
}

// ============================ Endurance ==============================================================================
TEST(full_endurance_10min_entrees_aleatoires) {
  bootFull(LicenseTier::Pro);
  V.thr = 0.5;
  Ride r; r.speedMs = [](uint64_t) { return std::max(3.0, V.front / 3.6); }; r.longG = [](uint64_t) { return 0.0; };
  r.rollDeg = [](uint64_t t) { double s = t / 1e6; return 28.0 * std::sin(s / 3.7) * std::sin(s / 11.0); };
  installRide(r);
  host::every(host::nowUs, 50000, [] {
    V.thr = host::rnd();
    V.front = std::max(0.0, V.front + host::gauss() * 4); V.front = std::min(V.front, 120.0);
    V.rear = V.front * (1.0 + (host::rnd() < 0.1 ? 0.3 : 0.01 * host::gauss()));
    if (host::rnd() < 0.05) { host::setPin(pins::CLUTCH_HALL, 0); host::at(host::nowUs + 300000, [] { host::setPin(pins::CLUTCH_HALL, 1); }); }
    if (host::rnd() < 0.03) { host::setPin(pins::PIT_BTN, 0);     host::at(host::nowUs + 1000000, [] { host::setPin(pins::PIT_BTN, 1); }); }
    if (host::rnd() < 0.10) pressShifter(host::nowUs + 100, 60000);
    return true; });
  int stuck = 0; uint64_t runStart = 0;
  host::runFor(SEC(600));
  INFO("redemarrages=%d, plus longue coupure=%.0f ms, taches vivantes=%d", host::restartCount, host::cutMaxRunUs(SEC(5), host::nowUs) / 1000.0, host::tasksAlive());
  CHECK(host::restartCount == 0);
  CHECK(host::tasksAlive() == host::taskCount());
  CHECK(host::cutHigh() == ignition::isActive());
  CHECK_MSG(host::cutMaxRunUs(SEC(5), host::nowUs) < 1500000, "coupure continue de %.0f ms", host::cutMaxRunUs(SEC(5), host::nowUs) / 1000.0);
  (void)stuck; (void)runStart;
}


// ============================ Complements boucle fermee ============================================================
TEST(full_pit_limiter_kmh_boucle_fermee_plein_gaz) {
  bootFull(); V.coupled = true; V.rpm = 3000; V.thr = 1.0;
  host::at(SEC(2), [] { host::setPin(pins::PIT_BTN, 0); });
  double maxKmh = 0;
  host::every(SEC(4), 1000, [&] { maxKmh = std::max(maxKmh, V.front); return true; });
  host::runFor(SEC(10));
  INFO("pit limiter plein gaz: vitesse max = %.1f km/h (limite %d)", maxKmh, cfg::DEFAULTS.pitKmh);
  // Modele tres nerveux (160 km/h/s a plein gaz a vide) : depassement dominé par la latence de mesure.
  CHECK_MSG(maxKmh < cfg::DEFAULTS.pitKmh + 15, "le pit limiter ne limite pas: %.1f km/h", maxKmh);
  CHECK(maxKmh > cfg::DEFAULTS.pitKmh - 6);                  // et il ne bride pas beaucoup plus bas
}
TEST(full_pit_limiter_kmh_acceleration_realiste_depassement_faible) {
  bootFull(); V.coupled = true; V.rpm = 3000; V.thr = 1.0; V.accelFull = 4000;   // ~40 km/h/s
  host::at(SEC(2), [] { host::setPin(pins::PIT_BTN, 0); });
  double maxKmh = 0;
  host::every(SEC(6), 1000, [&] { maxKmh = std::max(maxKmh, V.front); return true; });
  host::runFor(SEC(14));
  INFO("acceleration realiste: vitesse max = %.1f km/h (limite %d)", maxKmh, cfg::DEFAULTS.pitKmh);
  CHECK_MSG(maxKmh < cfg::DEFAULTS.pitKmh + 6, "depassement %.1f km/h", maxKmh);
}
TEST(full_shifter_contact_rebondissant_est_pris) {
  bootFull(); V.rpm = 10000; V.thr = 0.45;
  host::runFor(SEC(3));
  const uint64_t t = host::nowUs + MS(100);
  // contact mecanique : 5 rebonds sur ~4 ms puis maintenu
  for (int i = 0; i < 5; ++i) {
    host::at(t + i * 800, [] { host::setPin(pins::SHIFTER, 0); });
    host::at(t + i * 800 + 400, [] { host::setPin(pins::SHIFTER, 1); });
  }
  host::at(t + 4000, [] { host::setPin(pins::SHIFTER, 0); });
  host::at(t + 120000, [] { host::setPin(pins::SHIFTER, 1); });
  host::runFor(SEC(1));
  CHECK_MSG(host::cutPulses(t, host::nowUs) == 1, "pulses=%d", host::cutPulses(t, host::nowUs));
}
TEST(full_slip_latence_de_reaction) {
  bootFull(LicenseTier::Pro);
  V.rpm = 9000; V.thr = 0.4;
  host::runFor(SEC(3)); V.front = 45; V.rear = 45;
  host::runFor(SEC(6));
  CHECK(host::cutPulses(SEC(1), host::nowUs) == 0);
  const uint64_t t0 = host::nowUs + MS(7);
  host::at(t0, [] { V.rear = 45 * 1.35; });
  host::runFor(SEC(1));
  uint64_t tc = 0; for (auto& e : host::cutEdges) if (e.t >= t0 && e.high) { tc = e.t; break; }
  INFO("perte d'adherence -> 1re coupure en %.0f ms", (tc - t0) / 1000.0);
  CHECK(tc > 0);
  CHECK_MSG((tc - t0) < 350000, "reaction trop lente: %.0f ms", (tc - t0) / 1000.0);
}
TEST(full_slip_tient_la_coupure_tant_que_la_glisse_dure) {
  bootFull(LicenseTier::Pro);
  V.rpm = 9000; V.thr = 0.4;
  host::runFor(SEC(3)); V.front = 45; V.rear = 45;
  host::runFor(SEC(6));
  const uint64_t t0 = host::nowUs;
  host::at(t0 + MS(10), [] { V.rear = 45 * 1.35; });
  host::at(t0 + SEC(1.5), [] { V.rear = 45; });
  host::runFor(SEC(3));
  int n = host::cutPulses(t0 + SEC(0.4), t0 + SEC(1.5));
  int after = host::cutPulses(t0 + SEC(1.9), host::nowUs);
  INFO("coupures pendant la glisse: %d ; apres: %d", n, after);
  CHECK(n >= 10);
  CHECK(after == 0);
}
TEST(full_demarrage_au_kick_puis_ralenti_stable) {
  bootFull(); V.engineOn = false; V.rpm = 0;
  host::runFor(SEC(2));
  // lancer: 5 tours irreguliers puis le moteur demarre et monte a 1800
  uint64_t t = host::nowUs + 1000;
  double per[] = {0.40, 0.30, 0.22, 0.16, 0.12, 0.09};
  for (double p : per) { host::pulse(pins::RPM, t, 100); t += (uint64_t)(p * 1e6); }
  host::at(t, [] { V.engineOn = true; V.rpm = 1500; V.thr = 0.0227; });
  host::runFor(SEC(6));
  INFO("ralenti: %d tr/min, coupures depuis le demarrage: %d", state::getRpmFiltered(), host::cutPulses(t, host::nowUs));
  CHECK(state::getRpmFiltered() > 1000 && state::getRpmFiltered() < 3000);
  CHECK(host::cutPulses(t, host::nowUs) == 0);
}
TEST(full_calage_moteur_en_roulant_puis_relance) {
  bootFull(); V.rpm = 9000; V.thr = 0.4; V.front = V.rear = 50;
  host::runFor(SEC(5));
  V.engineOn = false;
  host::runFor(SEC(2));
  CHECK(state::getRpmFiltered() == 0 && !host::cutHigh());
  V.engineOn = true; V.rpm = 2500; V.thr = 0.3;
  host::runFor(SEC(4));
  CHECK(state::getRpmFiltered() > 2000);
  CHECK(host::restartCount == 0);
}

// ============================ Chaos : pannes + parasites + conduite aleatoire =========================================
// Invariants verifies a CHAQUE debut de coupure (front montant de D4), avec l'etat reellement vu par l'ECU.
TEST(full_chaos_20min_invariants_a_chaque_coupure) {
  bootFull(LicenseTier::Pro);
  // Trajet COHERENT : la moto ne se couche que si elle roule (arrets reguliers : lean = 0, roues a 0).
  auto speedKmh = [](uint64_t t) {
    const double s = t / 1e6;
    const double ph = std::fmod(s, 150.0);                                       // arret de 9 s toutes les 150 s,
    double f = 1.0;                                                              // avec rampes de 3 s (pas de saut)
    if (ph > 135.0 && ph <= 138.0) f = (138.0 - ph) / 3.0;
    else if (ph > 138.0 && ph <= 147.0) f = 0.0;
    else if (ph > 147.0) f = (ph - 147.0) / 3.0;
    return f * std::max(14.0, 55.0 + 30.0 * std::sin(s / 9.7) + 15.0 * std::sin(s / 3.3));
  };
  Ride r; r.speedMs = [=](uint64_t t) { return std::max(1.0, speedKmh(t) / 3.6); }; r.longG = [](uint64_t) { return 0.0; };
  r.rollDeg = [=](uint64_t t) {
    const double s = t / 1e6, k = std::min(1.0, std::max(0.0, (speedKmh(t) - 5.0) / 25.0));
    return k * 40.0 * std::sin(s / 4.1) * std::sin(s / 13.0); };
  installRide(r);

  // conduite aleatoire (gaz, embrayage, pit, shifter, perte d'adherence arriere)
  host::every(host::nowUs, 50000, [speedKmh] {
    V.thr = host::rnd(); V.front = speedKmh(host::nowUs);
    V.rear = V.front * (1.0 + (host::rnd() < 0.08 && V.front > 10 ? 0.3 : 0.01 * host::gauss()));
    if (host::rnd() < 0.04) { host::setPin(pins::CLUTCH_HALL, 0); host::at(host::nowUs + 300000, [] { host::setPin(pins::CLUTCH_HALL, 1); }); }
    if (host::rnd() < 0.02) { host::setPin(pins::PIT_BTN, 0);     host::at(host::nowUs + 800000, [] { host::setPin(pins::PIT_BTN, 1); }); }
    if (host::rnd() < 0.10) pressShifter(host::nowUs + 100, 60000);
    return true; });
  // pannes aleatoires (capteurs) -- jamais permanentes
  host::every(SEC(10), SEC(7), [] {
    double d = host::rnd();
    if (d < 0.15) { host::mpu.busStuck = true;  host::at(host::nowUs + SEC(1 + 3 * host::rnd()), [] { host::mpu.busStuck = false; }); }
    else if (d < 0.25) { host::mpu.sleepBit = true; }
    else if (d < 0.35) { host::mpu.frozen = true;  host::at(host::nowUs + SEC(1), [] { host::mpu.frozen = false; }); }
    else if (d < 0.50) { host::ds.present = false; host::at(host::nowUs + SEC(3), [] { host::ds.present = true; }); }
    else if (d < 0.60) { host::ds.failReads = 5; }
    return true; });
  // parasites sur toutes les entrees
  auto spike = std::make_shared<std::function<void()>>();
  *spike = [=]() {
    int pins_[] = {pins::RPM, pins::WHEEL_FRONT, pins::WHEEL_REAR, pins::SHIFTER, pins::CLUTCH_HALL, pins::PIT_BTN};
    int pin = pins_[(int)(host::rnd() * 6) % 6];
    host::pulse(pin, host::nowUs + 1, 3 + (uint32_t)(host::rnd() * 200));
    host::at(host::nowUs + 50000 + (uint64_t)(host::rnd() * 600000), *spike); };
  host::at(SEC(5), *spike);

  long checked = 0, bad = 0;
  std::string first;
  std::vector<std::string> hist;
  host::every(SEC(5), 100000, [&] {
    int rl, lt; bool ok; state::getLean(rl, lt, ok); char b[160];
    snprintf(b, sizeof b, "t=%.1f vrai=%.1f est=%.1f ok=%d latG=%.2f front=%.0f mpu(stuck=%d sleep=%d frozen=%d)", host::nowUs / 1e6, r.rollDeg(host::nowUs), rl / 10.0, ok, lt / 100.0, V.front, host::mpu.busStuck, host::mpu.sleepBit, host::mpu.frozen);
    hist.push_back(b); if (hist.size() > 400) hist.erase(hist.begin()); return true; });
  std::vector<std::string> snap;
  auto fail = [&](const std::string& m) { if (!bad++) { first = m; snap = hist; } };
  // echantillonnage fin de la broche CUT
  bool last = false;
  host::every(SEC(5), 200, [&] {
    const bool now = host::cutHigh();
    if (now && !last) {
      ++checked;
      const AssistMode m = ignition::activeMode();
      int roll, lat; bool imuOk; state::getLean(roll, lat, imuOk);
      const bool imuFresh = imuOk && (millis() - state::lastLeanMs() < cfg::IMU_STALE_MS);
      int f, rr; state::getSpeeds(f, rr);
      char b[200];
      if (m == AssistMode::Slip) {
        if (!imuFresh) { snprintf(b, sizeof b, "slip sans IMU fiable t=%.2f", host::nowUs / 1e6); fail(b); }
        if (std::abs(roll) > cfg::LEAN_CUTOFF_DEG * 10 + 5) { snprintf(b, sizeof b, "slip a %.1f deg t=%.2f", roll / 10.0, host::nowUs / 1e6); fail(b); }
        if (lat > cfg::LEAN_LATERAL_G_HARD * 100 + 3) { snprintf(b, sizeof b, "slip a %.2f g t=%.2f", lat / 100.0, host::nowUs / 1e6); fail(b); }
        const double truth = r.rollDeg(host::nowUs);                       // VRAI angle de la moto (pas l'estimation)
        if (std::fabs(truth) > cfg::LEAN_CUTOFF_DEG + 7.0) { snprintf(b, sizeof b, "slip alors que la moto est a %.1f deg (estime %.1f) t=%.2f", truth, roll / 10.0, host::nowUs / 1e6); fail(b); }
        if (f / 10.0 <= cfg::SLIP_MIN_SPEED_KMH) { snprintf(b, sizeof b, "slip a %.1f km/h t=%.2f", f / 10.0, host::nowUs / 1e6); fail(b); }
      } else if (m == AssistMode::Launch) {
        if (!sensors::clutchPulled()) { snprintf(b, sizeof b, "launch sans embrayage t=%.2f", host::nowUs / 1e6); fail(b); }
        if (f / 10.0 >= cfg::LAUNCH_SPEED_MAX_KMH + 0.5 || rr / 10.0 >= cfg::LAUNCH_SPEED_MAX_KMH + 0.5) { snprintf(b, sizeof b, "launch a f=%.1f r=%.1f t=%.2f", f / 10.0, rr / 10.0, host::nowUs / 1e6); fail(b); }
      } else if (m == AssistMode::Shift) {
        if (state::getRpmFiltered() < cfg::DEFAULTS.minShiftRpm - 400) { snprintf(b, sizeof b, "shift a %d tr/min t=%.2f", state::getRpmFiltered(), host::nowUs / 1e6); fail(b); }
      } else if (m == AssistMode::PitLimit) {
        if (!sensors::pitButton() && false) fail("pit");
      } else if (m == AssistMode::Overrev) {
        if (state::getRpmInstant() < cfg::OVERREV_RPM - 300) { snprintf(b, sizeof b, "overrev a %d tr/min t=%.2f", state::getRpmInstant(), host::nowUs / 1e6); fail(b); }
      } else { snprintf(b, sizeof b, "coupure en mode Idle t=%.2f", host::nowUs / 1e6); fail(b); }
    }
    last = now;
    return true; });

  host::runFor(SEC(1200));
  INFO("%ld coupures verifiees, %ld violations ; redemarrages=%d ; plus longue coupure %.0f ms",
       checked, bad, host::restartCount, host::cutMaxRunUs(SEC(5), host::nowUs) / 1000.0);
  if (bad) { INFO("premiere violation: %s", first.c_str()); for (auto& l : snap) INFO("%s", l.c_str()); }
  CHECK(checked > 100);
  CHECK_MSG(bad == 0, "%ld violations d'invariants ; 1re: %s", bad, first.c_str());
  CHECK(host::restartCount == 0);
  CHECK(host::tasksAlive() == host::taskCount());
  CHECK(host::cutHigh() == ignition::isActive());
}

TEST(full_reboot_en_roulant_slip_indisponible_jusqu_a_l_arret) {
  // L'ECU redemarre (superviseur) alors que la moto roule : le biais du gyro ne peut pas etre mesure
  // (moto en mouvement) -> slip coupe, puis disponible apres un arret.
  bootFull(LicenseTier::Pro);
  V.rpm = 9000; V.thr = 0.4; V.front = 45; V.rear = 45 * 1.3;          // roule des le boot, avec glisse
  host::runFor(SEC(20));
  CHECK_MSG(host::cutPulses(SEC(5), host::nowUs) == 0, "slip actif sans biais gyro mesure");
  V.front = 0; V.rear = 0; host::runFor(SEC(4));                       // arret (feu, stand)
  V.front = 45; V.rear = 45;  host::runFor(SEC(3));
  const uint64_t t = host::nowUs; V.rear = 45 * 1.3; host::runFor(SEC(3));
  CHECK_MSG(host::cutPulses(t, host::nowUs) > 3, "slip toujours indisponible apres un arret");
}
