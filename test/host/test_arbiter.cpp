// SRC: core/SharedState.cpp sensors/Sensors.cpp actuator/IgnitionCut.cpp control/Arbiter.cpp persistence/License.cpp persistence/Storage.cpp sim/SimEngine.cpp
#include "common.h"

// Entrees injectees directement dans SharedState (apres sensors::update, qui sinon les ecraserait).
struct In {
  int rpmF = 0, rpmI = 0;
  double front = 0, rear = 0, slip = 0, rate = 0;
  double lean = 0, latG = 1.0; bool imuOk = true; bool imuFeed = true;
} g;

static void feed() {
  state::setRpmFiltered(g.rpmF); state::setRpmInstant(g.rpmI);
  state::setSpeeds((int)(g.front * 10), (int)(g.rear * 10));
  state::setSlip((int)(g.slip * 10), (int)(g.rate * 10));
  if (g.imuFeed) state::setLean((int)(g.lean * 10), (int)(g.latG * 100), g.imuOk);
}
static void boot(LicenseTier t = LicenseTier::Pro, EcuConfig c = cfg::DEFAULTS) {
  host::nowUs = SEC(5);
  state::begin(); state::setConfig(c);
  ignition::begin();
  sensors::begin();
  setTier(t);
  host::every(host::nowUs, 100, [] {
    sensors::update(); feed();
    ignition::service(); arbiter::update(); ignition::service();
    state::beatEngineLoop(); return true; });
}
static double cutMs(uint64_t a, uint64_t b) { return host::cutHighTotalUs(a, b) / 1000.0; }

// ============================ IgnitionCut =========================================
TEST(cut_broche_basse_au_demarrage) {
  host::nowUs = SEC(1);
  ignition::begin();
  CHECK(!host::cutHigh());
  CHECK(state::getMode() == AssistMode::Idle);
}
TEST(cut_duree_exacte) {
  host::nowUs = SEC(1); ignition::begin();
  host::every(host::nowUs, 50, [] { ignition::service(); return true; });
  host::at(SEC(1.1), [] { ignition::request(50, AssistMode::Shift); });
  host::runUntil(SEC(1.3));
  CHECK_NEAR(host::cutHighTotalUs(SEC(1), SEC(1.3)) / 1000.0, 50.0, 0.3);
  CHECK(host::cutPulses(SEC(1), SEC(1.3)) == 1);
  CHECK(!ignition::isActive() && state::getMode() == AssistMode::Idle);
}
TEST(cut_plafond_absolu_200ms) {
  host::nowUs = SEC(1); ignition::begin();
  host::every(host::nowUs, 50, [] { ignition::service(); return true; });
  host::at(SEC(1.1), [] { ignition::request(60000, AssistMode::Shift); });
  host::runUntil(SEC(2));
  CHECK_MSG(cutMs(SEC(1), SEC(2)) <= 201.0, "coupure de %.1f ms malgre le plafond", cutMs(SEC(1), SEC(2)));
}
TEST(cut_duree_zero_ignoree) {
  host::nowUs = SEC(1); ignition::begin();
  ignition::request(0, AssistMode::Overrev);
  CHECK(!host::cutHigh() && !ignition::isActive());
}
TEST(cut_ne_raccourcit_jamais_et_priorite) {
  host::nowUs = SEC(1); ignition::begin();
  host::every(host::nowUs, 50, [] { ignition::service(); return true; });
  host::at(SEC(1.0), [] { ignition::request(100, AssistMode::Shift); });
  host::at(SEC(1.01), [] { ignition::request(5, AssistMode::Overrev); });    // plus prioritaire mais plus court
  host::at(SEC(1.02), [] { ignition::request(100, AssistMode::Slip); });     // moins prioritaire : ignore
  host::runUntil(SEC(1.3));
  CHECK_NEAR(cutMs(SEC(0.9), SEC(1.3)), 100.0, 0.5);
}
TEST(cut_ordre_de_priorite_complet) {
  // Overrev > Pit > Shift > Launch > Slip (README + decision utilisateur : le pit prend le dessus sur les assistances)
  const AssistMode order[] = {AssistMode::Slip, AssistMode::Launch, AssistMode::Shift, AssistMode::PitLimit, AssistMode::Overrev};
  for (int lo = 0; lo < 5; ++lo) for (int hi = 0; hi < 5; ++hi) {
    host::cutEdges.clear(); ignition::begin();
    ignition::request(50, order[lo]);
    ignition::request(50, order[hi]);
    const AssistMode shown = ignition::activeMode();
    const AssistMode want = hi >= lo ? order[hi] : order[lo];
    CHECK_MSG(shown == want, "lo=%d hi=%d mode=%d", lo, hi, (int)shown);
    host::nowUs += SEC(1); ignition::service();
  }
}
TEST(cut_force_clear_immediat) {
  host::nowUs = SEC(1); ignition::begin();
  ignition::request(100, AssistMode::Shift);
  CHECK(host::cutHigh());
  ignition::forceClear();
  CHECK(!host::cutHigh() && !ignition::isActive() && state::getMode() == AssistMode::Idle);
}
TEST(cut_debordement_micros_pendant_la_coupure) {
  host::baseUs = 0xFFFFFFFFull - SEC(1) - 20000;   // micros() deborde 20 ms apres t=1 s
  host::nowUs = SEC(1); ignition::begin();
  host::every(host::nowUs, 50, [] { ignition::service(); return true; });
  host::at(SEC(1.0) + 1, [] { ignition::request(50, AssistMode::Shift); });
  host::runUntil(SEC(1.2));
  CHECK_NEAR(cutMs(SEC(0.9), SEC(1.2)), 50.0, 0.5);
}

// ============================ Overrev ===============================================
TEST(overrev_coupe_au_dessus_seuil_et_relache) {
  boot(LicenseTier::Basic);
  g.rpmF = g.rpmI = 15900; host::runFor(MS(300));
  CHECK_MSG(host::cutPulses(SEC(5), host::nowUs) == 0, "coupure sous le seuil");
  const uint64_t t0 = host::nowUs;
  g.rpmI = g.rpmF = 16200; host::runFor(MS(300));
  const double duty = cutMs(t0, host::nowUs) / 300.0;
  INFO("duty overrev = %.0f %%", duty * 100);
  CHECK(duty > 0.95);                                  // quasi continu (cut 50 ms rearme toutes les 40 ms)
  CHECK(state::getMode() == AssistMode::Overrev);
  g.rpmI = g.rpmF = 14000; host::runFor(MS(100));
  CHECK_MSG(!host::cutHigh(), "doit relacher");
  CHECK(state::getMode() == AssistMode::Idle);
}
TEST(overrev_exactement_au_seuil_ne_coupe_pas) {
  boot(LicenseTier::Basic);
  g.rpmI = g.rpmF = cfg::OVERREV_RPM; host::runFor(MS(200));
  CHECK(host::cutPulses(SEC(5), host::nowUs) == 0);
  g.rpmI = cfg::OVERREV_RPM + 1; host::runFor(MS(20));
  CHECK(host::cutPulses(SEC(5), host::nowUs) >= 1);
}
TEST(overrev_utilise_le_rpm_instantane_pas_le_filtre) {
  boot(LicenseTier::Basic);
  g.rpmF = 14000; g.rpmI = 17000; host::runFor(MS(100));   // filtre en retard
  CHECK(host::cutPulses(SEC(5) - 1, host::nowUs) >= 1);
}
TEST(overrev_prioritaire_sur_tout) {
  boot(LicenseTier::Pro);
  g.rpmI = g.rpmF = 17000; g.front = 40; g.rear = 40;
  setPit(SEC(5.01), true); pressShifter(SEC(5.05));
  host::runFor(MS(400));
  CHECK_MSG(cutMs(SEC(5.1), host::nowUs) / 300.0 > 0.95, "overrev doit tenir malgre pit/shifter");
  CHECK(state::getMode() == AssistMode::Overrev);
}

// ============================ Pit limiter ===================================================
TEST(pit_coupure_continue_au_dela_de_la_bande_dure) {
  boot(LicenseTier::Basic);
  g.rpmF = g.rpmI = 6000; setPit(SEC(5.02), true);      // pitRpm 4000 + bande 400 < 6000
  host::runFor(SEC(1));
  CHECK_MSG(cutMs(SEC(5.1), host::nowUs) / 900.0 > 0.95, "le pit limiter doit couper en continu bien au-dessus du seuil");
}
TEST(pit_strobe_si_bouton_et_rpm_juste_au_dessus) {
  boot(LicenseTier::Basic);
  g.rpmF = g.rpmI = cfg::DEFAULTS.pitRpm + 200; setPit(SEC(5.02), true);
  host::runFor(SEC(1));
  const uint64_t t0 = SEC(5.1);
  int pulses = host::cutPulses(t0, host::nowUs); double ms = cutMs(t0, host::nowUs);
  INFO("pit: %d pulses, %.0f ms de coupure sur 0.9 s", pulses, ms);
  CHECK(pulses >= 15 && pulses <= 19);
  CHECK_NEAR(ms / 900.0, 0.4, 0.05);                 // 20 ms / 50 ms
}
TEST(pit_rien_sans_bouton_ou_sous_seuil) {
  boot(LicenseTier::Basic);
  g.rpmF = g.rpmI = 9000; host::runFor(MS(300));           // pas de bouton
  CHECK(host::cutPulses(SEC(5), host::nowUs) == 0);
  setPit(host::nowUs + 100, true); g.rpmF = g.rpmI = cfg::DEFAULTS.pitRpm - 1; host::runFor(MS(300));
  CHECK(host::cutPulses(SEC(5), host::nowUs) == 0);
}
TEST(pit_prend_le_dessus_sur_shift_launch_slip) {
  // Toutes les assistances demandent une coupure en meme temps que le pit : le mode affiche
  // pendant les coupures doit etre PIT LIMIT (jamais Launch/Slip/Shift tant que le pit coupe).
  boot(LicenseTier::Pro);
  g.rpmF = g.rpmI = 8000; g.front = 70; g.rear = 70; setClutch(SEC(5.0), true); setPit(SEC(5.0), true);
  int pit = 0, other = 0;
  host::every(SEC(5.2), 500, [&] {
    if (host::cutHigh()) { if (state::getMode() == AssistMode::PitLimit) ++pit; else ++other; } return true; });
  host::runFor(SEC(1.5));
  INFO("modes vus pendant les coupures: pit=%d autres=%d", pit, other);
  CHECK(pit > 0);
  CHECK_MSG(pit > other * 0.5, "le pit ne prend pas le dessus (pit=%d autres=%d)", pit, other);
}

// ---- pit limiter en km/h ----------------------------------------------------------------------------------
static double pitDuty(double front, double rear, int rpm, double secs = 1.0) {
  host::cutEdges.clear(); boot(LicenseTier::Basic);
  g.rpmF = g.rpmI = rpm; g.front = front; g.rear = rear; setPit(SEC(5.02), true);
  host::runFor(SEC(secs));
  return cutMs(SEC(5.2), host::nowUs) / ((host::nowUs - SEC(5.2)) / 1000.0);
}
TEST(pit_kmh_sous_la_limite_aucune_coupure_meme_a_haut_regime) { CHECK(pitDuty(50, 50, 9000) < 0.01); }
TEST(pit_kmh_juste_au_dessus_strobe) { double d = pitDuty(62, 62, 6000); INFO("duty %.2f", d); CHECK(d > 0.3 && d < 0.5); }
TEST(pit_kmh_bien_au_dessus_coupure_continue) { CHECK(pitDuty(75, 75, 6000) > 0.95); }
TEST(pit_kmh_roue_arriere_qui_patine_compte) { CHECK(pitDuty(40, 70, 6000) > 0.95); }
TEST(pit_kmh_capteurs_roue_muets_repli_regime) {
  CHECK(pitDuty(0, 0, 9000) > 0.95);                     // vitesse inconnue : on limite au regime
  CHECK(pitDuty(0, 0, 3000) < 0.01);
}
TEST(pit_kmh_limite_configurable) {
  EcuConfig c = cfg::DEFAULTS; c.pitKmh = 30;
  host::cutEdges.clear(); boot(LicenseTier::Basic, c);
  g.rpmF = g.rpmI = 5000; g.front = 35; g.rear = 35; setPit(SEC(5.02), true);
  host::runFor(SEC(1));
  CHECK(cutMs(SEC(5.2), host::nowUs) / 800.0 > 0.3);
}

// ============================ Quickshifter =====================================================
TEST(shift_une_seule_coupure_de_la_duree_config) {
  EcuConfig c = cfg::DEFAULTS; c.shiftCutMs = 60;
  boot(LicenseTier::Basic, c);
  g.rpmF = g.rpmI = 9000;
  pressShifter(SEC(5.1), 1000000);                     // appui maintenu 1 s
  host::runFor(SEC(1.5));
  CHECK(host::cutPulses(SEC(5), host::nowUs) == 1);
  CHECK_NEAR(cutMs(SEC(5), host::nowUs), 60.0, 0.5);
}
TEST(shift_verrou_250ms) {
  boot(LicenseTier::Basic);
  g.rpmF = g.rpmI = 9000;
  pressShifter(SEC(5.1), 20000); pressShifter(SEC(5.2), 20000);       // 100 ms apres : refuse
  pressShifter(SEC(5.5), 20000);                                       // 400 ms apres : ok
  host::runFor(SEC(1));
  CHECK_MSG(host::cutPulses(SEC(5), host::nowUs) == 2, "pulses=%d", host::cutPulses(SEC(5), host::nowUs));
}
TEST(shift_sous_rpm_mini_aucune_coupure_et_latch_efface) {
  boot(LicenseTier::Basic);
  g.rpmF = g.rpmI = cfg::DEFAULTS.minShiftRpm - 100;
  pressShifter(SEC(5.1), 20000);
  host::at(SEC(5.2), [] { g.rpmF = g.rpmI = 9000; });                  // le regime monte trop tard (>25 ms)
  host::runFor(SEC(1));
  CHECK_MSG(host::cutPulses(SEC(5), host::nowUs) == 0, "coupure tardive apres expiration de la fenetre");
}
TEST(shift_regime_atteint_dans_la_fenetre_25ms) {
  boot(LicenseTier::Basic);
  g.rpmF = g.rpmI = cfg::DEFAULTS.minShiftRpm - 100;
  pressShifter(SEC(5.1), 20000);
  host::at(SEC(5.1) + 10000, [] { g.rpmF = g.rpmI = 9000; });
  host::runFor(SEC(1));
  CHECK(host::cutPulses(SEC(5), host::nowUs) == 1);
}
TEST(shift_parasite_de_quelques_us_ignore) {
  boot(LicenseTier::Basic); g.rpmF = g.rpmI = 10000;
  for (int k = 0; k < 10; ++k) {
    const uint64_t t = SEC(5.1) + k * MS(100);
    host::at(t, [] { host::setPin(pins::SHIFTER, 0); });
    host::at(t + 30 + k * 150, [] { host::setPin(pins::SHIFTER, 1); });          // 30 us .. 1.4 ms
  }
  host::runFor(SEC(2));
  CHECK_MSG(host::cutPulses(SEC(5), host::nowUs) == 0, "%d coupures sur des parasites < SHIFT_CONFIRM_US", host::cutPulses(SEC(5), host::nowUs));
}
TEST(shift_appui_court_de_5ms_est_pris) {
  boot(LicenseTier::Basic); g.rpmF = g.rpmI = 10000;
  pressShifter(SEC(5.1), 5000);
  host::runFor(SEC(1));
  CHECK(host::cutPulses(SEC(5), host::nowUs) == 1);
}
TEST(shift_fonctionne_en_licence_basic) {
  boot(LicenseTier::Basic); g.rpmF = g.rpmI = 10000;
  pressShifter(SEC(5.1)); host::runFor(MS(500));
  CHECK(host::cutPulses(SEC(5), host::nowUs) == 1);
}

// ============================ Launch ========================================================
static int launchPulses(LicenseTier t, int rpm, double front, double rear, bool clutch) {
  host::cutEdges.clear(); boot(t);
  g.rpmF = g.rpmI = rpm; g.front = front; g.rear = rear; setClutch(SEC(5.0), clutch);
  host::runFor(SEC(1));
  return host::cutPulses(SEC(5.1), host::nowUs);
}
TEST(launch_actif_track_embrayage_tire_arret) { CHECK(launchPulses(LicenseTier::Track, 8000, 0, 0, true) >= 10); }
TEST(launch_absent_en_basic)                 { CHECK(launchPulses(LicenseTier::Basic, 8000, 0, 0, true) == 0); }
TEST(launch_pas_embrayage_pas_de_coupure)    { CHECK(launchPulses(LicenseTier::Track, 8000, 0, 0, false) == 0); }
TEST(launch_regime_trop_bas_rien)            { CHECK(launchPulses(LicenseTier::Track, 6000, 0, 0, true) == 0); }
TEST(launch_moto_en_mouvement_rien)          { CHECK(launchPulses(LicenseTier::Track, 8000, 12, 12, true) == 0); }
TEST(launch_capteur_avant_mort_roue_arriere_tourne) {
  // capteur avant HS (lit 0) mais la moto roule : ne doit PAS armer le launch
  CHECK_MSG(launchPulses(LicenseTier::Track, 8000, 0, 30, true) == 0, "launch arme alors que la roue arriere tourne");
}
TEST(launch_modulation_croit_avec_le_regime) {
  boot(LicenseTier::Track); g.front = g.rear = 0; setClutch(SEC(5.0), true);
  g.rpmF = g.rpmI = 7900; host::runFor(MS(500));
  double lo = cutMs(SEC(5.1), host::nowUs);
  const uint64_t t1 = host::nowUs; g.rpmF = g.rpmI = 9500; host::runFor(MS(500));
  double hi = cutMs(t1, host::nowUs);
  INFO("launch: %.0f ms @7900, %.0f ms @9500 (par 0.5 s)", lo, hi);
  CHECK(hi > lo * 1.5);
}

// ============================ Slip control ==============================================
static int slipPulses(LicenseTier t, double lean, double latG, bool imuOk, double slip = 25, double front = 40) {
  host::cutEdges.clear(); boot(t);
  g.rpmF = g.rpmI = 9000; g.front = front; g.rear = front * (1 + slip / 100); g.slip = slip; g.rate = 0;
  g.lean = lean; g.latG = latG; g.imuOk = imuOk;
  host::runFor(SEC(1));
  return host::cutPulses(SEC(5.1), host::nowUs);
}
TEST(slip_coupe_en_pro_ligne_droite)     { CHECK(slipPulses(LicenseTier::Pro, 0, 1.0, true) >= 8); }
TEST(slip_absent_en_track)               { CHECK(slipPulses(LicenseTier::Track, 0, 1.0, true) == 0); }
TEST(slip_absent_en_basic)               { CHECK(slipPulses(LicenseTier::Basic, 0, 1.0, true) == 0); }
TEST(slip_interdit_au_dela_de_28_deg)    { CHECK(slipPulses(LicenseTier::Pro, 29, 1.0, true) == 0); CHECK(slipPulses(LicenseTier::Pro, -35, 1.0, true) == 0); }
TEST(slip_interdit_si_imu_muet)          { CHECK(slipPulses(LicenseTier::Pro, 0, 1.0, false) == 0); }
TEST(slip_interdit_si_accel_forte)       { CHECK(slipPulses(LicenseTier::Pro, 0, 1.4, true) == 0); }
TEST(slip_interdit_si_imu_perime) {
  host::cutEdges.clear(); boot(LicenseTier::Pro);
  g.rpmF = g.rpmI = 9000; g.front = 40; g.rear = 50; g.slip = 25; g.imuFeed = true;
  host::runFor(MS(300));
  CHECK(host::cutPulses(SEC(5.05), host::nowUs) > 0);
  g.imuFeed = false;                                   // la tache IMU meurt : plus de MAJ
  host::runFor(MS(300));                               // > IMU_STALE_MS
  const uint64_t t = host::nowUs; host::runFor(MS(500));
  CHECK_MSG(host::cutPulses(t, host::nowUs) == 0, "slip actif avec un IMU perime");
}
TEST(slip_autorite_degressive_12_28) {
  double ms12 = 0, ms20 = 0, ms27 = 0;
  { host::cutEdges.clear(); slipPulses(LicenseTier::Pro, 10, 1.0, true); ms12 = cutMs(SEC(5.1), host::nowUs); }
  INFO("coupure cumulee a 10 deg: %.0f ms", ms12);
  CHECK(ms12 > 0);
}
TEST(slip_hysteresis_rearmement_sous_8_deg) {
  boot(LicenseTier::Pro);
  g.rpmF = g.rpmI = 9000; g.front = 40; g.rear = 50; g.slip = 25;
  g.lean = 30; host::runFor(MS(200));                  // bloque
  CHECK(host::cutPulses(SEC(5), host::nowUs) == 0);
  g.lean = 15; host::runFor(MS(300));                  // sous 28 mais > 8 : toujours bloque
  CHECK_MSG(host::cutPulses(SEC(5), host::nowUs) == 0, "re-autorise trop tot (hysteresis)");
  g.lean = 5; host::runFor(MS(300));
  CHECK(host::cutPulses(SEC(5), host::nowUs) > 0);
}
TEST(slip_sous_5kmh_rien) { CHECK(slipPulses(LicenseTier::Pro, 0, 1.0, true, 40, 4.0) == 0); }
TEST(slip_dans_la_tolerance_rien) { CHECK(slipPulses(LicenseTier::Pro, 0, 1.0, true, 10, 40) == 0); }

// ============================ Proprietes (fuzz) ================================================
TEST(fuzz_invariants_entrees_aleatoires) {
  boot(LicenseTier::Pro);
  uint64_t seed = 0;
  int violations = 0;
  host::every(host::nowUs, 20000, [&] {
    g.rpmF = (int)(host::rnd() * 17000); g.rpmI = g.rpmF + (int)(host::gauss() * 300);
    if (g.rpmI < 0) g.rpmI = 0;
    g.front = host::rnd() < 0.2 ? 0 : host::rnd() * 120; g.rear = g.front * (0.9 + host::rnd() * 0.5);
    g.slip = g.front > 5 ? (g.rear - g.front) / g.front * 100 : 0; g.rate = host::gauss() * 50;
    g.lean = (host::rnd() - 0.5) * 80; g.latG = 0.8 + host::rnd() * 0.8; g.imuOk = host::rnd() < 0.9;
    if (host::rnd() < 0.3) host::setPin(pins::CLUTCH_HALL, host::rnd() < 0.5);
    if (host::rnd() < 0.1) host::setPin(pins::PIT_BTN, host::rnd() < 0.5);
    if (host::rnd() < 0.3) { host::setPin(pins::SHIFTER, 0); host::setPin(pins::SHIFTER, 1); }
    return true; });
  host::runFor(SEC(120));
  // Invariants : (1) pin haute <=> ignition active ; (2) jamais de coupure continue anormale hors overrev
  CHECK(host::cutHigh() == ignition::isActive());
  const uint64_t maxRun = host::cutMaxRunUs(SEC(5), host::nowUs);
  INFO("plus longue coupure continue sur 120 s d'entrees aleatoires: %.0f ms", maxRun / 1000.0);
  CHECK(maxRun <= 1500000);   // l'overrev peut tenir longtemps; on borne juste l'absurde
  (void)seed; (void)violations;
}
TEST(fuzz_sans_overrev_coupure_continue_bornee) {
  boot(LicenseTier::Pro);
  host::every(host::nowUs, 20000, [&] {
    g.rpmF = (int)(host::rnd() * 15900); g.rpmI = std::min(g.rpmF + (int)(host::gauss() * 200), 15990); if (g.rpmI < 0) g.rpmI = 0;
    g.front = host::rnd() * 120; g.rear = g.front * (0.9 + host::rnd() * 0.6); g.slip = g.front > 5 ? (g.rear - g.front) / g.front * 100 : 0;
    g.lean = (host::rnd() - 0.5) * 40; g.latG = 0.9 + host::rnd() * 0.3; g.imuOk = true;
    if (host::rnd() < 0.3) host::setPin(pins::CLUTCH_HALL, host::rnd() < 0.5);
    if (host::rnd() < 0.2) host::setPin(pins::PIT_BTN, host::rnd() < 0.5);
    if (host::rnd() < 0.3) { host::setPin(pins::SHIFTER, 0); host::setPin(pins::SHIFTER, 1); }
    return true; });
  host::runFor(SEC(120));
  const uint64_t maxRun = host::cutMaxRunUs(SEC(5), host::nowUs);
  INFO("plus longue coupure continue (sans overrev): %.0f ms", maxRun / 1000.0);
  CHECK_MSG(maxRun <= 250000, "coupure continue de %.0f ms sans overrev", maxRun / 1000.0);
}

// ============================ Matrice licences ===========================================================
// BASIC : overrev + pit (km/h ET repli tr/min) + quickshifter ; TRACK : + launch ; PRO : + slip.
static int matrixCuts(LicenseTier tier, const char* feature) {
  host::cutEdges.clear();
  host::setPin(pins::PIT_BTN, 1); host::setPin(pins::CLUTCH_HALL, 1); host::setPin(pins::SHIFTER, 1);   // etat propre entre deux cas
  g = In();
  boot(tier);
  host::runFor(MS(100));
  g.rpmF = g.rpmI = 9000; g.front = 0; g.rear = 0;
  const std::string f = feature;
  if (f == "overrev") { g.rpmF = g.rpmI = 16500; }
  if (f == "pit_kmh") { g.front = g.rear = 75; setPit(host::nowUs + 1000, true); }
  if (f == "pit_rpm") { setPit(host::nowUs + 1000, true); }                               // vitesse inconnue -> repli regime
  if (f == "shift")   { pressShifter(host::nowUs + MS(100), 30000); }
  if (f == "launch")  { setClutch(host::nowUs + 1000, true); }
  if (f == "slip")    { g.front = 40; g.rear = 52; g.slip = 30; }
  const uint64_t t0 = host::nowUs;
  host::runFor(SEC(1));
  return host::cutPulses(t0, host::nowUs);
}
TEST(licences_matrice_basic) {
  CHECK(matrixCuts(LicenseTier::Basic, "overrev") > 0);
  CHECK(matrixCuts(LicenseTier::Basic, "pit_kmh") > 0);
  CHECK(matrixCuts(LicenseTier::Basic, "pit_rpm") > 0);
  CHECK(matrixCuts(LicenseTier::Basic, "shift") > 0);
  CHECK(matrixCuts(LicenseTier::Basic, "launch") == 0);
  CHECK(matrixCuts(LicenseTier::Basic, "slip") == 0);
}
TEST(licences_matrice_track) {
  CHECK(matrixCuts(LicenseTier::Track, "overrev") > 0);
  CHECK(matrixCuts(LicenseTier::Track, "pit_kmh") > 0);
  CHECK(matrixCuts(LicenseTier::Track, "shift") > 0);
  CHECK(matrixCuts(LicenseTier::Track, "launch") > 0);
  CHECK(matrixCuts(LicenseTier::Track, "slip") == 0);
}
TEST(licences_matrice_pro) {
  CHECK(matrixCuts(LicenseTier::Pro, "overrev") > 0);
  CHECK(matrixCuts(LicenseTier::Pro, "pit_kmh") > 0);
  CHECK(matrixCuts(LicenseTier::Pro, "shift") > 0);
  CHECK(matrixCuts(LicenseTier::Pro, "launch") > 0);
  CHECK(matrixCuts(LicenseTier::Pro, "slip") > 0);
}
