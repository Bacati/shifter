// SRC: core/SharedState.cpp sensors/Sensors.cpp actuator/IgnitionCut.cpp control/Arbiter.cpp persistence/License.cpp persistence/Storage.cpp sim/SimEngine.cpp
#include "common.h"

static void boot(uint64_t t0 = SEC(5)) {
  host::nowUs = t0;
  state::begin();
  state::setConfig(cfg::DEFAULTS);
  sensors::begin();
  startEcuLoop(100);
}
static int wheelKmhX10(bool front) { int f, r; state::getSpeeds(f, r); return front ? f : r; }

// ============================ RPM =====================================
TEST(rpm_precision_multi) {
  // Une seule boucle pour eviter de relancer le firmware : on change la consigne en cours de route.
  boot();
  double target = 1500;
  auto g = train(pins::RPM, [&](uint64_t) { return rpmHz(target); });
  for (int rpm = 1500; rpm <= 16000; rpm += 500) {
    target = rpm;
    host::runFor(SEC(1.0));
    const int f = state::getRpmFiltered();
    CHECK_MSG(std::abs(f - rpm) <= rpm * 0.01 + 5, "rpm=%d lu=%d", rpm, f);
  }
}

TEST(rpm_avec_jitter_5pct) {
  boot();
  auto g = train(pins::RPM, [](uint64_t) { return rpmHz(9000); }, 100, 0.05);
  host::runFor(SEC(1));
  int lo = 99999, hi = 0;
  for (int i = 0; i < 200; ++i) { host::runFor(MS(10)); int v = state::getRpmFiltered(); lo = std::min(lo, v); hi = std::max(hi, v); }
  INFO("filtre sous jitter 5%%: %d..%d", lo, hi);
  CHECK(lo > 9000 * 0.88 && hi < 9000 * 1.12);
}

TEST(rpm_calage_apres_500ms) {
  boot();
  auto g = train(pins::RPM, [](uint64_t) { return rpmHz(8000); });
  host::runFor(SEC(1));
  CHECK(state::getRpmFiltered() > 7900);
  g->stop = true;
  host::runFor(MS(400));
  CHECK_MSG(state::getRpmFiltered() > 0, "ne doit pas etre a 0 avant RPM_STALL_US");
  host::runFor(MS(150));
  CHECK_MSG(state::getRpmFiltered() == 0 && state::getRpmInstant() == 0, "doit retomber a 0 (moteur cale)");
  CHECK(sensors::engineStalled());
}

TEST(rpm_impulsion_manquee_rattrape) {
  boot();
  auto g = train(pins::RPM, [](uint64_t) { return rpmHz(8000); }, 100, 0.0, 0.10);   // 10% d'impulsions perdues
  host::runFor(SEC(1));
  int lo = 99999, hi = 0;
  for (int i = 0; i < 500; ++i) { host::runFor(MS(5)); int v = state::getRpmInstant(); lo = std::min(lo, v); hi = std::max(hi, v); }
  INFO("instant avec 10%% d'impulsions manquees: %d..%d", lo, hi);
  CHECK_MSG(hi <= cfg::OVERREV_RPM, "ne doit jamais provoquer un overrev (max=%d)", hi);
  CHECK_MSG(lo > 3500, "ne doit pas se figer a la moitie (min=%d)", lo);
}

TEST(rpm_rampe_acceleration_suivie) {
  boot();
  auto g = train(pins::RPM, [](uint64_t t) {
    double s = (t - SEC(5)) / 1e6; double r = 3000 + (s < 1 ? 0 : std::min(11000.0, (s - 1) * 22000)); return rpmHz(r); });
  host::runFor(SEC(1));
  host::runFor(MS(500));   // rampe de 3000 -> 14000 sur 0.5 s (22000 tr/min/s)
  int v = state::getRpmInstant();
  INFO("instant a la fin de la rampe: %d (reel 14000)", v);
  CHECK(v > 12500 && v <= 14500);
}

TEST(rpm_redemarrage_apres_calage) {
  boot();
  auto g1 = train(pins::RPM, [](uint64_t) { return rpmHz(8000); });
  host::runFor(SEC(1)); g1->stop = true; host::runFor(SEC(3));
  CHECK(state::getRpmFiltered() == 0);
  auto g2 = train(pins::RPM, [](uint64_t) { return rpmHz(900); });
  host::runFor(SEC(2));
  int v = state::getRpmFiltered();
  INFO("apres redemarrage a 900: %d", v);
  CHECK_NEAR(v, 900, 60);
}

TEST(rpm_rejette_impulsions_trop_rapprochees) {
  // 25 000 tr/min impossible : le filtre ne doit jamais depasser RPM_MAX_PLAUSIBLE.
  boot();
  auto g = train(pins::RPM, [](uint64_t) { return rpmHz(25000); });
  host::runFor(SEC(1));
  int i = state::getRpmInstant();
  INFO("25000 tr/min injectes -> instant=%d", i);
  CHECK(i <= cfg::RPM_MAX_PLAUSIBLE);
}

TEST(rpm_rafale_emi_dense_ne_perturbe_pas) {
  // Signal valide 8000 tr/min + rafales de parasites (espaces de 5..80 us) a des instants aleatoires.
  boot();
  auto g = train(pins::RPM, [](uint64_t) { return rpmHz(8000); });
  auto burst = std::make_shared<std::function<void()>>();
  *burst = [=]() {
    uint64_t t = host::nowUs;
    for (int i = 0; i < 25; ++i) { t += 5 + (uint64_t)(host::rnd() * 75); host::pulse(pins::RPM, t, 3); }
    host::at(host::nowUs + 3000 + (uint64_t)(host::rnd() * 40000), *burst);
  };
  host::at(host::nowUs + 10000, *burst);
  host::runFor(SEC(1));
  int lo = 99999, hi = 0, spurious = 0;
  for (int i = 0; i < 1000; ++i) {
    host::runFor(MS(2)); int v = state::getRpmInstant(); lo = std::min(lo, v); hi = std::max(hi, v);
    if (v > cfg::OVERREV_RPM) ++spurious;
  }
  INFO("instant sous EMI dense: %d..%d, depassements overrev=%d", lo, hi, spurious);
  CHECK_MSG(spurious == 0, "parasite pris pour overrev");
  // Un parasite accepte a mi-periode fausse une mesure (+/- 25 % ici) mais la pente max et le EWMA
  // l'empechent d'atteindre l'overrev ; mesure documentee dans AUDIT.md.
  CHECK_MSG(lo > 5500 && hi < 11000, "RPM perturbe: %d..%d", lo, hi);
}

TEST(rpm_emi_seul_sans_signal_retombe_a_zero) {
  // Moteur coupe, il ne reste que des rafales de parasites (chacune = quelques dizaines de pics de
  // quelques us) espacees de 20..60 ms : pas de "moteur fantome".
  boot();
  auto g = train(pins::RPM, [](uint64_t) { return rpmHz(8000); });
  host::runFor(SEC(1));
  CHECK(state::getRpmFiltered() > 7500);
  g->stop = true;
  auto burst = std::make_shared<std::function<void()>>();
  *burst = [=]() {
    uint64_t t = host::nowUs;
    for (int i = 0; i < 40; ++i) { t += 5 + (uint64_t)(host::rnd() * 60); host::pulse(pins::RPM, t, 3); }
    host::at(host::nowUs + 20000 + (uint64_t)(host::rnd() * 40000), *burst);
  };
  host::at(host::nowUs + 1000, *burst);
  host::runFor(SEC(1.5));
  int v = state::getRpmFiltered();
  INFO("rpm apres 1.5 s de rafales sans signal: %d", v);
  CHECK_MSG(v == 0, "moteur fantome: %d", v);
}
TEST(rpm_bruit_continu_apres_arret_limite_logicielle_documentee) {
  // Bruit CONTINU (> 300 impulsions/s) apres l'arret du moteur : aucun filtre logiciel ne peut le
  // distinguer d'un moteur a ~19 000 tr/min. Ce test MESURE seulement (le filtre materiel est requis).
  boot();
  auto g = train(pins::RPM, [](uint64_t) { return rpmHz(8000); });
  host::runFor(SEC(1)); g->stop = true;
  auto burst = std::make_shared<std::function<void()>>();
  *burst = [=]() { uint64_t t = host::nowUs; for (int i = 0; i < 40; ++i) { t += 5 + (uint64_t)(host::rnd() * 60); host::pulse(pins::RPM, t, 3); }
                   host::at(host::nowUs + 100 + (uint64_t)(host::rnd() * 2000), *burst); };
  host::at(host::nowUs + 1000, *burst);
  int ghost = 0, overrev = 0;
  for (int i = 0; i < 300; ++i) { host::runFor(MS(10)); if (state::getRpmFiltered() > 0) ++ghost; if (state::getRpmInstant() > cfg::OVERREV_RPM) ++overrev; }
  INFO("bruit continu apres arret: moteur fantome %d/300 lectures, instant > overrev %d/300", ghost, overrev);
}
TEST(rpm_emi_synchrone_a_phase_defavorable) {
  // CAS LIMITE: un parasite 3.1 ms APRES chaque vraie impulsion (hors du rejet 3 ms).
  boot();
  auto g = train(pins::RPM, [](uint64_t) { return rpmHz(8000); });
  auto g2 = train(pins::RPM, [](uint64_t) { return rpmHz(8000); }, 3, 0.0, 0.0, host::nowUs + 3100);
  host::runFor(SEC(2));
  int v = state::getRpmInstant();
  INFO("parasite periodique a +3.1 ms -> instant=%d (vrai 8000)", v);
  CHECK_MSG(v < cfg::OVERREV_RPM, "overrev declenche par un parasite periodique: %d", v);
}

TEST(rpm_debordement_micros_32bits) {
  host::baseUs = 0xFFFFFFFFull - SEC(5) - 500000;       // micros() deborde pendant le test
  boot();
  auto g = train(pins::RPM, [](uint64_t) { return rpmHz(7000); });
  int bad = 0;
  for (int i = 0; i < 400; ++i) { host::runFor(MS(10)); if (host::nowUs > SEC(6) && std::abs(state::getRpmFiltered() - 7000) > 150) ++bad; }
  CHECK_MSG(bad == 0, "%d lectures fausses autour du debordement micros()", bad);
}

// ============================ Roues ===================================
TEST(roues_vitesse_precise) {
  boot();
  double kmh = 20;
  auto f = train(pins::WHEEL_FRONT, [&](uint64_t) { return wheelHz(kmh, 1800, 3); });
  auto r = train(pins::WHEEL_REAR,  [&](uint64_t) { return wheelHz(kmh, 1800, 3); });
  for (kmh = 10; kmh <= 120; kmh += 10) {
    host::runFor(SEC(1.2));
    double vf = wheelKmhX10(true) / 10.0, vr = wheelKmhX10(false) / 10.0;
    CHECK_MSG(std::fabs(vf - kmh) < kmh * 0.03 + 0.5, "avant: %g km/h lu %g", kmh, vf);
    CHECK_MSG(std::fabs(vr - kmh) < kmh * 0.03 + 0.5, "arriere: %g km/h lu %g", kmh, vr);
  }
}

TEST(roues_arret_detecte_et_decroissance_continue) {
  boot();
  auto f = train(pins::WHEEL_FRONT, [](uint64_t) { return wheelHz(40, 1800, 3); });
  host::runFor(SEC(1));
  CHECK(wheelKmhX10(true) > 380);
  f->stop = true; host::runFor(MS(600));
  INFO("600 ms apres le dernier front: %.1f km/h", wheelKmhX10(true) / 10.0);
  CHECK_MSG(wheelKmhX10(true) < 40, "l'estimation doit decroitre (borne k/dt), lu %d", wheelKmhX10(true));
  host::runFor(SEC(1));
  CHECK_MSG(wheelKmhX10(true) == 0, "vitesse non nulle apres arret: %d", wheelKmhX10(true));
}

TEST(roues_glisse_calculee) {
  boot();
  auto f = train(pins::WHEEL_FRONT, [](uint64_t) { return wheelHz(60, 1800, 3); });
  auto r = train(pins::WHEEL_REAR,  [](uint64_t) { return wheelHz(66, 1800, 3); });
  host::runFor(SEC(2));
  int s, sr; state::getSlip(s, sr);
  INFO("glisse=%.1f %% (attendu 10)", s / 10.0);
  CHECK_NEAR(s / 10.0, 10.0, 1.5);
}

TEST(roues_pas_de_fausse_glisse_a_vitesse_egale_avec_jitter) {
  // Vraie moto: les 2 capteurs ont du jitter (1..3 %). Ne doit pas fabriquer de glisse > 8 %.
  boot();
  auto f = train(pins::WHEEL_FRONT, [](uint64_t) { return wheelHz(50, 1800, 3); }, 100, 0.02);
  auto r = train(pins::WHEEL_REAR,  [](uint64_t) { return wheelHz(50, 1800, 3); }, 100, 0.02);
  host::runFor(SEC(1));
  double worst = 0;
  for (int i = 0; i < 500; ++i) { host::runFor(MS(10)); int s, sr; state::getSlip(s, sr); worst = std::max(worst, std::fabs(s / 10.0)); }
  INFO("glisse parasite max = %.1f %%", worst);
  // marge necessaire : glisse parasite + terme predictif (4 %) doit rester sous la tolerance par defaut
  CHECK_MSG(worst + cfg::SLIP_PREDICT_CLAMP_PCT < cfg::DEFAULTS.maxSlipPercent,
            "fausse glisse %.1f %% + predictif %.0f %% >= tolerance %d %%", worst, (double)cfg::SLIP_PREDICT_CLAMP_PCT, cfg::DEFAULTS.maxSlipPercent);
}

TEST(roues_glitch_impulsion_doublee_borne_et_transitoire) {
  boot();
  auto f = train(pins::WHEEL_FRONT, [](uint64_t) { return wheelHz(30, 1800, 3); });
  host::runFor(SEC(1.5));
  host::pulse(pins::WHEEL_FRONT, host::nowUs + 300, 50);          // UNE impulsion parasite
  int worst = 0;
  for (int i = 0; i < 20; ++i) { host::runFor(MS(10)); worst = std::max(worst, wheelKmhX10(true)); }
  INFO("pic apres 1 glitch: %.1f km/h (vrai 30)", worst / 10.0);
  CHECK_MSG(worst <= 30 * 10 + cfg::WHEEL_SPEED_SLEW_KMH * 10 + 20, "pic %d > vrai + pente max", worst);
  host::runFor(MS(300));
  CHECK_NEAR(wheelKmhX10(true) / 10.0, 30.0, 2.0);                  // retour a la normale
}

TEST(roues_basse_vitesse_5kmh) {
  boot();
  auto f = train(pins::WHEEL_FRONT, [](uint64_t) { return wheelHz(5, 1800, 3); });
  host::runFor(SEC(3));
  double v = wheelKmhX10(true) / 10.0;
  INFO("5 km/h -> %.2f", v);
  CHECK_NEAR(v, 5.0, 1.0);
}

TEST(roues_changement_config_ppr_circonference) {
  boot();
  EcuConfig c = cfg::DEFAULTS; c.wheelCircumMm = 2000; c.pulsesPerRev = 4; state::setConfig(c);
  auto f = train(pins::WHEEL_FRONT, [](uint64_t) { return wheelHz(80, 2000, 4); });
  host::runFor(SEC(2));
  CHECK_NEAR(wheelKmhX10(true) / 10.0, 80.0, 3.0);
}

// ============================ Entrees TOR ===============================
TEST(entrees_antirebond_pit_embrayage) {
  boot();
  host::runFor(MS(50));
  CHECK(!sensors::pitButton());
  // rebonds pendant 8 ms puis stable
  for (int i = 0; i < 8; ++i) { host::setPin(pins::PIT_BTN, i % 2); host::runFor(MS(1)); }
  host::setPin(pins::PIT_BTN, 0);
  host::runFor(MS(10));
  CHECK_MSG(!sensors::pitButton(), "ne doit pas etre pris en compte avant INPUT_DEBOUNCE_MS");
  host::runFor(MS(10));
  CHECK(sensors::pitButton());
  host::setPin(pins::PIT_BTN, 1); host::runFor(MS(30));
  CHECK(!sensors::pitButton());
  host::setPin(pins::CLUTCH_HALL, 0); host::runFor(MS(30));
  CHECK(sensors::clutchPulled());
}

TEST(entrees_shifter_rebonds_un_seul_evenement) {
  boot();
  host::runFor(MS(100));
  // vrai appui : 6 rebonds sur ~4 ms puis la broche reste BASSE
  for (int i = 0; i < 6; ++i) { host::setPin(pins::SHIFTER, 0); host::runFor(MS(0.3)); host::setPin(pins::SHIFTER, 1); host::runFor(MS(0.3)); }
  host::setPin(pins::SHIFTER, 0);
  CHECK(sensors::peekShift().pending);
  sensors::consumeShift();
  host::runFor(MS(10));
  // nouveaux rebonds < 30 ms apres le 1er evenement : ignores par l'ISR
  host::setPin(pins::SHIFTER, 1);
  for (int i = 0; i < 6; ++i) { host::setPin(pins::SHIFTER, 0); host::runFor(MS(0.3)); host::setPin(pins::SHIFTER, 1); host::runFor(MS(0.3)); }
  CHECK_MSG(!sensors::peekShift().pending, "rebonds dans les 30 ms apres un evenement: doivent etre ignores");
  host::runFor(MS(50));
  host::setPin(pins::SHIFTER, 0);
  CHECK_MSG(sensors::peekShift().pending, "un nouvel appui apres 30 ms doit etre vu");
}

TEST(rpm_bruit_aleatoire_clairseme_ne_verrouille_pas) {
  // Parasites isoles a des instants aleatoires (>= 3 ms entre eux -> tous "valides" pour le filtre 1 etage).
  boot();
  auto burst = std::make_shared<std::function<void()>>();
  *burst = [=]() { host::pulse(pins::RPM, host::nowUs + 1, 5); host::at(host::nowUs + 3000 + (uint64_t)(host::rnd() * 40000), *burst); };
  host::at(host::nowUs + 1000, *burst);
  int nonzero = 0;
  for (int i = 0; i < 300; ++i) { host::runFor(MS(10)); if (state::getRpmFiltered() != 0) ++nonzero; }
  INFO("lectures non nulles sur 3 s de bruit clairseme: %d/300", nonzero);
  CHECK_MSG(nonzero < 15, "moteur fantome pendant %d/300 lectures", nonzero);
}
TEST(rpm_verrouillage_rapide_a_un_vrai_signal) {
  boot();
  auto g = train(pins::RPM, [](uint64_t) { return rpmHz(1200); });
  uint64_t t0 = host::nowUs; uint64_t tLock = 0;
  host::every(t0, 1000, [&] { if (!tLock && state::getRpmFiltered() > 0) tLock = host::nowUs - t0; return host::nowUs < t0 + SEC(2); });
  host::runFor(SEC(2));
  INFO("verrouille apres %.0f ms a 1200 tr/min", tLock / 1000.0);
  CHECK(tLock > 0 && tLock < 400000);
}
TEST(rpm_demarrage_lancer_irregulier_finit_par_verrouiller) {
  // Demarrage au kick : periodes 400 ms, 300, 220, 160, 120, puis moteur lance.
  boot();
  double per[] = {0.40, 0.30, 0.22, 0.16, 0.12, 0.09, 0.075, 0.07, 0.07, 0.07, 0.07, 0.07};
  uint64_t t = host::nowUs + 1000;
  for (double p : per) { host::pulse(pins::RPM, t, 100); t += (uint64_t)(p * 1e6); }
  host::runFor(SEC(2));
  INFO("apres demarrage au kick: %d tr/min", state::getRpmFiltered());
  CHECK(state::getRpmFiltered() > 600);
}
TEST(rpm_bruit_continu_dense_ne_verrouille_pas) {
  boot();
  auto burst = std::make_shared<std::function<void()>>();
  *burst = [=]() { uint64_t t = host::nowUs; for (int i = 0; i < 40; ++i) { t += 5 + (uint64_t)(host::rnd() * 60); host::pulse(pins::RPM, t, 3); }
                   host::at(host::nowUs + 100 + (uint64_t)(host::rnd() * 2000), *burst); };
  host::at(host::nowUs + 1000, *burst);
  host::runFor(SEC(3));
  CHECK_MSG(state::getRpmFiltered() == 0 && state::getRpmInstant() == 0, "moteur fantome %d", state::getRpmFiltered());
}
