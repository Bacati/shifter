// SRC: core/SharedState.cpp sensors/Imu.cpp
#include "host.h"
#include "mock/Arduino.h"
#include "mock/Wire.h"
#include "src/core/Config.h"
#include "src/core/SharedState.h"
#include "src/sensors/Imu.h"
#include "common_ride.h"

static void bootImu(bool begin = true) {
  host::nowUs = SEC(2);
  state::begin();
  if (begin) imu::begin();
  imu::startTask();
}
struct Lean { double roll; double latG; bool ok; };
static Lean lean() { int r, l; bool ok; state::getLean(r, l, ok); return {r / 10.0, l / 100.0, ok}; }

static Ride constant(double deg, double v = 16.0) {
  Ride r; r.rollDeg = [=](uint64_t) { return deg; }; r.speedMs = [=](uint64_t) { return v; }; r.longG = [](uint64_t) { return 0.0; };
  return r;
}

TEST(imu_droit_a_l_arret_valide_et_zero) {
  bootImu(); installRide(constant(0));
  host::runFor(SEC(3));
  Lean l = lean();
  INFO("roll=%.2f latG=%.2f ok=%d", l.roll, l.latG, l.ok);
  CHECK(l.ok);
  CHECK_NEAR(l.roll, 0.0, 1.0);
  CHECK_NEAR(l.latG, 1.0, 0.05);
}

TEST(imu_biais_gyro_eleve_pas_de_derive) {
  bootImu(); host::mpu.gyroBiasDps[0] = 4.0f; installRide(constant(0));
  host::runFor(SEC(60));
  CHECK_NEAR(lean().roll, 0.0, 1.5);
}

TEST(imu_virage_stabilise_20deg_garde_l_angle) {
  // LE bug critique : en virage stabilise l'accelero "voit a plat". L'estimation doit rester a 20 deg.
  bootImu();
  Ride r; r.speedMs = [](uint64_t) { return 16.7; }; r.longG = [](uint64_t) { return 0.0; };
  r.rollDeg = [](uint64_t t) { double s = (t - SEC(2)) / 1e6; if (s < 4) return 0.0; if (s < 5) return 20.0 * (s - 4); return 20.0; };
  installRide(r);
  host::runFor(SEC(5.5));
  double worst = 0;
  for (int i = 0; i < 60; ++i) { host::runFor(MS(100)); worst = std::max(worst, std::fabs(lean().roll - 20.0)); }
  INFO("erreur max pendant 6 s de virage stabilise a 20 deg: %.2f", worst);
  CHECK_MSG(worst < 3.0, "angle estime derive de %.1f deg en virage stabilise", worst);
  CHECK(lean().ok);
}

static void steadyTurn(double a) {
  bootImu();
  Ride r; r.speedMs = [](uint64_t) { return 14.0; }; r.longG = [](uint64_t) { return 0.0; };
  r.rollDeg = [=](uint64_t t) { double s = (t - SEC(2)) / 1e6; if (s < 3) return 0.0; if (s < 3.8) return a * (s - 3) / 0.8; return a; };
  installRide(r);
  host::runFor(SEC(8));
  CHECK_MSG(std::fabs(lean().roll - a) < 3.0, "lean reel %.0f estime %.1f", a, lean().roll);
  CHECK(lean().ok);
}
TEST(imu_virage_stabilise_10deg) { steadyTurn(10); }
TEST(imu_virage_stabilise_15deg) { steadyTurn(15); }
TEST(imu_virage_stabilise_25deg) { steadyTurn(25); }
TEST(imu_virage_stabilise_28deg) { steadyTurn(28); }
TEST(imu_virage_stabilise_35deg) { steadyTurn(35); }

TEST(imu_chicane_s_erreur_bornee) {
  bootImu();
  Ride r; r.speedMs = [](uint64_t) { return 15.0; }; r.longG = [](uint64_t) { return 0.0; };
  r.rollDeg = [](uint64_t t) { double s = (t - SEC(2)) / 1e6; if (s < 3) return 0.0; return 35.0 * std::sin(2 * M_PI * (s - 3) / 2.4); };
  installRide(r);
  host::runFor(SEC(3.5));
  double worst = 0;
  for (int i = 0; i < 400; ++i) {
    host::runFor(MS(20));
    uint64_t t = host::nowUs; double truth = r.rollDeg(t);
    worst = std::max(worst, std::fabs(lean().roll - truth));
  }
  INFO("erreur max dans une chicane 35 deg / 2.4 s: %.1f deg", worst);
  CHECK_MSG(worst < 6.0, "erreur %.1f deg", worst);
}

TEST(imu_bascule_rapide_0_a_40deg_en_600ms) {
  bootImu();
  Ride r; r.speedMs = [](uint64_t) { return 20.0; }; r.longG = [](uint64_t) { return 0.0; };
  r.rollDeg = [](uint64_t t) { double s = (t - SEC(2)) / 1e6; if (s < 3) return 0.0; if (s < 3.6) return 40.0 * (s - 3) / 0.6; return 40.0; };
  installRide(r);
  host::runFor(SEC(4.5));
  CHECK_NEAR(lean().roll, 40.0, 4.0);
}

TEST(imu_reboot_dans_un_virage_reste_invalide_jusqu_a_etre_droit) {
  // L'ECU redemarre (superviseur) alors que la moto est couchee a 20 deg : l'angle initial 0 est faux.
  bootImu();
  Ride r; r.speedMs = [](uint64_t) { return 16.7; }; r.longG = [](uint64_t) { return 0.0; };
  r.rollDeg = [](uint64_t t) { double s = (t - SEC(2)) / 1e6; if (s < 6) return 20.0; if (s < 7) return 20.0 * (7 - s); return 0.0; };
  installRide(r);
  int okInCorner = 0;
  host::every(SEC(3.0), MS(10), [&] { if (host::nowUs < SEC(8) && lean().ok) ++okInCorner; return host::nowUs < SEC(8); });
  host::runFor(SEC(5.5));                       // jusqu'a t=7.5 s
  CHECK_MSG(okInCorner == 0, "angle declare fiable %d fois alors que la moto est couchee et l'estimation fausse", okInCorner);
  host::runFor(SEC(4));
  CHECK(lean().ok);
  CHECK_NEAR(lean().roll, 0.0, 1.5);
}

TEST(imu_capteur_endormi_apres_brownout_detecte_puis_reinitialise) {
  bootImu(); installRide(constant(0));
  host::runFor(SEC(3));
  CHECK(lean().ok);
  host::mpu.sleepBit = true;                    // brownout : le MPU6050 retombe en sleep (registres a 0)
  host::runFor(MS(150));
  CHECK_MSG(!lean().ok, "capteur endormi non detecte");
  host::runFor(SEC(4));                         // re-init (500 ms) + biais (800 ms) + validation
  CHECK_MSG(host::mpu.resets == 0 && !host::mpu.sleepBit, "le capteur n'a pas ete reveille");
  CHECK(lean().ok);
  CHECK(host::serialContains("(re)initialise"));
}

TEST(imu_trame_figee_detectee) {
  bootImu(); installRide(constant(0));
  host::runFor(SEC(3));
  CHECK(lean().ok);
  host::mpu.frozen = true;
  host::runFor(MS(300));
  uint32_t last = state::lastLeanMs();
  CHECK_MSG(!lean().ok || (host::nowUs / 1000 - last) > cfg::IMU_STALE_MS, "trames figees non detectees");
  host::mpu.frozen = false;
  host::runFor(SEC(4));
  CHECK(lean().ok);
}

TEST(imu_bus_bloque_puis_debloque) {
  bootImu(); installRide(constant(0));
  host::runFor(SEC(3));
  host::mpu.busStuck = true;
  host::runFor(MS(120));
  CHECK_MSG(!lean().ok, "bus I2C bloque non detecte");
  host::runFor(SEC(2));
  host::mpu.busStuck = false;
  host::runFor(SEC(4));
  CHECK(lean().ok);
}

TEST(imu_absent_au_boot_branche_plus_tard) {
  host::mpu.present = false;
  bootImu(false);                               // imu::begin() echoue
  CHECK(!imu::begin());
  host::runFor(SEC(2));
  CHECK(!lean().ok);
  host::mpu.present = true; installRide(constant(0));
  host::runFor(SEC(5));
  CHECK_MSG(lean().ok, "IMU branche apres le boot jamais detecte");
}

TEST(imu_vibrations_moteur_pas_de_blocage_dur_parasite) {
  bootImu(); host::mpu.accNoiseG = 0.12f; host::mpu.gyroNoiseDps = 1.5f; installRide(constant(0));
  host::runFor(SEC(3));
  int hard = 0, total = 0;
  for (int i = 0; i < 1000; ++i) { host::runFor(MS(5)); if (lean().latG > cfg::LEAN_LATERAL_G_HARD) ++hard; ++total; }
  INFO("blocages durs sous vibrations 0.12 g: %d/%d", hard, total);
  CHECK(hard < total / 100);
  CHECK(lean().ok);
  CHECK_NEAR(lean().roll, 0.0, 2.5);
}

TEST(imu_freinage_fort_accel_longitudinale) {
  bootImu();
  Ride r = constant(0); r.longG = [](uint64_t t) { return t > SEC(5) ? -1.0 : 0.0; };
  installRide(r);
  host::runFor(SEC(6));
  Lean l = lean();
  INFO("freinage 1 g: latG=%.2f roll=%.1f", l.latG, l.roll);
  CHECK(l.latG > cfg::LEAN_LATERAL_G_HARD);     // blocage dur du slip (documente: virage/freinage fort)
  CHECK_NEAR(l.roll, 0.0, 3.0);                 // l'angle ne doit pas deriver
}

TEST(imu_orientation_negative_gauche) {
  bootImu();
  Ride r; r.speedMs = [](uint64_t) { return 16.0; }; r.longG = [](uint64_t) { return 0.0; };
  r.rollDeg = [](uint64_t t) { double s = (t - SEC(2)) / 1e6; if (s < 3) return 0.0; if (s < 3.8) return -25.0 * (s - 3) / 0.8; return -25.0; };
  installRide(r);
  host::runFor(SEC(8));
  CHECK_NEAR(lean().roll, -25.0, 3.0);
}

// Trajet long, sans panne : l'erreur d'angle, QUAND l'IMU se declare fiable, doit rester bornee.
static void longRide(double speedMin, double durS, double* worstOut, int* okCountOut) {
  if (std::getenv("IMU_IDEAL")) { host::mpu.gyroBiasDps[0] = 0; host::mpu.gyroNoiseDps = 0; host::mpu.accNoiseG = 0; }
  bootImu();
  static double v = 10;
  Ride r; r.longG = [](uint64_t) { return 0.0; };
  r.speedMs = [=](uint64_t t) { return std::max(speedMin, 8.0 + 6.0 * std::sin(t / 1e6 / 5.3) + 5.0 * std::sin(t / 1e6 / 1.9)); };
  r.rollDeg = [](uint64_t t) { double s = t / 1e6; return 40.0 * std::sin(s / 4.1) * std::sin(s / 13.0); };
  // 12 s a l'arret (demarrage au stand), puis le trajet : comme sur une vraie moto
  auto rr = r;
  r.rollDeg = [rr](uint64_t t) { return t < SEC(12) ? 0.0 : rr.rollDeg(t - SEC(12)); };
  r.speedMs = [rr](uint64_t t) { return t < SEC(12) ? 0.0 : rr.speedMs(t - SEC(12)); };
  installRide(r);
  host::every(host::nowUs, MS(20), [r] { int kmh10 = (int)(r.speedMs(host::nowUs) * 36.0); state::setSpeeds(kmh10, kmh10); return true; });
  double worst = 0; int okc = 0, total = 0;
  host::every(SEC(20), MS(20), [&] {
    int rl, lt; bool ok; state::getLean(rl, lt, ok);
    ++total; if (ok) { ++okc; double e = std::fabs(rl / 10.0 - r.rollDeg(host::nowUs)); worst = std::max(worst, e);
      if (std::getenv("IMU_TRACE") && total % 25 == 0) printf("TR t=%.1f vrai=%.1f est=%.1f err=%.1f v=%.1f\n", host::nowUs / 1e6, r.rollDeg(host::nowUs), rl / 10.0, e, r.speedMs(host::nowUs)); }
    return true; });
  host::runFor(SEC(durS));
  (void)v; *worstOut = worst; *okCountOut = okc * 100 / std::max(1, total);
}
TEST(imu_trajet_long_erreur_bornee_quand_fiable) {
  double worst; int okPct; longRide(3.0, 600, &worst, &okPct);
  INFO("erreur max quand ok=1 : %.1f deg ; ok %d %% du temps", worst, okPct);
  CHECK_MSG(worst < 8.0, "erreur d'angle de %.1f deg alors que l'IMU se declare fiable", worst);
}
TEST(imu_trajet_long_vitesse_elevee_erreur_bornee) {
  double worst; int okPct; longRide(15.0, 600, &worst, &okPct);
  INFO("erreur max quand ok=1 : %.1f deg ; ok %d %% du temps", worst, okPct);
  CHECK_MSG(worst < 8.0, "erreur d'angle de %.1f deg alors que l'IMU se declare fiable", worst);
}

TEST(imu_biais_non_mesure_en_roulant_ok_faux_jusqu_a_l_arret) {
  bootImu(); installRide(constant(0));
  state::setSpeeds(450, 450);                  // la moto roule des le demarrage de l'IMU
  host::runFor(SEC(10));
  CHECK_MSG(!lean().ok, "angle declare fiable alors que le biais gyro n'a jamais ete mesure au calme");
  state::setSpeeds(0, 0);
  host::runFor(SEC(4));
  CHECK(lean().ok);
}
