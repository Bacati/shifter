// SRC: core/SharedState.cpp sensors/WaterTemp.cpp
#include "host.h"
#include "mock/Arduino.h"
#include "mock/DallasTemperature.h"
#include "src/core/Config.h"
#include "src/core/SharedState.h"
#include "src/sensors/WaterTemp.h"

static void boot(std::function<float(uint64_t)> temp) {
  host::nowUs = SEC(2);
  state::begin();
  host::ds.temp = temp;
  watertemp::begin();
  watertemp::startTask();
}
struct T { double c; bool ok; };
static T read() { int x; bool ok; state::getWaterTemp(x, ok); return {x / 10.0, ok}; }

TEST(temp_lecture_normale_et_precision) {
  boot([](uint64_t) { return 92.3f; });
  host::runFor(SEC(3));
  CHECK(read().ok);
  CHECK_NEAR(read().c, 92.3, 0.11);
}
TEST(temp_conversion_non_bloquante_cycle_1s) {
  boot([](uint64_t) { return 60.0f; });
  host::runFor(SEC(30));
  INFO("requetes=%u lectures=%u attente bloquante=%d ms", host::ds.requests, host::ds.reads, host::ds.reportBlockingMs);
  CHECK_MSG(host::ds.reportBlockingMs == 0, "la bibliotheque bloque encore 750 ms par lecture");
  CHECK_MSG(host::ds.requests >= 27 && host::ds.requests <= 31, "cycle != 1 s : %u lectures en 30 s", host::ds.requests);
}
TEST(temp_suit_une_rampe) {
  boot([](uint64_t t) { return (float)(20.0 + (t / 1e6 - 2) * 1.5); });   // +1.5 C/s
  host::runFor(SEC(61));
  double truth = 20.0 + (host::nowUs / 1e6 - 2) * 1.5;
  INFO("lu %.1f vrai %.1f", read().c, truth);
  CHECK(read().ok);
  CHECK(std::fabs(read().c - truth) < 3.0);
}
TEST(temp_85C_d_usine_isole_est_rejete) {
  // La sonde renvoie 85.0 (conversion non faite) une fois au milieu d'une serie a 60 C.
  int n = 0;
  boot([&](uint64_t t) { return (t > SEC(10) && t < SEC(13)) ? 85.0f : 60.0f; });
  host::runFor(SEC(9));
  CHECK_NEAR(read().c, 60.0, 0.2);
  double maxSeen = 0;
  host::every(host::nowUs, MS(50), [&] { if (read().ok) maxSeen = std::max(maxSeen, read().c); return host::nowUs < SEC(18); });
  host::runFor(SEC(9)); (void)n;
  INFO("max affiche = %.1f", maxSeen);
  CHECK_MSG(maxSeen < 70.0, "85 C d'usine affiche comme vraie temperature (%.1f)", maxSeen);
}
TEST(temp_85C_reel_accepte_si_continuite) {
  boot([](uint64_t t) { double s = t / 1e6 - 2; return (float)(80.0 + std::min(s, 5.0) * 1.0); });   // monte de 80 a 85 puis reste a 85.0
  host::runFor(SEC(20));
  INFO("lu %.1f", read().c);
  CHECK(read().ok);
  CHECK_NEAR(read().c, 85.0, 0.2);
}
TEST(temp_echec_isole_tolere) {
  boot([](uint64_t) { return 70.0f; });
  host::runFor(SEC(5));
  host::ds.failReads = 1;                           // un CRC rate
  host::runFor(MS(1100));
  CHECK_MSG(read().ok && std::fabs(read().c - 70.0) < 0.2, "un echec isole ne doit pas faire perdre la mesure");
  host::runFor(SEC(5));
  CHECK(read().ok);
}
TEST(temp_echecs_repetes_declarent_absent_puis_recuperation) {
  boot([](uint64_t) { return 70.0f; });
  host::runFor(SEC(5));
  host::ds.present = false;
  host::runFor(SEC(5));
  CHECK_MSG(!read().ok, "sonde debranchee non detectee");
  host::ds.present = true;
  host::runFor(SEC(8));
  CHECK_MSG(read().ok && std::fabs(read().c - 70.0) < 0.2, "sonde rebranchee non retrouvee");
}
TEST(temp_hors_plage_rejetee) {
  boot([](uint64_t) { return 175.0f; });
  host::runFor(SEC(8));
  CHECK(!read().ok);
  host::ds.temp = [](uint64_t) { return -45.0f; };
  host::runFor(SEC(8));
  CHECK(!read().ok);
}
TEST(temp_absente_au_boot_puis_branchee) {
  host::ds.present = false;
  boot([](uint64_t) { return 55.0f; });
  host::runFor(SEC(5));
  CHECK(!read().ok);
  host::ds.present = true;
  host::runFor(SEC(6));
  CHECK(read().ok);
  CHECK_NEAR(read().c, 55.0, 0.2);
}
TEST(temp_valeur_perimee_invalidee_par_la_fraicheur) {
  host::nowUs = SEC(2); state::begin();
  state::setWaterTemp(900, true);
  host::runFor(MS(2900));  CHECK(read().ok);
  host::runFor(MS(300));   CHECK_MSG(!read().ok, "valeur de plus de WATER_TEMP_STALE_MS encore donnee valide");
}
TEST(temp_negative_et_zero) {
  boot([](uint64_t) { return -5.5f; });
  host::runFor(SEC(4));
  CHECK(read().ok); CHECK_NEAR(read().c, -5.5, 0.11);
  host::ds.temp = [](uint64_t) { return 0.0f; };
  host::runFor(SEC(4));
  CHECK(read().ok); CHECK_NEAR(read().c, 0.0, 0.11);
}
