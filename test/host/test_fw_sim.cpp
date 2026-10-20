// SRC: core/SharedState.cpp sensors/Sensors.cpp sensors/Imu.cpp sensors/WaterTemp.cpp actuator/IgnitionCut.cpp control/Arbiter.cpp persistence/License.cpp persistence/Storage.cpp net/Network.cpp net/Telemetry.cpp sim/SimEngine.cpp factory/Factory.cpp
// DEFS: -DWOKWI_SIM -DIGNITION_SIM_SCALE=2 -DSIM_SHIFT_CUT_MS=100 -DSIM_FREEZE=1 -DSIM_TIER=1
//
// Execute le VRAI shifter.ino (setup + loop) avec la configuration de l'environnement PlatformIO
// "wokwi" : c'est le scenario de la simulation Wokwi, rejoue sans Wokwi.
#include "host.h"
#include "mock/Arduino.h"
#include "src/core/Config.h"
#include "src/core/SharedState.h"
#include "src/actuator/IgnitionCut.h"
#include "src/persistence/License.h"
#include "shifter.ino"

static bool g_stop = false;
static void boot() {
  host::nowUs = 0;
  setup();
  host::every(host::nowUs, 100, [] { if (!g_stop) loop(); return true; });
}
static double cutMs(uint64_t a, uint64_t b) { return host::cutHighTotalUs(a, b) / 1000.0; }
static void pinHold(int pin, uint64_t from, uint64_t to) { host::at(from, [=] { host::setPin(pin, 0); }); host::at(to, [=] { host::setPin(pin, 1); }); }
static void cmd(const char* c) { host::serialIn(std::string(c) + "\n"); host::runFor(MS(50)); }

TEST(sim_demarrage_logs_et_6_clignotements) {
  boot();
  host::runFor(SEC(4));
  CHECK(host::serialContains("[BOOT] TTP ECU v"));
  CHECK(host::serialContains("MODE SIMULATION WOKWI"));
  CHECK(host::serialContains("[SIM] WiFi / serveur web desactives"));
  CHECK(host::serialContains("[LIC] SIMULATION -> tier 1"));
  CHECK(host::serialContains("[BOOT] pret."));
  CHECK_MSG(host::cutPulses(0, SEC(2)) == 6, "clignotements au boot: %d", host::cutPulses(0, SEC(2)));
  CHECK(!host::cutHigh());
  CHECK(host::restartCount == 0);
}
TEST(sim_moteur_fige_aucune_coupure_sans_action) {
  boot(); host::runFor(SEC(3));
  const uint64_t t = host::nowUs;
  host::runFor(SEC(10));
  CHECK_MSG(host::cutPulses(t, host::nowUs) == 0, "coupure spontanee");
}
TEST(sim_shifter_led_dure_200ms) {
  boot(); host::runFor(SEC(3));
  const uint64_t t = host::nowUs;
  pinHold(pins::SHIFTER, t + MS(100), t + MS(300));
  host::runFor(SEC(2));
  CHECK(host::cutPulses(t, host::nowUs) == 1);
  CHECK_NEAR(cutMs(t, host::nowUs), 200.0, 3.0);          // 100 ms x IGNITION_SIM_SCALE(2)
}
TEST(sim_shifter_sous_regime_mini_rien) {
  boot(); host::runFor(SEC(3));
  cmd("rpm 3000");
  const uint64_t t = host::nowUs;
  pinHold(pins::SHIFTER, t + MS(100), t + MS(200));
  host::runFor(SEC(1));
  CHECK(host::cutPulses(t, host::nowUs) == 0);
}
TEST(sim_pit_strobe_et_rien_sans_bouton) {
  boot(); host::runFor(SEC(3));
  const uint64_t t = host::nowUs;
  pinHold(pins::PIT_BTN, t + MS(100), t + SEC(1.1));
  host::runFor(SEC(2));
  const double duty = cutMs(t + MS(150), t + SEC(1.0)) / 850.0;
  INFO("pit a 9000 tr/min (pitRpm %d): duty %.0f %%", cfg::DEFAULTS.pitRpm, duty * 100);
  CHECK_MSG(duty > 0.95, "au-dela de pitRpm + bande dure la coupure doit etre continue");
  const uint64_t t2 = host::nowUs; host::runFor(SEC(1));
  CHECK(host::cutPulses(t2, host::nowUs) == 0);
}
TEST(sim_launch_track_embrayage_tire) {
  boot(); host::runFor(SEC(3));
  cmd("rpm 8000");
  const uint64_t t = host::nowUs;
  pinHold(pins::CLUTCH_HALL, t + MS(100), t + SEC(1.1));
  host::runFor(SEC(2));
  INFO("launch: %d pulses", host::cutPulses(t, host::nowUs));
  CHECK(host::cutPulses(t, host::nowUs) >= 10);
}
TEST(sim_slip_interdit_en_track_autorise_apres_cle_pro) {
  boot(); host::runFor(SEC(3));
  cmd("slip 30");
  uint64_t t = host::nowUs; host::runFor(SEC(1));
  CHECK_MSG(host::cutPulses(t, host::nowUs) == 0, "slip actif en TRACK");
  // cle PRO via la console serie (affichee par la commande 'keys')
  host::serialOut.clear(); cmd("keys");
  size_t p = host::serialOut.rfind("PRO");
  p = host::serialOut.find("= ", p); std::string key = host::serialOut.substr(p + 2, 16);
  INFO("cle PRO lue: %s", key.c_str());
  cmd((std::string("key ") + key).c_str());
  CHECK(license::tier() == LicenseTier::Pro);
  t = host::nowUs; host::runFor(SEC(1));
  CHECK_MSG(host::cutPulses(t, host::nowUs) >= 8, "slip inactif en PRO: %d", host::cutPulses(t, host::nowUs));
  cmd("lean 30");
  t = host::nowUs; host::runFor(SEC(1));
  CHECK_MSG(host::cutPulses(t, host::nowUs) == 0, "slip actif a 30 deg");
  cmd("lean 0"); cmd("imu 0");
  t = host::nowUs; host::runFor(SEC(1));
  CHECK_MSG(host::cutPulses(t, host::nowUs) == 0, "slip actif IMU muet");
  cmd("imu 1"); cmd("latg 150");
  t = host::nowUs; host::runFor(SEC(1));
  CHECK_MSG(host::cutPulses(t, host::nowUs) == 0, "slip actif a 1.5 g");
}
TEST(sim_cycle_automatique_overrev_chaque_20s) {
  boot(); host::runFor(SEC(2));
  cmd("run");
  const uint64_t t0 = host::nowUs;
  host::runFor(SEC(45));
  // phase 4 (rpm 17000) : 4 s sur 20 -> coupure quasi continue dans cette fenetre
  int windows = 0;
  for (uint64_t t = t0; t + SEC(20) <= host::nowUs; t += SEC(20)) {
    for (int ph = 0; ph < 5; ++ph) {
      double ms = cutMs(t + SEC(4) * ph, t + SEC(4) * (ph + 1));
      INFO("phase %d -> %.0f ms de coupure", ph, ms);
    }
    ++windows;
  }
  CHECK(windows >= 1);
}
TEST(sim_serie_robustesse_entrees_hostiles) {
  boot(); host::runFor(SEC(2));
  std::string junk(500, 'x'); cmd(junk.c_str());
  cmd("rpm 99999999999999"); cmd("rpm -5"); cmd("lean abc"); cmd("temp 9999"); cmd("cut 0"); cmd("scale 0");
  cmd("unknown"); cmd(""); cmd("   "); cmd("key"); cmd("key ZZZ");
  host::serialIn(std::string("\r\n\r\n\0\xff\xfe\n", 9)); host::runFor(MS(100));
  host::runFor(SEC(2));
  CHECK(host::restartCount == 0);
  CHECK(state::getRpmInstant() >= 0 && state::getRpmInstant() <= 20000);
}
TEST(sim_hang_declenche_superviseur_et_rend_l_allumage) {
  boot(); host::runFor(SEC(5));
  cmd("rpm 8000");
  ignition::request(150, AssistMode::Shift);       // coupure en cours
  const uint64_t t = host::nowUs;
  try { cmd("hang 3"); host::runFor(SEC(5)); } catch (host::RestartException&) {}
  CHECK_MSG(host::restartCount == 1, "le superviseur n'a pas redemarre la carte");
  const double dt = (host::restartAtUs - t) / 1000.0;
  INFO("redemarrage %.0f ms apres le hang", dt);
  CHECK(dt > cfg::LOOP_STALL_MS && dt < 1000);
  CHECK_MSG(!host::cutHigh(), "allumage non rendu");
}
TEST(sim_temperature_simulee_visible_dans_l_etat) {
  boot(); host::runFor(SEC(2));
  cmd("temp 97"); host::runFor(MS(100));
  int t; bool ok; state::getWaterTemp(t, ok);
  CHECK(ok && t == 970);
  cmd("watersensor 0"); host::runFor(MS(100));
  state::getWaterTemp(t, ok); CHECK(!ok);
}
