// SRC: core/SharedState.cpp actuator/IgnitionCut.cpp net/Telemetry.cpp persistence/License.cpp persistence/Storage.cpp sim/SimEngine.cpp
#include "host.h"
#include "mock/Arduino.h"
#include "src/core/Config.h"
#include "src/core/SharedState.h"
#include "src/actuator/IgnitionCut.h"
#include "src/net/Telemetry.h"
#include "src/net/WebServer.h"
#include "src/persistence/License.h"

// ---- doublures du serveur web ----------------------------------------------------------
static size_t g_clients = 1;
static std::vector<std::string> g_sse;
size_t web::sseClients() { return g_clients; }
void web::sseSend(const char* json, const char* event) { g_sse.push_back(std::string(event) + "|" + json); }
void web::begin() {}

// ECU "vivante" : loop() bat toutes les 200 us tant que g_frozen est faux.
static bool g_frozen = false;
static void startLoop() {
  host::every(host::nowUs, 200, [] { if (!g_frozen) { ignition::service(); state::beatEngineLoop(); } return true; });
}
static void bootSup(LicenseTier t = LicenseTier::Pro) {
  host::nowUs = SEC(1);
  host::nvs.clear(); license::begin();                 // RAM remise a BASIC (NVS vide)
  if (t != LicenseTier::Basic) { uint32_t r = 0; if (!license::tryUnlock(license::expectedKey(t), r)) host::fail(__FILE__, __LINE__, "tryUnlock refuse"); }
  state::begin(); ignition::begin(); state::beatEngineLoop();
  telemetry::startTasks();
  startLoop();
}

TEST(superviseur_pas_de_faux_positif_en_marche_normale) {
  bootSup();
  host::runFor(SEC(30));
  CHECK(host::restartCount == 0);
}
TEST(superviseur_loop_figee_rend_l_etincelle_puis_redemarre) {
  bootSup();
  host::runFor(SEC(5));
  ignition::request(150, AssistMode::Shift);          // coupure en cours au moment du gel
  g_frozen = true;
  const uint64_t tf = host::nowUs;
  bool cutLowedBeforeRestart = false;
  try {
    host::every(tf, 1000, [&] { if (!host::cutHigh()) cutLowedBeforeRestart = true; return true; });
    host::runFor(SEC(2));
  } catch (host::RestartException&) {}
  const double dt = (host::restartAtUs - tf) / 1000.0;
  INFO("gel a t=%.3f s ; redemarrage %.0f ms plus tard", tf / 1e6, dt);
  CHECK_MSG(host::restartCount == 1, "pas de redemarrage");
  CHECK_MSG(dt >= cfg::LOOP_STALL_MS && dt <= cfg::LOOP_STALL_MS + cfg::SUPERVISOR_PERIOD_MS + 50, "delai de detection %.0f ms", dt);
  CHECK_MSG(!host::cutHigh(), "allumage non rendu au redemarrage");
  CHECK(cutLowedBeforeRestart);
}
TEST(superviseur_blocage_court_300ms_tolere) {
  bootSup();
  host::runFor(SEC(5));
  g_frozen = true; host::runFor(MS(300)); g_frozen = false;       // ex. ecriture NVS un peu longue
  host::runFor(SEC(3));
  CHECK_MSG(host::restartCount == 0, "redemarrage sur un blocage de 300 ms (< LOOP_STALL_MS)");
}
TEST(superviseur_blocage_500ms_redemarre) {
  bootSup();
  host::runFor(SEC(5));
  g_frozen = true;
  try { host::runFor(MS(900)); } catch (host::RestartException&) {}
  CHECK(host::restartCount == 1);
}
TEST(superviseur_periode_de_grace_documentee) {
  // Un gel dans les 3 premieres secondes n'est PAS vu (l'init WiFi peut etre longue) : on le mesure.
  bootSup();
  host::runFor(MS(500));
  g_frozen = true;
  try { host::runFor(SEC(2)); } catch (host::RestartException&) {}
  INFO("restart pendant la grace: %d", host::restartCount);
  CHECK_MSG(host::restartCount == 0, "(informatif) la grace de 3 s masque un gel precoce");
  try { host::runFor(SEC(2)); } catch (host::RestartException&) {}
  CHECK(host::restartCount == 1);
}

// ---- SSE ------------------------------------------------------------------------------------
static bool validJsonObject(const std::string& s, std::string& why) {
  size_t i = 0; auto ws = [&] { while (i < s.size() && isspace((unsigned char)s[i])) ++i; };
  ws(); if (i >= s.size() || s[i] != '{') { why = "pas un objet"; return false; } ++i;
  for (;;) {
    ws(); if (s[i] != '"') { why = "cle attendue @" + std::to_string(i); return false; }
    ++i; while (i < s.size() && s[i] != '"') ++i; if (i >= s.size()) { why = "cle non fermee"; return false; } ++i;
    ws(); if (s[i] != ':') { why = "':' attendu"; return false; } ++i; ws();
    if (s[i] == '"') { ++i; while (i < s.size() && s[i] != '"') ++i; if (i >= s.size()) { why = "chaine non fermee"; return false; } ++i; }
    else if (!strncmp(s.c_str() + i, "true", 4)) i += 4; else if (!strncmp(s.c_str() + i, "false", 5)) i += 5;
    else { size_t st = i; if (s[i] == '-') ++i; while (i < s.size() && (isdigit((unsigned char)s[i]) || s[i] == '.')) ++i;
      if (i == st || (i == st + 1 && s[st] == '-')) { why = "valeur invalide @" + std::to_string(i) + " (" + s.substr(st, 8) + ")"; return false; } }
    ws(); if (s[i] == ',') { ++i; continue; } if (s[i] == '}') return true; why = "',' ou '}' attendu"; return false;
  }
}
TEST(sse_json_valide_dans_tous_les_etats) {
  bootSup();
  const int rpms[] = {0, 8000, 20000};
  for (int rpm : rpms) for (int ok = 0; ok < 2; ++ok) {
    state::setRpmFiltered(rpm); state::setLean(-285, 130, ok); state::setWaterTemp(ok ? 951 : 0, ok);
    state::setSpeeds(1234, 1300); state::setSlip(105, 0);
    g_sse.clear(); host::runFor(MS(600));
    CHECK(!g_sse.empty());
    for (auto& m : g_sse) {
      std::string why; std::string body = m.substr(m.find('|') + 1);
      CHECK_MSG(validJsonObject(body, why), "JSON invalide (%s): %s", why.c_str(), body.c_str());
      CHECK(m.rfind("telemetry|", 0) == 0);
      CHECK_MSG(body.size() < 255, "trame de %zu octets proche du buffer de 256", body.size());
    }
  }
}
TEST(sse_pas_d_emission_sans_client) {
  bootSup(); g_clients = 0; g_sse.clear();
  host::runFor(SEC(3));
  CHECK(g_sse.empty());
}
TEST(sse_etat_complet_present) {
  bootSup();
  state::setRpmFiltered(7000); state::setWaterTemp(881, true); state::setLean(123, 100, true);
  g_sse.clear(); host::runFor(MS(500));
  CHECK(!g_sse.empty());
  const std::string& m = g_sse.back();
  CHECK(m.find("\"rpm\":7000") != std::string::npos);
  CHECK(m.find("\"waterTemp\":88.1") != std::string::npos);
  CHECK(m.find("\"waterOk\":true") != std::string::npos);
  CHECK(m.find("\"lean\":12.3") != std::string::npos);
}

// ---- telemetrie reservee PRO : en dessous, AUCUNE valeur capteur dans le flux ------------------
static void lockedCase(LicenseTier t, const char* name) {
  bootSup(t);
  state::setRpmFiltered(7000); state::setWaterTemp(881, true); state::setLean(123, 100, true);
  state::setSpeeds(1234, 1300); state::setSlip(105, 0);
  g_sse.clear(); host::runFor(SEC(2));
  CHECK_MSG(!g_sse.empty(), "pas de trame %s", name);
  const std::string expect = std::string("telemetry|{\"tier\":\"") + toString(t) + "\",\"locked\":true}";
  for (auto& m : g_sse) {
    CHECK_MSG(m == expect, "trame inattendue: %s", m.c_str());
    for (const char* k : {"rpm", "speed", "slip", "lean", "imu", "waterTemp", "waterOk", "mode"})
      CHECK_MSG(m.find(std::string("\"") + k + "\"") == std::string::npos, "champ %s fuit en %s", k, name);
  }
}
TEST(sse_basic_trame_verrouillee_sans_donnees) { lockedCase(LicenseTier::Basic, "BASIC"); }
TEST(sse_track_trame_verrouillee_sans_donnees) { lockedCase(LicenseTier::Track, "TRACK"); }
TEST(sse_pro_trame_complete_sans_locked) {
  bootSup(LicenseTier::Pro);
  state::setRpmFiltered(7000); state::setWaterTemp(881, true); state::setLean(123, 100, true);
  state::setSpeeds(1234, 1300); state::setSlip(105, 0);
  g_sse.clear(); host::runFor(MS(500));
  CHECK(!g_sse.empty());
  for (auto& m : g_sse) {
    CHECK(m.find("locked") == std::string::npos);
    CHECK(m.find("\"rpm\":7000") != std::string::npos);
    CHECK(m.find("\"speed\":123.4") != std::string::npos);
    CHECK(m.find("\"slip\":10.5") != std::string::npos);
    CHECK(m.find("\"tier\":\"PRO") != std::string::npos);
  }
}
