// SRC: core/SharedState.cpp persistence/Storage.cpp persistence/License.cpp net/Network.cpp net/WebServer.cpp sim/SimEngine.cpp
//
// Les gestionnaires HTTP du VRAI WebServer.cpp, appeles avec des requetes fabriquees
// (le transport AsyncTCP n'est pas simule, la logique d'auth / validation / OTA l'est).
#include "host.h"
#include "mock/Arduino.h"
#include "mock/ESPAsyncWebServer.h"
#include "mock/Update.h"
#include "src/core/Config.h"
#include "src/core/SharedState.h"
#include "src/persistence/Storage.h"
#include "src/persistence/License.h"
#include "src/net/Network.h"
#include "src/net/WebServer.h"
#include <memory>

typedef std::map<std::string, std::string> M;

static std::string g_pin;
static void boot() {
  host::nowUs = SEC(10);
  state::begin(); storage::begin(); state::setConfig(cfg::DEFAULTS);
  net::begin();
  host::serialOut.clear();
  web::begin();
  size_t p = host::serialOut.find("CONFIG PIN: ");
  g_pin = host::serialOut.substr(p + 12, 6);
}
static AsyncWebServerRequest* call(const char* path, M post = M(), M hdr = M()) {
  auto* r = new AsyncWebServerRequest(); r->post = post; r->hdr = hdr;
  hostweb::routes().at(path).h(r);
  return r;
}
static int code(AsyncWebServerRequest* r) { return r->sent ? r->sent->code : -1; }
static std::string body(AsyncWebServerRequest* r) { return r->sent ? r->sent->body : ""; }
static std::string login() {
  auto* r = call("/api/login", {{"pin", g_pin}});
  std::string b = body(r); size_t p = b.find("\"token\":\"");
  return b.substr(p + 9, 32);
}
static M bearer(const std::string& t) { return {{"Authorization", "Bearer " + t}}; }

// ============================ Pages publiques ===========================================================
TEST(web_racine_en_tetes_de_securite) {
  boot();
  auto* r = call("/");
  CHECK(code(r) == 200);
  auto& h = r->sent->headers;
  CHECK(h.count("Content-Security-Policy") && h["Content-Security-Policy"].find("default-src 'none'") != std::string::npos);
  CHECK(h["X-Content-Type-Options"] == "nosniff" && h["Cache-Control"] == "no-store");
  CHECK(r->sent->body.find("<title>Aptatio ECU</title>") != std::string::npos);
  CHECK_MSG(r->sent->body.find("src=\"http") == std::string::npos, "ressource externe dans la page");
}
TEST(web_page_calculateur_pit_basic_present) {
  boot();
  const std::string h = call("/")->sent->body;
  for (const char* id : {"id=\"pitKmh\"", "id=\"pitRpm\"", "id=\"pitRatio\"", "id=\"btnPitCalc\"", "pitRpmFor"})
    CHECK_MSG(h.find(id) != std::string::npos, "%s absent de la page", id);
}
TEST(web_etat_json_valide_et_complet) {
  boot();
  auto* r = call("/api/state");
  CHECK(code(r) == 200);
  const std::string b = body(r);
  for (const char* k : {"\"fw\"", "\"ssid\"", "\"hwid\"", "\"tier\"", "\"shiftCutMs\"", "\"minShiftRpm\"", "\"pitRpm\"", "\"pitKmh\"", "\"launchRpm\"", "\"maxSlipPercent\"", "\"wheelCircumMm\"", "\"pulsesPerRev\""})
    CHECK_MSG(b.find(k) != std::string::npos, "champ %s absent: %s", k, b.c_str());
  CHECK(b.front() == '{' && b.back() == '}');
  CHECK_MSG(b.size() < 600, "reponse de %zu octets proche du buffer de 640", b.size());
  CHECK(b.find("\"pin\"") == std::string::npos);                      // jamais de secret dans /api/state
  CHECK(b.find(net::apPassword().c_str()) == std::string::npos);
}
TEST(web_etat_ne_fuite_ni_pin_ni_mot_de_passe_wifi) {
  boot();
  auto* r = call("/api/state");
  CHECK(body(r).find(g_pin) == std::string::npos);
}

// ============================ Authentification ============================================================
TEST(web_login_correct_donne_un_jeton_hex_de_32) {
  boot();
  auto* r = call("/api/login", {{"pin", g_pin}});
  CHECK(code(r) == 200);
  std::string t = login();
  CHECK(t.size() == 32);
  for (char c : t) CHECK_MSG(isxdigit((unsigned char)c), "jeton non hexa: %s", t.c_str());
}
TEST(web_login_mauvais_pin_403_et_verrouillage_apres_5) {
  boot();
  for (int i = 0; i < 4; ++i) CHECK(code(call("/api/login", {{"pin", "000000"}})) == 403);
  CHECK(code(call("/api/login", {{"pin", "000000"}})) == 403);          // 5e echec -> verrou
  auto* r = call("/api/login", {{"pin", g_pin}});                        // meme le bon PIN est refuse
  CHECK_MSG(code(r) == 429, "code %d", code(r));
  CHECK(r->sent->headers.count("Retry-After"));
  host::runFor(MS(cfg::AUTH_LOCKOUT_MS + 500));
  CHECK(code(call("/api/login", {{"pin", g_pin}})) == 200);
}
TEST(web_login_parametre_absent_ou_longueur_fausse) {
  boot();
  CHECK(code(call("/api/login")) == 400);
  CHECK(code(call("/api/login", {{"pin", ""}})) == 403);
  CHECK(code(call("/api/login", {{"pin", g_pin + "0"}})) == 403);
  CHECK(code(call("/api/login", {{"pin", g_pin.substr(0, 5)}})) == 403);
}
TEST(web_pin_aleatoire_6_chiffres_different_a_chaque_boot_simule) {
  boot(); std::string a = g_pin; CHECK(a.size() == 6);
  for (char c : a) CHECK(isdigit((unsigned char)c));
}
TEST(web_jeton_requis_sur_les_routes_sensibles) {
  boot();
  for (const char* p : {"/api/config", "/api/wifi"}) {
    CHECK_MSG(code(call(p)) == 401, "%s sans jeton", p);
    CHECK_MSG(code(call(p, {{"shiftCutMs", "60"}}, bearer("0123456789abcdef0123456789abcdef"))) == 401, "%s jeton invalide", p);
    CHECK_MSG(code(call(p, {}, {{"Authorization", "Basic abc"}})) == 401, "%s mauvais schema", p);
  }
}
TEST(web_jeton_expire_apres_ttl) {
  boot(); std::string t = login();
  CHECK(code(call("/api/config", {{"shiftCutMs", "60"}}, bearer(t))) == 200);
  host::runFor(MS(cfg::AUTH_TOKEN_TTL_MS - 1000));
  CHECK(code(call("/api/config", {{"shiftCutMs", "61"}}, bearer(t))) == 200);
  host::runFor(MS(2000));
  CHECK_MSG(code(call("/api/config", {{"shiftCutMs", "62"}}, bearer(t))) == 401, "jeton encore valide apres le TTL");
}
TEST(web_nouveau_login_invalide_l_ancien_jeton) {
  boot(); std::string t1 = login(); std::string t2 = login();
  CHECK(t1 != t2);
  CHECK(code(call("/api/config", {{"shiftCutMs", "60"}}, bearer(t1))) == 401);
  CHECK(code(call("/api/config", {{"shiftCutMs", "60"}}, bearer(t2))) == 200);
}

// ============================ Configuration ==================================================================
TEST(web_config_valide_appliquee_et_persistee) {
  boot(); std::string t = login();
  auto* r = call("/api/config", {{"shiftCutMs", "70"}, {"minShiftRpm", "7500"}, {"pitRpm", "5000"}, {"wheelCircumMm", "1750"}, {"pulsesPerRev", "4"}}, bearer(t));
  CHECK(code(r) == 200);
  EcuConfig c = state::getConfig();
  CHECK(c.shiftCutMs == 70 && c.minShiftRpm == 7500 && c.pitRpm == 5000 && c.wheelCircumMm == 1750 && c.pulsesPerRev == 4);
  EcuConfig s = storage::loadConfig();
  CHECK(s.shiftCutMs == 70 && s.pitRpm == 5000);
}
TEST(web_config_valeur_invalide_400_et_rien_n_est_enregistre) {
  boot(); std::string t = login(); EcuConfig before = state::getConfig();
  const char* bad[] = {"", "abc", "12x", "-1", "999999", "1.5", " 60", "+60", "0"};
  for (const char* v : bad) {
    auto* r = call("/api/config", {{"shiftCutMs", v}}, bearer(t));
    CHECK_MSG(code(r) == 400, "shiftCutMs='%s' -> %d", v, code(r));
  }
  CHECK(state::getConfig().shiftCutMs == before.shiftCutMs);
  // un champ valide + un champ invalide : AUCUN des deux n'est applique
  auto* r = call("/api/config", {{"shiftCutMs", "90"}, {"pitRpm", "xx"}}, bearer(t));
  CHECK(code(r) == 400);
  CHECK_MSG(state::getConfig().shiftCutMs == before.shiftCutMs, "application partielle d'une requete invalide");
  CHECK(storage::loadConfig().shiftCutMs == before.shiftCutMs);
}
TEST(web_config_bornes_exactes_acceptees_et_depassement_refuse) {
  boot(); std::string t = login();
  CHECK(code(call("/api/config", {{"shiftCutMs", std::to_string(cfg::SHIFT_CUT_MIN)}}, bearer(t))) == 200);
  CHECK(code(call("/api/config", {{"shiftCutMs", std::to_string(cfg::SHIFT_CUT_MAX)}}, bearer(t))) == 200);
  CHECK(code(call("/api/config", {{"shiftCutMs", std::to_string(cfg::SHIFT_CUT_MIN - 1)}}, bearer(t))) == 400);
  CHECK(code(call("/api/config", {{"shiftCutMs", std::to_string(cfg::SHIFT_CUT_MAX + 1)}}, bearer(t))) == 400);
}
TEST(web_config_pitKmh_valide_et_bornes) {
  boot(); std::string t = login();
  CHECK(code(call("/api/config", {{"pitKmh", "50"}}, bearer(t))) == 200);
  CHECK(state::getConfig().pitKmh == 50 && storage::loadConfig().pitKmh == 50);
  CHECK(code(call("/api/config", {{"pitKmh", "9"}}, bearer(t))) == 400);
  CHECK(code(call("/api/config", {{"pitKmh", "101"}}, bearer(t))) == 400);
  CHECK(code(call("/api/config", {{"pitKmh", "abc"}}, bearer(t))) == 400);
  CHECK(state::getConfig().pitKmh == 50);
}
TEST(web_config_roues_et_pit_accessibles_en_basic) {
  // Circonference, impulsions/tour, pitKmh, pitRpm : aucune licence requise (le pit limiter est en BASIC).
  boot(); std::string t = login();
  CHECK(license::tier() == LicenseTier::Basic);
  auto* r = call("/api/config", {{"wheelCircumMm", "1650"}, {"pulsesPerRev", "5"}, {"pitKmh", "45"}, {"pitRpm", "5500"}}, bearer(t));
  CHECK_MSG(code(r) == 200, "code %d : %s", code(r), body(r).c_str());
  EcuConfig c = state::getConfig();
  CHECK(c.wheelCircumMm == 1650 && c.pulsesPerRev == 5 && c.pitKmh == 45 && c.pitRpm == 5500);
}
TEST(web_config_licences_launch_track_slip_pro) {
  boot(); std::string t = login();
  CHECK(code(call("/api/config", {{"launchRpm", "9000"}}, bearer(t))) == 403);
  CHECK(code(call("/api/config", {{"maxSlipPercent", "20"}}, bearer(t))) == 403);
  uint32_t retry = 0; license::tryUnlock(license::expectedKey(LicenseTier::Track), retry);
  CHECK(code(call("/api/config", {{"launchRpm", "9000"}}, bearer(t))) == 200);
  CHECK(code(call("/api/config", {{"maxSlipPercent", "20"}}, bearer(t))) == 403);
  license::tryUnlock(license::expectedKey(LicenseTier::Pro), retry);
  CHECK(code(call("/api/config", {{"maxSlipPercent", "20"}}, bearer(t))) == 200);
  CHECK(state::getConfig().maxSlipPercent == 20 && state::getConfig().launchRpm == 9000);
}
TEST(web_config_refusee_moteur_en_marche_409) {
  boot(); std::string t = login(); state::setRpmFiltered(3000);
  CHECK(code(call("/api/config", {{"shiftCutMs", "60"}}, bearer(t))) == 409);
  state::setRpmFiltered(0); state::setRpmInstant(2500);
  CHECK(code(call("/api/config", {{"shiftCutMs", "60"}}, bearer(t))) == 409);
  state::setRpmInstant(0);
  CHECK(code(call("/api/config", {{"shiftCutMs", "60"}}, bearer(t))) == 200);
}
TEST(web_config_injection_de_champ_inconnu_ignoree) {
  boot(); std::string t = login();
  auto* r = call("/api/config", {{"shiftCutMs", "60"}, {"licenseTier", "2"}, {"__proto__", "x"}, {"wifiPass", "pwned"}}, bearer(t));
  CHECK(code(r) == 200);
  CHECK(license::tier() == LicenseTier::Basic);
  CHECK(storage::loadWifiPassword() == net::apPassword());
}

// ============================ Licence + WiFi =================================================================
TEST(web_licence_bonne_cle_200_reboot_programme) {
  boot();
  auto* r = call("/api/license", {{"key", std::string(license::expectedKey(LicenseTier::Pro).c_str())}});
  CHECK(code(r) == 200);
  CHECK(license::tier() == LicenseTier::Pro);
  CHECK(net::rebootPending());
}
TEST(web_licence_mauvaise_cle_403_puis_429) {
  boot();
  for (int i = 0; i < 7; ++i) CHECK(code(call("/api/license", {{"key", "0000000000000000"}})) == 403);
  auto* r = call("/api/license", {{"key", "0000000000000000"}});
  CHECK_MSG(code(r) == 429, "code %d", code(r));
  CHECK(r->sent->headers.count("Retry-After"));
  CHECK(code(call("/api/license")) == 400);
}
TEST(web_licence_refusee_moteur_en_marche) {
  boot(); state::setRpmFiltered(4000);
  CHECK(code(call("/api/license", {{"key", std::string(license::expectedKey(LicenseTier::Pro).c_str())}})) == 409);
  CHECK(license::tier() == LicenseTier::Basic && !net::rebootPending());
}
TEST(web_wifi_rotation_authentifiee_et_refusee_moteur_en_marche) {
  boot();
  CHECK(code(call("/api/wifi")) == 401);
  std::string t = login();
  state::setRpmFiltered(3000);
  CHECK(code(call("/api/wifi", {}, bearer(t))) == 409);
  state::setRpmFiltered(0);
  String old = net::apPassword();
  auto* r = call("/api/wifi", {}, bearer(t));
  CHECK(code(r) == 200);
  std::string b = body(r); size_t p = b.find("\"password\":\"");
  std::string np = b.substr(p + 12, 12);
  CHECK(np.size() == 12 && np != old.c_str());
  CHECK(storage::loadWifiPassword() == String(np));
  CHECK(net::rebootPending());
}

// ============================ OTA ===============================================================================
static void upload(AsyncWebServerRequest* r, const std::vector<uint8_t>& img, size_t chunk) {
  auto up = hostweb::routes().at("/api/ota").up;
  size_t i = 0;
  while (i < img.size() || i == 0) {
    size_t n = std::min(chunk, img.size() - i);
    std::vector<uint8_t> c(img.begin() + i, img.begin() + i + n);
    up(r, "fw.bin", i, c.data(), n, i + n >= img.size());
    i += n; if (n == 0) break;
  }
}
static std::vector<uint8_t> image(size_t n, uint8_t magic = 0xE9) { std::vector<uint8_t> v(n, 0x5A); v[0] = magic; return v; }
TEST(web_ota_sans_jeton_refusee_sans_ecrire) {
  boot();
  auto* r = new AsyncWebServerRequest();
  upload(r, image(5000), 1024);
  hostweb::routes().at("/api/ota").h(r);
  CHECK(code(r) == 403);
  CHECK_MSG(Update.begins == 0 && Update.written == 0, "ecriture flash sans authentification");
  CHECK(!net::rebootPending());
}
TEST(web_ota_moteur_en_marche_refusee) {
  boot(); std::string t = login(); state::setRpmInstant(3000);
  auto* r = new AsyncWebServerRequest(); r->hdr = bearer(t);
  upload(r, image(5000), 1024);
  hostweb::routes().at("/api/ota").h(r);
  CHECK(code(r) == 403);
  CHECK(Update.begins == 0);
}
TEST(web_ota_image_valide_200_et_reboot) {
  boot(); std::string t = login();
  auto* r = new AsyncWebServerRequest(); r->hdr = bearer(t);
  upload(r, image(20000), 1460);
  hostweb::routes().at("/api/ota").h(r);
  CHECK(code(r) == 200);
  CHECK(Update.written == 20000 && Update.ended);
  CHECK(net::rebootPending());
}
TEST(web_ota_image_invalide_magic_500_sans_reboot) {
  boot(); std::string t = login();
  auto* r = new AsyncWebServerRequest(); r->hdr = bearer(t);
  upload(r, image(5000, 0x00), 1024);
  hostweb::routes().at("/api/ota").h(r);
  CHECK(code(r) == 500);
  CHECK_MSG(!net::rebootPending(), "redemarrage apres une image invalide");
}
TEST(web_ota_apres_un_refus_une_nouvelle_tentative_valide_passe) {
  boot(); std::string t = login();
  { auto* r = new AsyncWebServerRequest(); upload(r, image(3000), 1024); hostweb::routes().at("/api/ota").h(r); CHECK(code(r) == 403); }
  { auto* r = new AsyncWebServerRequest(); r->hdr = bearer(t); upload(r, image(3000), 1024); hostweb::routes().at("/api/ota").h(r); CHECK(code(r) == 200); }
}
