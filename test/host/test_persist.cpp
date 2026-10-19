// SRC: core/SharedState.cpp persistence/Storage.cpp persistence/License.cpp net/Network.cpp sim/SimEngine.cpp
#include "host.h"
#include "mock/Arduino.h"
#include "mock/WiFi.h"
#include "src/core/Config.h"
#include "src/core/SharedState.h"
#include "mock/Preferences.h"
#include "src/persistence/Storage.h"
#include "src/persistence/License.h"
#include "src/persistence/secrets.h"
#include "src/net/Network.h"
#include "src/net/ParamParse.h"
#include <cstdlib>

static void boot() { host::nowUs = SEC(10); state::begin(); storage::begin(); }

// ============================ Config / NVS =========================================
TEST(config_defauts_dans_les_bornes) {
  const EcuConfig d = cfg::DEFAULTS, s = storage::sanitize(d);
  CHECK(d.shiftCutMs == s.shiftCutMs && d.minShiftRpm == s.minShiftRpm && d.launchRpm == s.launchRpm &&
        d.maxSlipPercent == s.maxSlipPercent && d.wheelCircumMm == s.wheelCircumMm &&
        d.pulsesPerRev == s.pulsesPerRev && d.pitRpm == s.pitRpm);
}
TEST(config_sanitize_clampe_chaque_champ) {
  EcuConfig lo = {-5, -5, -5, -5, -5, -5, -5, -5}, hi = {99999, 99999, 99999, 99999, 99999, 99999, 99999, 99999};
  EcuConfig a = storage::sanitize(lo), b = storage::sanitize(hi);
  CHECK(a.shiftCutMs == cfg::SHIFT_CUT_MIN && b.shiftCutMs == cfg::SHIFT_CUT_MAX);
  CHECK(a.minShiftRpm == cfg::MIN_RPM_MIN && b.minShiftRpm == cfg::MIN_RPM_MAX);
  CHECK(a.launchRpm == cfg::LAUNCH_MIN && b.launchRpm == cfg::LAUNCH_MAX);
  CHECK(a.maxSlipPercent == cfg::SLIP_MIN && b.maxSlipPercent == cfg::SLIP_MAX);
  CHECK(a.wheelCircumMm == cfg::WHEEL_MIN && b.wheelCircumMm == cfg::WHEEL_MAX);
  CHECK(a.pulsesPerRev == cfg::PULSES_MIN && b.pulsesPerRev == cfg::PULSES_MAX);
  CHECK(a.pitRpm == cfg::PIT_MIN && b.pitRpm == cfg::PIT_MAX);
  CHECK(a.pitKmh == cfg::PITKMH_MIN && b.pitKmh == cfg::PITKMH_MAX);
}
TEST(config_bornes_coherentes_avec_securite) {
  CHECK_MSG(cfg::SLIP_MIN > cfg::SLIP_HYSTERESIS_PCT, "tolerance mini <= hysteresis : le slip ne se desarme jamais");
  CHECK_MSG(cfg::LAUNCH_MAX + cfg::LAUNCH_ARM_BAND_RPM < cfg::OVERREV_RPM + 3000, "launch trop pres de l'overrev");
  CHECK_MSG(cfg::PIT_MAX < cfg::OVERREV_RPM, "pit limiter au-dessus de l'overrev");
  CHECK_MSG(cfg::MIN_RPM_MAX < cfg::OVERREV_RPM, "regime mini du shifter au-dessus de l'overrev");
  CHECK_MSG(cfg::SHIFT_CUT_MAX <= cfg::CUT_ABS_MAX_MS, "coupure shifter max > plafond absolu");
  CHECK(cfg::RPM_MAX_PLAUSIBLE > cfg::OVERREV_RPM);
  CHECK(cfg::LOOP_STALL_MS < cfg::WDT_TIMEOUT_S * 1000);
}
TEST(config_nvs_vide_donne_les_defauts) {
  boot();
  EcuConfig c = storage::loadConfig();
  CHECK(c.shiftCutMs == cfg::DEFAULTS.shiftCutMs && c.pitRpm == cfg::DEFAULTS.pitRpm && c.maxSlipPercent == cfg::DEFAULTS.maxSlipPercent);
  CHECK(storage::loadTier() == LicenseTier::Basic);
  CHECK(storage::loadLicenseKey().length() == 0 && storage::loadWifiPassword().length() == 0);
}
TEST(config_aller_retour_nvs) {
  boot();
  EcuConfig c = {70, 7000, 9000, 20, 1750, 4, 5000, 45};
  storage::saveConfig(c);
  EcuConfig r = storage::loadConfig();
  CHECK(r.shiftCutMs == 70 && r.minShiftRpm == 7000 && r.launchRpm == 9000 && r.maxSlipPercent == 20 &&
        r.wheelCircumMm == 1750 && r.pulsesPerRev == 4 && r.pitRpm == 5000 && r.pitKmh == 45);
}
TEST(config_nvs_corrompue_clampee_au_chargement) {
  boot();
  host::nvs["ecu_config/#ns"] = "1";
  host::nvs["ecu_config/shiftCut"] = "-40000"; host::nvs["ecu_config/slipMax"] = "0"; host::nvs["ecu_config/pitRpm"] = "999999";
  host::nvs["ecu_config/pulses"] = "abc";
  EcuConfig c = storage::loadConfig();
  CHECK(c.shiftCutMs == cfg::SHIFT_CUT_MIN && c.maxSlipPercent == cfg::SLIP_MIN && c.pitRpm == cfg::PIT_MAX);
  CHECK_MSG(c.pulsesPerRev >= cfg::PULSES_MIN && c.pulsesPerRev <= cfg::PULSES_MAX, "pulses=%d", c.pulsesPerRev);
}
TEST(config_migration_depuis_anciennes_cles_v23_0) {
  boot();
  host::nvs["ecu_config/#ns"] = "1";
  host::nvs["ecu_config/shiftCut"] = "77"; host::nvs["ecu_config/pitRpm"] = "4500"; host::nvs["ecu_config/slipMax"] = "22";
  EcuConfig c = storage::loadConfig();
  CHECK(c.shiftCutMs == 77 && c.pitRpm == 4500 && c.maxSlipPercent == 22);
  storage::saveConfig(c);                                      // ecrit le blob
  host::nvs["ecu_config/shiftCut"] = "11";                     // l'ancienne cle ne doit plus compter
  CHECK(storage::loadConfig().shiftCutMs == 77);
}
TEST(config_migration_blob_v1_sans_pitKmh) {
  // Blob ecrit par le firmware v23.x (7 valeurs, version 1) : doit etre relu, pitKmh = defaut.
  boot();
  struct V1 { uint16_t magic; uint16_t version; int32_t v[7]; uint32_t crc; } b;
  memset(&b, 0, sizeof(b)); b.magic = 0xEC01; b.version = 1;
  const int32_t vals[7] = {77, 6500, 8500, 18, 1750, 4, 4500};
  for (int i = 0; i < 7; ++i) b.v[i] = vals[i];
  uint32_t c = 0xFFFFFFFFu; const uint8_t* p = (const uint8_t*)&b;
  for (size_t i = 0; i < offsetof(V1, crc); ++i) { c ^= p[i]; for (int k = 0; k < 8; ++k) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u))); }
  b.crc = ~c;
  Preferences pr; pr.begin("ecu_config", false); pr.putBytes("cfg1", &b, sizeof(b)); pr.end();
  EcuConfig r = storage::loadConfig();
  CHECK(r.shiftCutMs == 77 && r.minShiftRpm == 6500 && r.pitRpm == 4500 && r.maxSlipPercent == 18);
  CHECK(r.pitKmh == cfg::DEFAULTS.pitKmh);
}
TEST(config_blob_corrompu_detecte_par_crc) {
  boot();
  EcuConfig c = {70, 7000, 9000, 20, 1750, 4, 5000};
  storage::saveConfig(c);
  std::string& h = host::nvs["ecu_config/#b/cfg1"];
  h[10] = (h[10] == 'a') ? 'b' : 'a';                          // un bit/quartet flippe
  EcuConfig r = storage::loadConfig();
  CHECK_MSG(!(r.shiftCutMs == 70 && r.minShiftRpm == 7000 && r.launchRpm == 9000), "blob corrompu accepte");
  CHECK(r.shiftCutMs >= cfg::SHIFT_CUT_MIN && r.shiftCutMs <= cfg::SHIFT_CUT_MAX);
}
TEST(config_coupure_courant_pendant_sauvegarde) {
  // Coupure de courant apres k ecritures NVS (k = 0..6) : l'etat relu doit toujours etre valide,
  // et on mesure combien de cas donnent une carto MIXTE (ancienne + nouvelle).
  int mixed = 0, invalid = 0;
  for (int k = 0; k <= 7; ++k) {
    host::nvs.clear(); host::nvsWrites = 0; host::nvsFailAfter = -1; boot();
    EcuConfig oldc = {50, 6000, 8000, 15, 1800, 3, 4000, 60}, newc = {90, 9000, 12000, 30, 1900, 5, 8000, 40};
    storage::saveConfig(oldc);
    host::nvsWrites = 0; host::nvsFailAfter = k;
    try { storage::saveConfig(newc); } catch (host::PowerLoss&) {}
    host::nvsFailAfter = -1;
    EcuConfig r = storage::loadConfig();
    bool anyNew = r.shiftCutMs == 90 || r.minShiftRpm == 9000 || r.launchRpm == 12000 || r.maxSlipPercent == 30 || r.pitRpm == 8000 || r.wheelCircumMm == 1900 || r.pulsesPerRev == 5;
    bool anyOld = r.shiftCutMs == 50 || r.minShiftRpm == 6000 || r.launchRpm == 8000 || r.maxSlipPercent == 15 || r.pitRpm == 4000 || r.wheelCircumMm == 1800 || r.pulsesPerRev == 3;
    if (anyNew && anyOld) ++mixed;
    EcuConfig s = storage::sanitize(r);
    if (s.shiftCutMs != r.shiftCutMs || s.pitRpm != r.pitRpm) ++invalid;
  }
  INFO("coupures de courant simulees: %d cas sur 8 donnent une carto mixte", mixed);
  CHECK_MSG(invalid == 0, "etat relu hors bornes");
  CHECK_MSG(mixed == 0, "(point ouvert AUDIT.md) %d/8 coupures laissent une carto MIXTE: ecriture non atomique", mixed);
}

// ============================ parseIntStrict ===================================================
TEST(parse_int_strict) {
  int o = -1;
  CHECK(parseIntStrict("123", 0, 1000, o) && o == 123);
  CHECK(parseIntStrict("0", 0, 10, o) && o == 0);
  CHECK(parseIntStrict("-5", -10, 10, o) && o == -5);
  CHECK(parseIntStrict("1000", 10, 1000, o) && o == 1000);
  o = 77;
  CHECK(!parseIntStrict("", 0, 100, o));      CHECK(!parseIntStrict("abc", 0, 100, o));
  CHECK(!parseIntStrict("12x", 0, 100, o));   CHECK(!parseIntStrict(" 12", 0, 100, o));
  CHECK(!parseIntStrict("1.5", 0, 100, o));   CHECK(!parseIntStrict("-", 0, 100, o));
  CHECK(!parseIntStrict("101", 0, 100, o));   CHECK(!parseIntStrict("-1", 0, 100, o));
  CHECK(!parseIntStrict("99999999", 0, 2000000000, o));   CHECK(!parseIntStrict("+5", 0, 100, o));
  CHECK_MSG(o == 77, "sortie modifiee sur entree invalide");
}

// ============================ Licence ==========================================================
static std::string runPy(const std::string& cmd) {
  std::string out; FILE* f = popen(cmd.c_str(), "r"); if (!f) return out;
  char b[256]; while (fgets(b, sizeof(b), f)) out += b; pclose(f); return out;
}
TEST(licence_cles_identiques_a_tools_make_key_py) {
  boot(); host::wifiMac = "AA:BB:CC:DD:EE:FF";
  putenv((std::string("APTATIO_SECRET=") + LICENSE_HMAC_SECRET).c_str());
  const char* py = getenv("HOST_TEST_PYTHON");
  std::string out = runPy(std::string("\"") + (py ? py : "python") + "\" tools/make_key.py AA:BB:CC:DD:EE:FF --all 2>&1");
  for (int t = 0; t < 3; ++t) {
    std::string key = license::expectedKey((LicenseTier)t).c_str();
    CHECK_MSG(out.find(key) != std::string::npos, "tier %d: firmware=%s ; outil Python=\n%s", t, key.c_str(), out.c_str());
  }
}
TEST(licence_cles_distinctes_par_tier_et_par_carte) {
  boot(); host::wifiMac = "AA:BB:CC:DD:EE:FF";
  std::string k0 = license::expectedKey(LicenseTier::Basic).c_str(), k1 = license::expectedKey(LicenseTier::Track).c_str(), k2 = license::expectedKey(LicenseTier::Pro).c_str();
  CHECK(k0 != k1 && k1 != k2 && k0 != k2);
  CHECK(k1.size() == LICENSE_KEY_HEX_LEN);
  host::wifiMac = "AA:BB:CC:DD:EE:00";
  CHECK(std::string(license::expectedKey(LicenseTier::Track).c_str()) != k1);
}
TEST(licence_defaut_basic) { boot(); license::begin(); CHECK(license::tier() == LicenseTier::Basic); }
TEST(licence_deblocage_persistance_et_redemarrage) {
  boot(); uint32_t retry = 0;
  CHECK(license::tryUnlock(license::expectedKey(LicenseTier::Track), retry));
  CHECK(license::tier() == LicenseTier::Track);
  license::begin();                                      // "redemarrage"
  CHECK(license::tier() == LicenseTier::Track);
  CHECK(license::tryUnlock(license::expectedKey(LicenseTier::Pro), retry));
  license::begin(); CHECK(license::tier() == LicenseTier::Pro);
}
TEST(licence_minuscules_et_espaces_acceptes) {
  boot(); uint32_t retry = 0;
  std::string k = license::expectedKey(LicenseTier::Pro).c_str();
  for (auto& c : k) c = (char)tolower((unsigned char)c);
  CHECK(license::tryUnlock(String("  ") + k.c_str() + " \n", retry));
  CHECK(license::tier() == LicenseTier::Pro);
}
TEST(licence_mauvaises_cles_refusees_et_verrouillage) {
  boot(); uint32_t retry = 0;
  for (int i = 0; i < 7; ++i) { CHECK(!license::tryUnlock("0000000000000000", retry)); CHECK(retry == 0); }
  CHECK(!license::tryUnlock("0000000000000000", retry));
  CHECK_MSG(retry == cfg::LICENSE_LOCKOUT_MS, "8e echec: verrou attendu, retry=%u", retry);
  // pendant le verrou, meme la bonne cle est refusee
  CHECK(!license::tryUnlock(license::expectedKey(LicenseTier::Pro), retry));
  CHECK(retry > 0 && license::tier() == LicenseTier::Basic);
  host::runFor(MS(cfg::LICENSE_LOCKOUT_MS + 1000));
  CHECK(license::tryUnlock(license::expectedKey(LicenseTier::Pro), retry));
  CHECK(license::tier() == LicenseTier::Pro);
}
TEST(licence_formats_invalides) {
  boot(); uint32_t retry = 0;
  CHECK(!license::tryUnlock("", retry)); CHECK(!license::tryUnlock("ABC", retry));
  CHECK(!license::tryUnlock(String(license::expectedKey(LicenseTier::Pro)) + "0", retry));
  CHECK(license::tier() == LicenseTier::Basic);
}
TEST(licence_cle_d_une_autre_carte_refusee) {
  boot(); host::wifiMac = "11:22:33:44:55:66";
  String foreign = license::expectedKey(LicenseTier::Pro);
  host::wifiMac = "AA:BB:CC:DD:EE:FF"; uint32_t retry = 0;
  CHECK(!license::tryUnlock(foreign, retry));
}
TEST(licence_nvs_falsifiee_ne_debloque_rien) {
  boot(); host::wifiMac = "AA:BB:CC:DD:EE:FF";
  host::nvs["ecu_config/#ns"] = "1";
  host::nvs["ecu_config/licenseTier"] = "2";                          // tier ecrit a la main, sans cle
  license::begin(); CHECK(license::tier() == LicenseTier::Basic);
  host::nvs["ecu_config/licenseKey"] = license::expectedKey(LicenseTier::Track).c_str();   // cle TRACK mais tier=2
  license::begin(); CHECK_MSG(license::tier() == LicenseTier::Basic, "cle TRACK + tier PRO ecrit a la main a debloque PRO");
  host::nvs["ecu_config/licenseTier"] = "9";
  license::begin(); CHECK(license::tier() == LicenseTier::Basic);
  host::nvs["ecu_config/licenseTier"] = "2";
  host::nvs["ecu_config/licenseKey"] = license::expectedKey(LicenseTier::Pro).c_str();
  license::begin(); CHECK(license::tier() == LicenseTier::Pro);
}
TEST(licence_carte_clonee_flash_sur_autre_mac) {
  // On copie la NVS d'une carte licenciee PRO sur une autre carte (MAC differente) : doit retomber en BASIC.
  boot(); host::wifiMac = "AA:BB:CC:DD:EE:FF"; uint32_t retry = 0;
  CHECK(license::tryUnlock(license::expectedKey(LicenseTier::Pro), retry));
  host::wifiMac = "10:20:30:40:50:60";
  license::begin();
  CHECK(license::tier() == LicenseTier::Basic);
}

// ============================ Reseau ============================================================
TEST(reseau_mot_de_passe_ap_genere_persiste_et_reutilise) {
  boot(); net::begin();
  String pw = net::apPassword();
  CHECK_MSG(pw.length() == 12, "longueur %zu", pw.length());
  for (size_t i = 0; i < pw.length(); ++i) CHECK_MSG(!strchr("0O1lI", pw[i]), "caractere ambigu '%c'", pw[i]);
  CHECK(WiFi.apUp && WiFi.lastPass == pw);
  CHECK(storage::loadWifiPassword() == pw);
  net::begin();                                              // 2e boot
  CHECK(net::apPassword() == pw);
}
TEST(reseau_ssid_et_hwid) {
  boot(); host::wifiMac = "AA:BB:CC:11:22:33"; net::begin();
  CHECK(net::hwid() == "AA:BB:CC:11:22:33");
  CHECK(net::ssid() == "AptatioECU-112233");
}
TEST(reseau_rotation_mot_de_passe_differe) {
  boot(); net::begin(); String before = net::apPassword();
  String np; net::rotateApPassword(np);
  CHECK(np.length() == 12 && np != before);
  CHECK(storage::loadWifiPassword() == np);
}
TEST(reseau_mots_de_passe_aleatoires_tous_differents) {
  boot(); std::map<std::string, int> seen;
  for (int i = 0; i < 200; ++i) { String p; net::rotateApPassword(p); seen[p.c_str()]++; }
  CHECK_MSG(seen.size() == 200, "collisions de mots de passe: %zu uniques sur 200", seen.size());
}
TEST(reseau_reboot_differe_exactement_apres_le_delai) {
  boot(); net::requestReboot(800);
  CHECK(net::rebootPending());
  host::runFor(MS(700)); net::serviceReboot(); CHECK(host::restartCount == 0);
  host::runFor(MS(200));
  bool restarted = false;
  try { net::serviceReboot(); } catch (host::RestartException&) { restarted = true; }
  CHECK(restarted);
}
