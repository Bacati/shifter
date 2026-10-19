// SRC: core/SharedState.cpp persistence/Storage.cpp persistence/License.cpp net/Network.cpp net/WebServer.cpp sim/SimEngine.cpp
// Batterie securite / abonnements : un client BASIC -> TRACK -> PRO doit garder son niveau quoi
// qu'il arrive (coupure de courant, redemarrages, longues periodes sans usage, NVS abimee), et
// personne ne doit pouvoir obtenir un niveau qu'il n'a pas achete.
#include "host.h"
#include "mock/Arduino.h"
#include "mock/WiFi.h"
#include "mock/ESPAsyncWebServer.h"
#include "mock/Update.h"
#include "src/core/Config.h"
#include "src/core/SharedState.h"
#include "mock/Preferences.h"
#include "src/persistence/Storage.h"
#include "src/persistence/License.h"
#include "src/persistence/secrets.h"
#include "src/net/Network.h"
#include "src/net/WebServer.h"
#include "src/sim/SimEngine.h"
#include <set>

typedef std::map<std::string, std::string> M;
static const char* const MAC_A = "AA:BB:CC:DD:EE:FF";
static const LicenseTier TIERS[3] = {LicenseTier::Basic, LicenseTier::Track, LicenseTier::Pro};

static void boot() { host::nowUs = SEC(10); state::begin(); storage::begin(); state::setConfig(cfg::DEFAULTS); host::wifiMac = MAC_A; }
static String keyFor(LicenseTier t) { return license::expectedKey(t); }
static void unlock(LicenseTier t) { uint32_t r = 0; CHECK(license::tryUnlock(keyFor(t), r)); }
static std::string rawNvsLic() { auto it = host::nvs.find("ecu_config/lic"); return it == host::nvs.end() ? "" : it->second; }

// ---- helpers web (memes que test_web.cpp) ----
static std::string g_pin;
static void bootWeb() {
  boot(); net::begin(); host::serialOut.clear(); web::begin();
  size_t p = host::serialOut.find("CONFIG PIN: "); g_pin = host::serialOut.substr(p + 12, 6);
}
static AsyncWebServerRequest* call(const char* path, M post = M(), M hdr = M()) {
  auto* r = new AsyncWebServerRequest(); r->post = post; r->hdr = hdr; hostweb::routes().at(path).h(r); return r;
}
static int code(AsyncWebServerRequest* r) { return r->sent ? r->sent->code : -1; }
static std::string body(AsyncWebServerRequest* r) { return r->sent ? r->sent->body : ""; }
static M bearer() {
  auto* r = call("/api/login", {{"pin", g_pin}}); std::string b = body(r); size_t p = b.find("\"token\":\"");
  return {{"Authorization", "Bearer " + b.substr(p + 9, 32)}};
}
static int webUnlock(LicenseTier t) { return code(call("/api/license", {{"key", std::string(keyFor(t).c_str())}})); }

// ============================ 1. Coupure de courant / persistance =================================================
TEST(licsec_coupure_pendant_upgrade_ne_fait_jamais_perdre_un_niveau) {
  // Pour chaque nombre d'ecritures NVS accordees avant la coupure (0..4) et chaque transition, l'etat relu
  // au redemarrage doit etre l'ANCIEN niveau ou le NOUVEAU, jamais un niveau inferieur.
  const LicenseTier from[] = {LicenseTier::Basic, LicenseTier::Basic, LicenseTier::Track};
  const LicenseTier to[]   = {LicenseTier::Track, LicenseTier::Pro,   LicenseTier::Pro};
  for (int c = 0; c < 3; ++c)
    for (int k = 0; k <= 4; ++k) {
      host::nvs.clear(); host::nvsWrites = 0; host::nvsFailAfter = -1; boot(); license::begin();   // RAM remise a BASIC (NVS vide)
      if (from[c] != LicenseTier::Basic) unlock(from[c]);
      host::nvsWrites = 0; host::nvsFailAfter = k;
      uint32_t retry = 0;
      try { license::tryUnlock(keyFor(to[c]), retry); } catch (host::PowerLoss&) {}
      host::nvsFailAfter = -1;
      license::begin();
      const LicenseTier got = license::tier();
      CHECK_MSG(got == from[c] || got == to[c], "transition %d->%d coupee apres %d ecriture(s) : niveau relu = %d (perte de licence)",
                (int)from[c], (int)to[c], k, (int)got);
    }
}
TEST(licsec_activation_ecrit_un_seul_item_nvs) {
  boot(); host::nvsWrites = 0; unlock(LicenseTier::Pro);
  CHECK_MSG(host::nvsWrites == 1, "%d ecritures NVS pour une activation (atomicite = 1 seul item)", host::nvsWrites);
}
TEST(licsec_1000_redemarrages_gardent_le_niveau_sans_toucher_la_nvs) {
  for (LicenseTier t : {LicenseTier::Track, LicenseTier::Pro}) {
    boot(); unlock(t); host::nvsWrites = 0;
    for (int i = 0; i < 1000; ++i) { license::begin(); CHECK_MSG(license::tier() == t, "reboot %d : tier %d", i, (int)license::tier()); }
    CHECK_MSG(host::nvsWrites == 0, "begin() ecrit la NVS (%d ecritures) : usure flash / risque de perte", host::nvsWrites);
  }
}
TEST(licsec_ancien_format_nvs_deux_items_toujours_lu_puis_remplace) {
  // Carte deja deployee (firmware <= v23.x) : licenseKey + licenseTier. Doit rester PRO, et l'upgrade ne doit pas etre masque.
  boot(); host::nvs["ecu_config/#ns"] = "1";
  host::nvs["ecu_config/licenseKey"] = keyFor(LicenseTier::Track).c_str(); host::nvs["ecu_config/licenseTier"] = "1";
  license::begin(); CHECK(license::tier() == LicenseTier::Track);
  unlock(LicenseTier::Pro); license::begin(); CHECK(license::tier() == LicenseTier::Pro);
  CHECK(!rawNvsLic().empty());
  host::nvs["ecu_config/licenseKey"] = keyFor(LicenseTier::Track).c_str(); host::nvs["ecu_config/licenseTier"] = "1";   // l'ancien item perime ne doit pas reprendre le dessus
  license::begin(); CHECK_MSG(license::tier() == LicenseTier::Pro, "l'ancien format a ecrase le nouveau");
}
TEST(licsec_coupures_pendant_config_ou_wifi_ne_touchent_pas_la_licence) {
  for (int k = 0; k <= 3; ++k) {
    host::nvs.clear(); host::nvsFailAfter = -1; boot(); unlock(LicenseTier::Pro);
    host::nvsWrites = 0; host::nvsFailAfter = k;
    try { EcuConfig c = cfg::DEFAULTS; c.shiftCutMs = 70; storage::saveConfig(c); storage::saveWifiPassword("abcdefghijkl"); } catch (host::PowerLoss&) {}
    host::nvsFailAfter = -1; license::begin();
    CHECK_MSG(license::tier() == LicenseTier::Pro, "coupure apres %d ecriture(s) de config : licence perdue", k);
  }
}
TEST(licsec_glitch_mac_au_boot_ne_detruit_rien_et_se_reguerit) {
  // Si WiFi.macAddress() renvoyait une valeur fausse (WiFi pas pret) au boot : BASIC pour ce boot, mais la licence
  // stockee doit rester intacte et revenir au boot suivant.
  boot(); unlock(LicenseTier::Pro); const M before = host::nvs; host::nvsWrites = 0;
  host::wifiMac = "00:00:00:00:00:00"; license::begin();
  CHECK(license::tier() == LicenseTier::Basic);
  CHECK_MSG(host::nvs == before && host::nvsWrites == 0, "la NVS a ete modifiee pendant le boot en BASIC (licence detruite)");
  host::wifiMac = MAC_A; license::begin();
  CHECK_MSG(license::tier() == LicenseTier::Pro, "la licence ne revient pas apres un boot normal");
}
TEST(licsec_longue_inactivite_pro_reste_pro) {
  // Aucun mecanisme d'expiration : on saute 1 jour, 30 j, 49,7 j (debordement millis), 400 j, 10 ans.
  const double days[] = {1, 30, 49.7103, 49.7104, 400, 3650};
  boot(); unlock(LicenseTier::Pro);
  for (double d : days) {
    host::nowUs += (uint64_t)(d * 86400.0 * 1e6);
    host::baseMs += 123456789u; host::baseUs += 987654321u;
    license::begin();
    CHECK_MSG(license::tier() == LicenseTier::Pro, "apres %.1f jours : tier %d", d, (int)license::tier());
    uint32_t retry = 0; CHECK(license::tryUnlock(keyFor(LicenseTier::Pro), retry));
    CHECK(license::tier() == LicenseTier::Pro);
  }
}
TEST(licsec_aucune_notion_de_temps_dans_le_code_licence) {
  // Garde-fou de conception : la licence ne depend d'aucune horloge. Si quelqu'un ajoute une expiration, ce test doit etre revu.
  boot(); unlock(LicenseTier::Track);
  for (uint64_t base : {0ull, 1ull << 31, (1ull << 32) - 5, 1ull << 40}) {
    host::baseMs = (uint64_t)(base & 0xFFFFFFFFu); host::baseUs = 0; license::begin();
    CHECK_MSG(license::tier() == LicenseTier::Track, "horloge a %llu : tier %d", (unsigned long long)base, (int)license::tier());
  }
}
TEST(licsec_parcours_client_basic_track_pro_avec_redemarrages_intercales) {
  boot(); license::begin(); CHECK(license::tier() == LicenseTier::Basic);
  unlock(LicenseTier::Track); license::begin(); CHECK(license::tier() == LicenseTier::Track);
  host::nowUs += SEC(86400.0 * 120); license::begin(); CHECK(license::tier() == LicenseTier::Track);   // 4 mois sans l'utiliser
  unlock(LicenseTier::Pro);   license::begin(); CHECK(license::tier() == LicenseTier::Pro);
  host::nowUs += SEC(86400.0 * 400); for (int i = 0; i < 50; ++i) { license::begin(); CHECK(license::tier() == LicenseTier::Pro); }
}

// ============================ 2. Pas de baisse de niveau par erreur ===============================================
TEST(licsec_ancienne_cle_inferieure_ne_retrograde_pas) {
  boot(); unlock(LicenseTier::Pro); const std::string lic = rawNvsLic();
  uint32_t retry = 0;
  CHECK(license::tryUnlock(keyFor(LicenseTier::Track), retry));   // cle valide, acceptee
  CHECK_MSG(license::tier() == LicenseTier::Pro, "re-saisie d'une cle TRACK : le PRO est perdu (tier %d)", (int)license::tier());
  CHECK(license::tryUnlock(keyFor(LicenseTier::Basic), retry));
  CHECK(license::tier() == LicenseTier::Pro);
  license::begin(); CHECK_MSG(license::tier() == LicenseTier::Pro && rawNvsLic() == lic, "NVS modifiee par la re-saisie d'une cle inferieure");
}
TEST(licsec_meme_cle_resaisie_est_idempotente) {
  boot(); unlock(LicenseTier::Pro); host::nvsWrites = 0;
  for (int i = 0; i < 20; ++i) { uint32_t r = 0; CHECK(license::tryUnlock(keyFor(LicenseTier::Pro), r)); }
  CHECK_MSG(host::nvsWrites == 0, "%d ecritures NVS pour la re-saisie de la meme cle", host::nvsWrites);
  CHECK(license::tier() == LicenseTier::Pro);
}
TEST(licsec_cle_track_ne_donne_jamais_pro_et_inversement_sur_toute_la_chaine) {
  boot(); uint32_t retry = 0;
  CHECK(license::tryUnlock(keyFor(LicenseTier::Track), retry));
  CHECK(license::tier() == LicenseTier::Track);
  license::begin(); CHECK(license::tier() == LicenseTier::Track);
  // la cle TRACK ne doit pas etre prise pour une cle PRO par aucune transformation simple
  std::string k = keyFor(LicenseTier::Track).c_str();
  std::string swapped = k.substr(8) + k.substr(0, 8), rev(k.rbegin(), k.rend());
  for (const std::string& v : {swapped, rev, std::string(16, '0'), std::string(16, 'F')}) {
    CHECK(!license::tryUnlock(String(v.c_str()), retry));
    CHECK(license::tier() == LicenseTier::Track);
    host::nowUs += SEC(400);   // evite le verrouillage entre deux essais
  }
}

// ============================ 3. NVS falsifiee / copiee ============================================================
TEST(licsec_nvs_falsifiee_nouveau_format) {
  boot(); const std::string kT = keyFor(LicenseTier::Track).c_str(), kP = keyFor(LicenseTier::Pro).c_str();
  struct Case { std::string v; LicenseTier expect; const char* what; };
  const Case cases[] = {
    {"2:" + kT, LicenseTier::Basic, "cle TRACK etiquetee PRO"},
    {"1:" + kP, LicenseTier::Basic, "cle PRO etiquetee TRACK"},
    {"2:",      LicenseTier::Basic, "PRO sans cle"},
    {"2",       LicenseTier::Basic, "tier seul"},
    {":" + kP,  LicenseTier::Basic, "sans tier"},
    {"9:" + kP, LicenseTier::Basic, "tier hors plage"},
    {"-1:" + kP, LicenseTier::Basic, "tier negatif"},
    {"22:" + kP, LicenseTier::Basic, "tier a 2 chiffres"},
    {"2:" + kP + "0", LicenseTier::Basic, "cle trop longue"},
    {"2:" + kP.substr(0, 15), LicenseTier::Basic, "cle tronquee"},
    {"2::" + kP, LicenseTier::Basic, "separateur double"},
    {"2:" + std::string(4096, 'A'), LicenseTier::Basic, "4 Ko de A"},
    {std::string(4096, ':'), LicenseTier::Basic, "4 Ko de ':'"},
    {"2:" + kP, LicenseTier::Pro, "cle PRO correcte (controle positif)"},
    {"1:" + kT, LicenseTier::Track, "cle TRACK correcte (controle positif)"},
  };
  for (const Case& c : cases) {
    host::nvs["ecu_config/#ns"] = "1"; host::nvs["ecu_config/lic"] = c.v; license::begin();
    CHECK_MSG(license::tier() == c.expect, "%s : tier %d (attendu %d)", c.what, (int)license::tier(), (int)c.expect);
  }
}
TEST(licsec_nvs_vide_ou_efface_donne_basic_sans_planter) {
  boot(); unlock(LicenseTier::Pro);
  host::nvs.clear(); license::begin(); CHECK(license::tier() == LicenseTier::Basic);
  host::nvs["ecu_config/#ns"] = "1"; license::begin(); CHECK(license::tier() == LicenseTier::Basic);
  host::nvs["ecu_config/lic"] = ""; license::begin(); CHECK(license::tier() == LicenseTier::Basic);
}
TEST(licsec_nvs_d_une_carte_copiee_sur_toutes_les_autres_cartes_donne_basic) {
  // Dump NVS d'une carte PRO flashe sur 300 autres cartes : toutes en BASIC, et une cle volee ne passe pas par l'IHM non plus.
  boot(); unlock(LicenseTier::Pro); const M dump = host::nvs; const std::string stolen = keyFor(LicenseTier::Pro).c_str();
  for (int i = 0; i < 300; ++i) {
    char mac[24]; snprintf(mac, sizeof(mac), "24:6F:28:%02X:%02X:%02X", (i * 7) & 255, (i * 13) & 255, i & 255);
    if (std::string(mac) == MAC_A) continue;
    host::wifiMac = mac; host::nvs = dump; license::begin();
    CHECK_MSG(license::tier() == LicenseTier::Basic, "carte %s : la NVS copiee d'une autre carte donne le tier %d", mac, (int)license::tier());
    uint32_t r = 0; CHECK(!license::tryUnlock(String(stolen.c_str()), r)); host::nowUs += SEC(400);
    CHECK(license::tier() == LicenseTier::Basic);
  }
}
TEST(licsec_cle_de_chaque_carte_refusee_sur_chaque_autre_carte) {
  // 40 cartes x 3 tiers : une cle n'est valide que pour (sa carte, son tier).
  std::vector<std::string> macs; std::map<std::string, std::string> keys;
  for (int i = 0; i < 40; ++i) { char m[24]; snprintf(m, sizeof(m), "24:6F:28:%02X:%02X:%02X", i * 5, 255 - i, i); macs.push_back(m); }
  boot();
  for (auto& m : macs) for (int t = 0; t < 3; ++t) { host::wifiMac = m; keys[m + "#" + char('0' + t)] = keyFor(TIERS[t]).c_str(); }
  std::set<std::string> uniq; for (auto& kv : keys) uniq.insert(kv.second);
  CHECK_MSG(uniq.size() == keys.size(), "collision de cles entre cartes/tiers : %zu uniques sur %zu", uniq.size(), keys.size());
  for (auto& a : macs) {
    host::nvs.clear(); host::wifiMac = a; license::begin(); host::nowUs += SEC(400);
    for (auto& kv : keys) {
      if (kv.first.compare(0, a.size(), a) == 0) continue;               // ses propres cles
      uint32_t r = 0; host::nowUs += SEC(400);
      CHECK_MSG(!license::tryUnlock(String(kv.second.c_str()), r), "cle de %s acceptee sur %s", kv.first.c_str(), a.c_str());
    }
    CHECK(license::tier() == LicenseTier::Basic);
  }
}

// ============================ 4. Attaques sur la saisie de cle ====================================================
TEST(licsec_mutations_d_une_bonne_cle_refusees) {
  // Chaque bit retourne, chaque caractere change, troncature, ajout, separateurs : rien ne passe.
  boot(); const std::string good = keyFor(LicenseTier::Pro).c_str(); uint32_t r = 0; long tries = 0;
  auto tryKey = [&](const std::string& s) { host::nowUs += SEC(400); ++tries; return license::tryUnlock(String(s.c_str()), r) && license::tier() == LicenseTier::Pro; };
  for (size_t i = 0; i < good.size(); ++i)
    for (const char* h = "0123456789ABCDEF"; *h; ++h) {
      if (*h == good[i]) continue;
      std::string m = good; m[i] = *h; CHECK_MSG(!tryKey(m), "mutation en %zu acceptee: %s", i, m.c_str());
    }
  for (size_t n = 0; n < good.size(); ++n) { CHECK(!tryKey(good.substr(0, n))); CHECK(!tryKey(good.substr(n + 1))); }
  CHECK(!tryKey(good + "0")); CHECK(!tryKey(good + "-")); CHECK(!tryKey("0" + good)); CHECK(!tryKey(good.substr(0, 8) + "-" + good.substr(8)));
  CHECK(!tryKey("0x" + good.substr(2))); CHECK(!tryKey(good.substr(0, 8) + " " + good.substr(8)));
  CHECK_MSG(license::tier() == LicenseTier::Basic, "une mutation a debloque le niveau %d apres %ld essais", (int)license::tier(), tries);
}
TEST(licsec_octets_speciaux_et_unicode_refuses) {
  boot(); const std::string good = keyFor(LicenseTier::Pro).c_str(); uint32_t r = 0;
  std::string withNul = good; withNul[5] = '\0';
  std::vector<std::string> bad = {withNul, std::string("\xC3\xA9") + good.substr(2), "\xEF\xBC\xA1" + good.substr(3), std::string(16, '\xFF'), std::string(300, 'A'), std::string(70000, 'A'), "%00" + good, "'; DROP TABLE--"};
  for (auto& b : bad) { host::nowUs += SEC(400); CHECK(!license::tryUnlock(String(b), r)); }
  CHECK(license::tier() == LicenseTier::Basic);
  CHECK(host::nvs.find("ecu_config/lic") == host::nvs.end());
}
TEST(licsec_verrou_anti_bruteforce_arithmetique_et_debit_maximal) {
  // Debit maximal d'un attaquant : LICENSE_MAX_FAILS essais par LICENSE_LOCKOUT_MS, meme en le laissant s'acharner 30 jours.
  boot(); uint32_t r = 0; long attempts = 0; const uint64_t t0 = host::nowUs;
  while (host::nowUs - t0 < SEC(86400.0 * 30)) {
    if (license::tryUnlock("0123456789ABCDEF", r)) { CHECK(false); break; }
    ++attempts;
    if (r) host::nowUs += MS(r);           // l'attaquant attend la fin du verrou
    host::nowUs += MS(5);
  }
  const double perDayMax = (double)cfg::LICENSE_MAX_FAILS * 86400000.0 / cfg::LICENSE_LOCKOUT_MS;
  INFO("%ld essais en 30 jours (%.0f/jour, plafond theorique %.0f/jour)", attempts, attempts / 30.0, perDayMax);
  CHECK_MSG(attempts <= perDayMax * 31.0, "%ld essais en 30 jours : verrou contourne", attempts);
  // 2^64 cles, 3 tiers testes par essai : annees necessaires pour 50 % de chances
  const double yearsHalf = std::pow(2.0, 63.0) / 3.0 / (perDayMax * 365.0);
  INFO("temps moyen pour deviner une cle par essais en ligne : %.3g annees", yearsHalf);
  CHECK(yearsHalf > 1e9);
}
TEST(licsec_verrou_pas_de_resurrection_ni_de_reduction_par_bonne_cle_pendant_le_verrou) {
  boot(); uint32_t r = 0;
  for (int i = 0; i < (int)cfg::LICENSE_MAX_FAILS; ++i) license::tryUnlock("BAD", r);
  CHECK(r == cfg::LICENSE_LOCKOUT_MS);
  for (int i = 0; i < 50; ++i) { host::nowUs += SEC(1); CHECK(!license::tryUnlock(keyFor(LicenseTier::Pro), r)); CHECK(r > 0 && r <= cfg::LICENSE_LOCKOUT_MS); }
  CHECK(license::tier() == LicenseTier::Basic);
  host::nowUs += SEC(cfg::LICENSE_LOCKOUT_MS / 1000.0); CHECK(license::tryUnlock(keyFor(LicenseTier::Pro), r));
}
TEST(licsec_verrou_correct_au_passage_du_debordement_de_millis) {
  boot(); uint32_t r = 0;
  host::baseMs = 0xFFFFFFFFu - 100000u;       // millis() deborde 100 s apres le debut
  for (int i = 0; i < (int)cfg::LICENSE_MAX_FAILS; ++i) license::tryUnlock("BAD", r);
  CHECK(r == cfg::LICENSE_LOCKOUT_MS);
  host::nowUs += SEC(150);                     // millis a deborde, verrou toujours actif
  CHECK_MSG(!license::tryUnlock(keyFor(LicenseTier::Pro), r) && r > 0, "verrou perdu au debordement de millis");
  host::nowUs += SEC(200);
  CHECK(license::tryUnlock(keyFor(LicenseTier::Pro), r));
}
TEST(licsec_echecs_n_ecrivent_rien_et_ne_changent_aucun_etat) {
  boot(); unlock(LicenseTier::Track); const M before = host::nvs; host::nvsWrites = 0; uint32_t r = 0;
  for (int i = 0; i < 2000; ++i) { host::nowUs += SEC(400); license::tryUnlock(String("K") + String(i), r); }
  CHECK(host::nvs == before && host::nvsWrites == 0);
  CHECK(license::tier() == LicenseTier::Track);
}
TEST(licsec_fuzz_300000_cles_aleatoires_aucun_succes_aucune_fuite_memoire) {
  boot(); uint32_t r = 0; license::tryUnlock("x", r); const long a0 = host::liveAllocs(), b0 = host::liveBytes(); long wins = 0;
  for (long i = 0; i < 300000; ++i) {
    std::string s; const int len = (i % 5 == 0) ? 16 : (int)(host::rnd() * 40);
    for (int k = 0; k < len; ++k) s += (i % 3 == 0) ? "0123456789ABCDEF"[(int)(host::rnd() * 16)] : (char)(1 + (int)(host::rnd() * 254));
    host::nowUs += MS(1500) + (r ? MS(r) : 0);
    if (license::tryUnlock(String(s), r)) ++wins;
  }
  CHECK_MSG(wins == 0, "%ld cles aleatoires acceptees", wins);
  CHECK(license::tier() == LicenseTier::Basic);
  CHECK_MSG(host::liveAllocs() <= a0 && host::liveBytes() <= b0, "fuite memoire : +%ld allocations, +%ld octets", host::liveAllocs() - a0, host::liveBytes() - b0);
}
TEST(licsec_qualite_des_cles_uniformes_et_avalanche) {
  // Aucune structure exploitable : 2000 cartes consecutives -> cles toutes differentes, chiffres hex uniformes, ~50 % de bits differents entre MAC voisines.
  boot(); std::set<std::string> uniq; long hist[16] = {0}, total = 0; double bitDiff = 0; std::string prev;
  for (int i = 0; i < 2000; ++i) {
    char m[24]; snprintf(m, sizeof(m), "24:6F:28:00:%02X:%02X", (i >> 8) & 255, i & 255);
    host::wifiMac = m; std::string k = keyFor(LicenseTier::Pro).c_str(); uniq.insert(k);
    for (char c : k) { hist[(c <= '9') ? c - '0' : c - 'A' + 10]++; ++total; }
    if (!prev.empty()) for (int j = 0; j < 16; ++j) { int a = (prev[j] <= '9') ? prev[j] - '0' : prev[j] - 'A' + 10, b = (k[j] <= '9') ? k[j] - '0' : k[j] - 'A' + 10; bitDiff += __builtin_popcount(a ^ b); }
    prev = k;
  }
  CHECK(uniq.size() == 2000);
  for (int d = 0; d < 16; ++d) CHECK_MSG(hist[d] > total / 16 * 0.85 && hist[d] < total / 16 * 1.15, "chiffre %X : %ld occurrences sur %ld", d, hist[d], total);
  const double frac = bitDiff / (1999.0 * 64.0);
  CHECK_MSG(frac > 0.47 && frac < 0.53, "avalanche : %.3f des bits changent entre deux MAC voisines", frac);
}

// ============================ 5. Parcours web (client reel) ========================================================
TEST(licsec_web_parcours_basic_track_pro_droits_exacts_a_chaque_etape) {
  bootWeb(); M auth = bearer();
  CHECK(code(call("/api/config", {{"launchRpm", "9000"}}, auth)) == 403);
  CHECK(code(call("/api/config", {{"maxSlipPercent", "20"}}, auth)) == 403);
  CHECK(code(call("/api/config", {{"shiftCutMs", "60"}}, auth)) == 200);           // BASIC garde le reglage de base
  CHECK(webUnlock(LicenseTier::Track) == 200);
  license::begin();                                                                // redemarrage
  CHECK(code(call("/api/config", {{"launchRpm", "9000"}}, auth)) == 200);
  CHECK(code(call("/api/config", {{"maxSlipPercent", "20"}}, auth)) == 403);
  CHECK(webUnlock(LicenseTier::Pro) == 200); license::begin();
  CHECK(code(call("/api/config", {{"maxSlipPercent", "20"}}, auth)) == 200);
  const EcuConfig c = state::getConfig(); CHECK(c.launchRpm == 9000 && c.maxSlipPercent == 20 && c.shiftCutMs == 60);
  // 6 mois plus tard, apres redemarrage : memes droits
  host::nowUs += SEC(86400.0 * 180); license::begin();
  CHECK(code(call("/api/config", {{"maxSlipPercent", "25"}, {"launchRpm", "8500"}}, auth)) == 200);
  CHECK(state::getConfig().maxSlipPercent == 25);
  const std::string st = body(call("/api/state")); CHECK_MSG(st.find("\"tier\":2") != std::string::npos && st.find("\"proUnlocked\":true") != std::string::npos, "%s", st.c_str());
}
TEST(licsec_web_refus_de_niveau_ne_modifie_rien) {
  bootWeb(); M auth = bearer(); const EcuConfig before = state::getConfig(); const M nvsBefore = host::nvs;
  CHECK(code(call("/api/config", {{"shiftCutMs", "90"}, {"pitKmh", "40"}, {"launchRpm", "9000"}}, auth)) == 403);
  CHECK(code(call("/api/config", {{"shiftCutMs", "90"}, {"maxSlipPercent", "20"}}, auth)) == 403);
  const EcuConfig a = state::getConfig();
  CHECK_MSG(a.shiftCutMs == before.shiftCutMs && a.pitKmh == before.pitKmh && a.launchRpm == before.launchRpm, "un refus 403 a quand meme modifie la config");
  CHECK_MSG(host::nvs == nvsBefore, "un refus 403 a ecrit en NVS");
  unlock(LicenseTier::Track);
  CHECK(code(call("/api/config", {{"shiftCutMs", "90"}, {"maxSlipPercent", "20"}}, auth)) == 403);
  CHECK(state::getConfig().shiftCutMs == before.shiftCutMs && state::getConfig().maxSlipPercent == before.maxSlipPercent);
}
TEST(licsec_web_champs_de_licence_injectes_dans_les_requetes_sans_effet) {
  bootWeb(); M auth = bearer();
  for (const char* k : {"tier", "licenseTier", "license", "lic", "proUnlocked", "trackUnlocked", "tierName", "key", "licenseKey", "Tier", "LICENSETIER"}) {
    call("/api/config", {{"shiftCutMs", "60"}, {k, "2"}}, auth);
    call("/api/wifi", {{k, "2"}}, M());
    call("/api/login", {{"pin", "000000"}, {k, "2"}});
    CHECK_MSG(license::tier() == LicenseTier::Basic, "champ '%s' a change le tier", k);
    host::nowUs += SEC(100); host::nowUs += SEC(100);
  }
  CHECK(host::nvs.find("ecu_config/lic") == host::nvs.end() && host::nvs.find("ecu_config/licenseTier") == host::nvs.end());
}
TEST(licsec_web_licence_sans_cle_ou_en_double_ou_en_query) {
  bootWeb();
  CHECK(code(call("/api/license")) == 400);
  CHECK(code(call("/api/license", {{"Key", keyFor(LicenseTier::Pro).c_str()}})) == 400);   // nom de champ different
  auto* r = new AsyncWebServerRequest(); r->post["key"] = "0000000000000000"; hostweb::routes().at("/api/license").h(r);
  CHECK(code(r) == 403 && license::tier() == LicenseTier::Basic);
}
TEST(licsec_web_brute_force_http_429_puis_aucune_cle_acceptee_pendant_le_verrou) {
  bootWeb(); int ok = 0, locked = 0;
  for (int i = 0; i < 100; ++i) { int c = code(call("/api/license", {{"key", "FFFFFFFFFFFFFF" + std::string(1, "0123456789ABCDEF"[i % 16]) + "0"}})); if (c == 200) ++ok; if (c == 429) ++locked; }
  CHECK(ok == 0 && locked >= 90);
  CHECK_MSG(webUnlock(LicenseTier::Pro) == 429, "la bonne cle passe pendant le verrou");
  CHECK(license::tier() == LicenseTier::Basic && !net::rebootPending());
}
TEST(licsec_web_licence_valide_planifie_un_redemarrage_et_survit_a_la_coupure_avant_ce_redemarrage) {
  bootWeb(); CHECK(webUnlock(LicenseTier::Pro) == 200); CHECK(net::rebootPending());
  host::nvsFailAfter = -1; license::begin();       // coupure brutale avant le reboot programme : la NVS a deja tout
  CHECK(license::tier() == LicenseTier::Pro);
}
TEST(licsec_web_aucune_reponse_ne_contient_cle_secret_pin_ou_mot_de_passe) {
  bootWeb(); M auth = bearer(); unlock(LicenseTier::Pro);
  std::vector<std::string> secrets = {LICENSE_HMAC_SECRET, keyFor(LicenseTier::Basic).c_str(), keyFor(LicenseTier::Track).c_str(), keyFor(LicenseTier::Pro).c_str()};
  std::vector<AsyncWebServerRequest*> rs = {call("/"), call("/api/state"), call("/api/license", {{"key", "0"}}), call("/api/config", {{"shiftCutMs", "60"}}, auth), call("/api/config", {{"shiftCutMs", "abc"}}, auth), call("/api/login", {{"pin", "1"}}), call("/api/config")};
  for (auto* r : rs) {
    std::string all = body(r); for (auto& h : r->sent->headers) all += h.first + h.second;
    for (auto& s : secrets) CHECK_MSG(all.find(s) == std::string::npos, "secret/cle presente dans une reponse HTTP: %.40s", all.c_str());
  }
  // la cle valide ne doit pas non plus etre recopiee dans la reponse de sa propre activation
  auto* r = call("/api/license", {{"key", std::string(keyFor(LicenseTier::Pro).c_str())}}); CHECK(body(r).find(keyFor(LicenseTier::Pro).c_str()) == std::string::npos);
}
TEST(licsec_console_serie_ne_divulgue_ni_secret_ni_cles) {
  bootWeb(); unlock(LicenseTier::Pro); license::begin();
  for (int i = 0; i < 5; ++i) { uint32_t r = 0; license::tryUnlock("BADBADBADBADBAD0", r); }
  const std::string& s = host::serialOut;
  CHECK(s.find(LICENSE_HMAC_SECRET) == std::string::npos);
  for (LicenseTier t : TIERS) CHECK_MSG(s.find(keyFor(t).c_str()) == std::string::npos, "une cle de licence (tier %d) est imprimee sur la liaison serie", (int)t);
}
TEST(licsec_pas_de_mode_simulation_dans_le_firmware_de_production) {
  CHECK_MSG(sim::forcedTier() == -1, "tier force actif hors simulation");
  CHECK_MSG(sim::pinOverride() == nullptr, "PIN force actif hors simulation");
  boot(); license::begin(); CHECK(license::tier() == LicenseTier::Basic);
}
