// SRC: core/SharedState.cpp persistence/Storage.cpp persistence/License.cpp net/Network.cpp net/WebServer.cpp sim/SimEngine.cpp
//
// Fuite memoire et robustesse du serveur web : des dizaines de milliers de requetes variees (valides, invalides,
// non authentifiees, licences, OTA) ; le tas du firmware ne doit pas croitre.
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

typedef std::map<std::string, std::string> M;
static std::string g_pin;

static int call(const char* path, const M& post = M(), const M& hdr = M(), std::string* bodyOut = nullptr) {
  auto* r = new AsyncWebServerRequest(); r->post = post; r->hdr = hdr;
  hostweb::routes().at(path).h(r);
  const int c = r->sent ? r->sent->code : -1;
  if (bodyOut && r->sent) *bodyOut = r->sent->body;
  delete r->sent; delete r;                 // la bibliotheque reelle libere la reponse : le banc aussi
  return c;
}
static std::string login() {
  std::string b; call("/api/login", {{"pin", g_pin}}, M(), &b);
  size_t p = b.find("\"token\":\""); return p == std::string::npos ? "" : b.substr(p + 9, 32);
}

TEST(stress_web_40000_requetes_sans_fuite) {
  host::nowUs = SEC(10);
  state::begin(); storage::begin(); state::setConfig(cfg::DEFAULTS);
  net::begin(); host::serialOut.clear(); web::begin();
  g_pin = host::serialOut.substr(host::serialOut.find("CONFIG PIN: ") + 12, 6);
  host::serialOut.reserve(1 << 20);
  std::string tok = login();
  long base = -1, baseAllocs = -1; long okCfg = 0, bad = 0, unauth = 0;
  const char* junk[] = {"", "abc", "-1", "999999999", "1.5", " 7", "0", "%00", "\xff\xfe", "9999999999999"};
  for (int i = 0; i < 40000; ++i) {
    host::nowUs += 1000000;                                   // 1 s par requete : traverse les TTL de jeton
    const int k = (int)(host::rnd() * 9);
    switch (k) {
      case 0: call("/"); break;
      case 1: call("/api/state"); break;
      case 2: { std::string t = login(); if (!t.empty()) tok = t; break; }
      case 3: call("/api/login", {{"pin", junk[(int)(host::rnd() * 10)]}}); break;
      case 4: { int c = call("/api/config", {{"shiftCutMs", std::to_string(10 + (int)(host::rnd() * 140))}, {"pitKmh", std::to_string(10 + (int)(host::rnd() * 90))}}, {{"Authorization", "Bearer " + tok}}); if (c == 200) ++okCfg; break; }
      case 5: { int c = call("/api/config", {{"shiftCutMs", junk[(int)(host::rnd() * 10)]}}, {{"Authorization", "Bearer " + tok}}); if (c == 400) ++bad; break; }
      case 6: { int c = call("/api/config", {{"shiftCutMs", "50"}}); if (c == 401) ++unauth; break; }
      case 7: call("/api/license", {{"key", "0000000000000000"}}); break;
      case 8: { std::string b; call("/api/wifi", M(), {{"Authorization", "Bearer " + tok}}, &b); break; }
    }
    state::setRpmFiltered(0); state::setRpmInstant(0);        // moteur a l'arret
    if (i == 2000) { base = host::liveBytes(); baseAllocs = host::liveAllocs(); }
    if (i % 500 == 0) host::serialOut.clear();
  }
  const long growth = host::liveBytes() - base, allocs = host::liveAllocs() - baseAllocs;
  INFO("40000 requetes : config ok %ld, refusees %ld, 401 %ld ; tas %+ld octets, %+ld allocations ; ecritures NVS %d", okCfg, bad, unauth, growth, allocs, host::nvsWrites);
  CHECK_MSG(growth <= 512 && allocs <= 8, "FUITE ? %+ld octets, %+ld allocations", growth, allocs);
  CHECK(okCfg > 1000 && bad > 1000 && unauth > 1000);
  // Les refus ne doivent jamais avoir modifie la config
  EcuConfig c = state::getConfig();
  CHECK(c.shiftCutMs >= cfg::SHIFT_CUT_MIN && c.shiftCutMs <= cfg::SHIFT_CUT_MAX && c.pitKmh >= cfg::PITKMH_MIN && c.pitKmh <= cfg::PITKMH_MAX);
}
