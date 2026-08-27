#include "WebServer.h"
#include "../core/Config.h"
#include "../core/Version.h"
#include "../core/Types.h"
#include "../core/SharedState.h"
#include "../persistence/Storage.h"
#include "../persistence/License.h"
#include "Network.h"
#include "WebPage.h"
#include "../sim/SimEngine.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>
#include <string.h>

#if __has_include(<esp_random.h>)
  #include <esp_random.h>
#else
  #include <esp_system.h>
#endif

namespace {

AsyncWebServer   s_server(80);
AsyncEventSource s_events("/events");

// ---- Auth (jeton Bearer ephemere en RAM, echange contre un PIN serie) ----
char     s_pin[7]        = {0};
char     s_token[33]     = {0};
uint32_t s_tokenExp      = 0;
uint8_t  s_authFails     = 0;
uint32_t s_authLockUntil = 0;

// ---- Etat OTA (le callback d'upload ne repond pas lui-meme) --------------
bool s_otaRunning = false;
bool s_otaReject  = false;

bool ctEq(const char* a, const char* b, size_t n) {
  uint8_t d = 0;
  for (size_t i = 0; i < n; ++i) d |= static_cast<uint8_t>(a[i]) ^ static_cast<uint8_t>(b[i]);
  return d == 0;
}

void authInit() {
  const char* forced = sim::pinOverride();   // PIN fixe en mode simulation
  if (forced) {
    strncpy(s_pin, forced, sizeof(s_pin) - 1);
    s_pin[sizeof(s_pin) - 1] = '\0';
  } else {
    const uint32_t p = esp_random() % 1000000UL;
    snprintf(s_pin, sizeof(s_pin), "%06lu", static_cast<unsigned long>(p));
  }
  Serial.printf("  CONFIG PIN: %s\n", s_pin);
  Serial.println("=================================================");
}

bool authValid(AsyncWebServerRequest* r) {
  if (!s_token[0]) return false;
  if (static_cast<int32_t>(millis() - s_tokenExp) >= 0) { s_token[0] = 0; return false; }
  if (!r->hasHeader("Authorization")) return false;
  String h = r->header("Authorization");
  if (!h.startsWith("Bearer ")) return false;
  String tok = h.substring(7);
  return tok.length() == 32 && ctEq(tok.c_str(), s_token, 32);
}

void addSec(AsyncWebServerResponse* res) {
  res->addHeader("X-Content-Type-Options", "nosniff");
  res->addHeader("Cache-Control", "no-store");
  res->addHeader("Referrer-Policy", "no-referrer");
}

bool engineRunning() {
  return state::getRpmFiltered() != 0 || state::getRpmInstant() != 0;
}

int paramInt(AsyncWebServerRequest* r, const char* name, int fallback) {
  if (r->hasParam(name, true)) return r->getParam(name, true)->value().toInt();
  return fallback;
}

// -------------------------------------------------------------------- routes
void handleRoot(AsyncWebServerRequest* r) {
  AsyncWebServerResponse* res = r->beginResponse(
      200, "text/html", reinterpret_cast<const uint8_t*>(INDEX_HTML), strlen_P(INDEX_HTML));
  addSec(res);
  res->addHeader("Content-Security-Policy",
                 "default-src 'none'; style-src 'unsafe-inline'; "
                 "script-src 'unsafe-inline'; connect-src 'self'");
  r->send(res);
}

void handleState(AsyncWebServerRequest* r) {
  const EcuConfig   c = state::getConfig();
  const LicenseTier t = license::tier();

  char b[512];
  snprintf(b, sizeof(b),
    "{\"fw\":\"%s\",\"ssid\":\"%s\",\"hwid\":\"%s\",\"tier\":%d,\"tierName\":\"%s\","
    "\"trackUnlocked\":%s,\"proUnlocked\":%s,"
    "\"shiftCutMs\":%d,\"minShiftRpm\":%d,\"pitRpm\":%d,\"launchRpm\":%d,"
    "\"maxSlipPercent\":%d,\"wheelCircumMm\":%d,\"pulsesPerRev\":%d}",
    FW_VERSION, net::ssid().c_str(), net::hwid().c_str(), static_cast<int>(t), toString(t),
    (t >= LicenseTier::Track) ? "true" : "false",
    (t >= LicenseTier::Pro)   ? "true" : "false",
    c.shiftCutMs, c.minShiftRpm, c.pitRpm, c.launchRpm,
    c.maxSlipPercent, c.wheelCircumMm, c.pulsesPerRev);

  AsyncWebServerResponse* res = r->beginResponse(200, "application/json", b);
  addSec(res);
  r->send(res);
}

void handleLogin(AsyncWebServerRequest* r) {
  const uint32_t now = millis();
  if (s_authLockUntil && static_cast<int32_t>(s_authLockUntil - now) > 0) {
    AsyncWebServerResponse* res = r->beginResponse(429, "text/plain", "verrouille");
    res->addHeader("Retry-After", String((s_authLockUntil - now) / 1000 + 1));
    addSec(res);
    r->send(res);
    return;
  }
  if (!r->hasParam("pin", true)) { r->send(400, "text/plain", "pin requis"); return; }

  const String pin = r->getParam("pin", true)->value();
  if (pin.length() == 6 && ctEq(pin.c_str(), s_pin, 6)) {
    s_authFails = 0;
    static const char kHex[] = "0123456789abcdef";
    for (int i = 0; i < 32; ++i) s_token[i] = kHex[esp_random() & 0x0F];
    s_token[32] = '\0';
    s_tokenExp  = now + cfg::AUTH_TOKEN_TTL_MS;

    AsyncWebServerResponse* res = r->beginResponse(
        200, "application/json",
        String("{\"token\":\"") + s_token + "\",\"ttl\":" + String(cfg::AUTH_TOKEN_TTL_MS / 1000) + "}");
    addSec(res);
    r->send(res);
    return;
  }

  if (++s_authFails >= cfg::AUTH_MAX_FAILS) {
    s_authLockUntil = now + cfg::AUTH_LOCKOUT_MS;
    s_authFails     = 0;
  }
  r->send(403, "text/plain", "PIN invalide");
}

void handleConfig(AsyncWebServerRequest* r) {
  if (!authValid(r)) { r->send(401, "text/plain", "auth requise"); return; }

  const LicenseTier t = license::tier();
  if (r->hasParam("launchRpm", true) && t < LicenseTier::Track) {
    r->send(403, "text/plain", "launchRpm : licence TRACK requise");
    return;
  }
  if (r->hasParam("maxSlipPercent", true) && t < LicenseTier::Pro) {
    r->send(403, "text/plain", "maxSlipPercent : licence PRO requise");
    return;
  }

  EcuConfig c = state::getConfig();
  c.shiftCutMs    = paramInt(r, "shiftCutMs",    c.shiftCutMs);
  c.minShiftRpm   = paramInt(r, "minShiftRpm",   c.minShiftRpm);
  c.pitRpm        = paramInt(r, "pitRpm",        c.pitRpm);
  c.wheelCircumMm = paramInt(r, "wheelCircumMm", c.wheelCircumMm);
  c.pulsesPerRev  = paramInt(r, "pulsesPerRev",  c.pulsesPerRev);
  if (t >= LicenseTier::Track) c.launchRpm      = paramInt(r, "launchRpm",      c.launchRpm);
  if (t >= LicenseTier::Pro)   c.maxSlipPercent = paramInt(r, "maxSlipPercent", c.maxSlipPercent);

  c = storage::sanitize(c);
  state::setConfig(c);
  storage::saveConfig(c);

  AsyncWebServerResponse* res = r->beginResponse(200, "text/plain", "OK");
  addSec(res);
  r->send(res);
}

void handleLicense(AsyncWebServerRequest* r) {
  if (!r->hasParam("key", true)) { r->send(400, "text/plain", "key requise"); return; }

  uint32_t retry = 0;
  if (license::tryUnlock(r->getParam("key", true)->value(), retry)) {
    AsyncWebServerResponse* res = r->beginResponse(200, "text/plain", "OK");
    addSec(res);
    r->send(res);
    net::requestReboot(800);
  } else if (retry) {
    AsyncWebServerResponse* res = r->beginResponse(429, "text/plain", "trop d'essais");
    res->addHeader("Retry-After", String(retry / 1000 + 1));
    addSec(res);
    r->send(res);
  } else {
    AsyncWebServerResponse* res = r->beginResponse(403, "text/plain", "cle refusee");
    addSec(res);
    r->send(res);
  }
}

void handleWifi(AsyncWebServerRequest* r) {
  if (!authValid(r)) { r->send(401, "text/plain", "auth requise"); return; }
  String np;
  net::rotateApPassword(np);
  AsyncWebServerResponse* res =
      r->beginResponse(200, "application/json", String("{\"password\":\"") + np + "\"}");
  addSec(res);
  r->send(res);
  net::requestReboot(1500);
}

void handleOtaDone(AsyncWebServerRequest* r) {
  if (s_otaReject) {
    s_otaReject  = false;
    s_otaRunning = false;
    r->send(403, "text/plain", "OTA refusee (auth ou moteur en marche)");
    return;
  }
  const bool ok = s_otaRunning && !Update.hasError();
  s_otaRunning = false;
  AsyncWebServerResponse* res =
      r->beginResponse(ok ? 200 : 500, "text/plain", ok ? "OK" : "echec OTA");
  res->addHeader("Connection", "close");
  addSec(res);
  r->send(res);
  if (ok) net::requestReboot(1000);
}

void handleOtaUpload(AsyncWebServerRequest* r, const String& filename, size_t index,
                     uint8_t* data, size_t len, bool isLast) {
  if (index == 0) {
    s_otaRunning = false;
    s_otaReject  = false;
    if (!authValid(r))   { s_otaReject = true; return; }
    if (engineRunning()) { s_otaReject = true; return; }
    Serial.printf("[OTA] debut : %s\n", filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Update.printError(Serial);
      s_otaReject = true;
      return;
    }
    s_otaRunning = true;
  }
  if (!s_otaRunning) return;

  if (Update.write(data, len) != len) Update.printError(Serial);

  if (isLast) {
    if (Update.end(true)) Serial.println("[OTA] termine, redemarrage imminent");
    else                  Update.printError(Serial);
  }
}

}  // namespace

void web::begin() {
  authInit();

  s_server.on("/",            HTTP_GET,  handleRoot);
  s_server.on("/api/state",   HTTP_GET,  handleState);
  s_server.on("/api/login",   HTTP_POST, handleLogin);
  s_server.on("/api/config",  HTTP_POST, handleConfig);
  s_server.on("/api/license", HTTP_POST, handleLicense);
  s_server.on("/api/wifi",    HTTP_POST, handleWifi);
  s_server.on("/api/ota",     HTTP_POST, handleOtaDone, handleOtaUpload);

  s_server.addHandler(&s_events);
  s_server.onNotFound([](AsyncWebServerRequest* r) { r->send(404, "text/plain", "404"); });
  s_server.begin();

  Serial.println("[WEB] serveur HTTP demarre");
}

size_t web::sseClients() { return s_events.count(); }

void web::sseSend(const char* json, const char* event) {
  s_events.send(json, event, millis());
}
