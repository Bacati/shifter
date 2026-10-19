#include "Storage.h"
#include "../core/Config.h"
#include <Preferences.h>
#include <stddef.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace {
Preferences       s_prefs;
SemaphoreHandle_t s_lock = nullptr;
const char* const NS = "ecu_config";  // conserve pour compat NVS des versions anterieures

// Carto = UN SEUL blob NVS (version + CRC32) : l'ecriture d'un item NVS est atomique, donc une
// coupure de courant laisse l'ancienne carto OU la nouvelle, jamais un melange. Les anciennes
// cles separees ("shiftCut", ...) ne sont plus qu'en lecture (migration depuis v23.0).
constexpr uint16_t CFG_MAGIC   = 0xEC01;
constexpr uint16_t CFG_VERSION = 2;
const char* const  CFG_KEY     = "cfg1";
struct CfgBlob {          // v2 : + pitKmh
  uint16_t magic;
  uint16_t version;
  int32_t  v[8];
  uint32_t crc;
};
struct CfgBlobV1 {        // v1 (v23.x) : lue pour migration
  uint16_t magic;
  uint16_t version;
  int32_t  v[7];
  uint32_t crc;
};

uint32_t crc32(const uint8_t* d, size_t n) {
  uint32_t c = 0xFFFFFFFFu;
  for (size_t i = 0; i < n; ++i) {
    c ^= d[i];
    for (int k = 0; k < 8; ++k) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
  }
  return ~c;
}
uint32_t blobCrc(const CfgBlob& b) { return crc32(reinterpret_cast<const uint8_t*>(&b), offsetof(CfgBlob, crc)); }
uint32_t blobCrcV1(const CfgBlobV1& b) { return crc32(reinterpret_cast<const uint8_t*>(&b), offsetof(CfgBlobV1, crc)); }

struct Guard {
  Guard()  { if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY); }
  ~Guard() { if (s_lock) xSemaphoreGive(s_lock); }
};
}  // namespace

void storage::begin() {
  if (!s_lock) s_lock = xSemaphoreCreateMutex();
}

EcuConfig storage::sanitize(const EcuConfig& in) {
  EcuConfig o;
  o.shiftCutMs     = constrain(in.shiftCutMs,     cfg::SHIFT_CUT_MIN, cfg::SHIFT_CUT_MAX);
  o.minShiftRpm    = constrain(in.minShiftRpm,    cfg::MIN_RPM_MIN,   cfg::MIN_RPM_MAX);
  o.launchRpm      = constrain(in.launchRpm,      cfg::LAUNCH_MIN,    cfg::LAUNCH_MAX);
  o.maxSlipPercent = constrain(in.maxSlipPercent, cfg::SLIP_MIN,      cfg::SLIP_MAX);
  o.wheelCircumMm  = constrain(in.wheelCircumMm,  cfg::WHEEL_MIN,     cfg::WHEEL_MAX);
  o.pulsesPerRev   = constrain(in.pulsesPerRev,   cfg::PULSES_MIN,    cfg::PULSES_MAX);
  o.pitRpm         = constrain(in.pitRpm,         cfg::PIT_MIN,       cfg::PIT_MAX);
  o.pitKmh         = constrain(in.pitKmh,         cfg::PITKMH_MIN,    cfg::PITKMH_MAX);
  return o;
}

EcuConfig storage::loadConfig() {
  Guard g;
  EcuConfig c;
  s_prefs.begin(NS, true);

  CfgBlob b;
  CfgBlobV1 b1;
  c.pitKmh = cfg::DEFAULTS.pitKmh;
  const size_t len = s_prefs.getBytesLength(CFG_KEY);
  if (len == sizeof(b) &&
      s_prefs.getBytes(CFG_KEY, &b, sizeof(b)) == sizeof(b) &&
      b.magic == CFG_MAGIC && b.version == CFG_VERSION && b.crc == blobCrc(b)) {
    c.shiftCutMs = b.v[0]; c.minShiftRpm = b.v[1]; c.launchRpm = b.v[2]; c.maxSlipPercent = b.v[3];
    c.wheelCircumMm = b.v[4]; c.pulsesPerRev = b.v[5]; c.pitRpm = b.v[6]; c.pitKmh = b.v[7];
  } else if (len == sizeof(b1) &&
             s_prefs.getBytes(CFG_KEY, &b1, sizeof(b1)) == sizeof(b1) &&
             b1.magic == CFG_MAGIC && b1.version == 1 && b1.crc == blobCrcV1(b1)) {
    c.shiftCutMs = b1.v[0]; c.minShiftRpm = b1.v[1]; c.launchRpm = b1.v[2]; c.maxSlipPercent = b1.v[3];
    c.wheelCircumMm = b1.v[4]; c.pulsesPerRev = b1.v[5]; c.pitRpm = b1.v[6];   // pitKmh = defaut
  } else {   // pas de blob valide : anciennes cles (v23.0) ou valeurs par defaut
    c.shiftCutMs     = s_prefs.getInt("shiftCut",  cfg::DEFAULTS.shiftCutMs);
    c.minShiftRpm    = s_prefs.getInt("minRpm",    cfg::DEFAULTS.minShiftRpm);
    c.launchRpm      = s_prefs.getInt("launchRpm", cfg::DEFAULTS.launchRpm);
    c.maxSlipPercent = s_prefs.getInt("slipMax",   cfg::DEFAULTS.maxSlipPercent);
    c.wheelCircumMm  = s_prefs.getInt("wheelCirc", cfg::DEFAULTS.wheelCircumMm);
    c.pulsesPerRev   = s_prefs.getInt("pulses",    cfg::DEFAULTS.pulsesPerRev);
    c.pitRpm         = s_prefs.getInt("pitRpm",    cfg::DEFAULTS.pitRpm);
  }
  s_prefs.end();
  return sanitize(c);
}

void storage::saveConfig(const EcuConfig& in) {
  const EcuConfig c = sanitize(in);
  CfgBlob b;
  memset(&b, 0, sizeof(b));
  b.magic = CFG_MAGIC; b.version = CFG_VERSION;
  b.v[0] = c.shiftCutMs; b.v[1] = c.minShiftRpm; b.v[2] = c.launchRpm; b.v[3] = c.maxSlipPercent;
  b.v[4] = c.wheelCircumMm; b.v[5] = c.pulsesPerRev; b.v[6] = c.pitRpm; b.v[7] = c.pitKmh;
  b.crc = blobCrc(b);

  Guard g;
  s_prefs.begin(NS, false);
  s_prefs.putBytes(CFG_KEY, &b, sizeof(b));   // un seul item = ecriture atomique
  s_prefs.end();
}

// Licence = UN SEUL item NVS "lic" = "<tier>:<cle>" : ecriture atomique, donc une coupure de courant
// pendant l'activation laisse l'ancienne licence OU la nouvelle (avant : 2 items "licenseKey" +
// "licenseTier", une coupure entre les deux faisait retomber un client TRACK/PRO en BASIC).
// Les anciens items ne sont lus que si "lic" n'existe pas (cartes deja deployees).
namespace {
// lit la licence (mutex deja pris) ; tier = -1 si invalide
void readLicense(String& key, int& tier) {
  s_prefs.begin(NS, true);
  const String lic = s_prefs.getString("lic", "");
  if (lic.length() > 0) {
    const int sep = lic.indexOf(':');
    tier = (sep == 1 && lic[0] >= '0' && lic[0] < '0' + kLicenseTierCount) ? (lic[0] - '0') : -1;
    key  = (sep >= 0) ? lic.substring(sep + 1) : String("");
  } else {
    tier = s_prefs.getInt("licenseTier", 0);
    key  = s_prefs.getString("licenseKey", "");
  }
  s_prefs.end();
}
}  // namespace

LicenseTier storage::loadTier() {
  Guard g;
  String k; int t = 0;
  readLicense(k, t);
  if (t < 0 || t >= kLicenseTierCount) t = 0;
  return static_cast<LicenseTier>(t);
}

String storage::loadLicenseKey() {
  Guard g;
  String k; int t = 0;
  readLicense(k, t);
  return k;
}

void storage::saveLicense(const String& key, LicenseTier tier) {
  Guard g;
  s_prefs.begin(NS, false);
  s_prefs.putString("lic", String(static_cast<int>(tier)) + ":" + key);
  s_prefs.end();
}

String storage::loadWifiPassword() {
  Guard g;
  s_prefs.begin(NS, true);
  String p = s_prefs.getString("wifiPass", "");
  s_prefs.end();
  return p;
}

void storage::saveWifiPassword(const String& pw) {
  Guard g;
  s_prefs.begin(NS, false);
  s_prefs.putString("wifiPass", pw);
  s_prefs.end();
}
