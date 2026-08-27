#include "Storage.h"
#include "../core/Config.h"
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace {
Preferences       s_prefs;
SemaphoreHandle_t s_lock = nullptr;
const char* const NS = "ecu_config";  // conserve pour compat NVS des versions anterieures

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
  return o;
}

EcuConfig storage::loadConfig() {
  Guard g;
  EcuConfig c;
  s_prefs.begin(NS, true);
  c.shiftCutMs     = s_prefs.getInt("shiftCut",  cfg::DEFAULTS.shiftCutMs);
  c.minShiftRpm    = s_prefs.getInt("minRpm",    cfg::DEFAULTS.minShiftRpm);
  c.launchRpm      = s_prefs.getInt("launchRpm", cfg::DEFAULTS.launchRpm);
  c.maxSlipPercent = s_prefs.getInt("slipMax",   cfg::DEFAULTS.maxSlipPercent);
  c.wheelCircumMm  = s_prefs.getInt("wheelCirc", cfg::DEFAULTS.wheelCircumMm);
  c.pulsesPerRev   = s_prefs.getInt("pulses",    cfg::DEFAULTS.pulsesPerRev);
  c.pitRpm         = s_prefs.getInt("pitRpm",    cfg::DEFAULTS.pitRpm);
  s_prefs.end();
  return sanitize(c);
}

void storage::saveConfig(const EcuConfig& in) {
  const EcuConfig c = sanitize(in);
  Guard g;
  s_prefs.begin(NS, false);
  // Preferences::putInt ne reecrit pas si la valeur est inchangee (limite l'usure flash).
  s_prefs.putInt("shiftCut",  c.shiftCutMs);
  s_prefs.putInt("minRpm",    c.minShiftRpm);
  s_prefs.putInt("launchRpm", c.launchRpm);
  s_prefs.putInt("slipMax",   c.maxSlipPercent);
  s_prefs.putInt("wheelCirc", c.wheelCircumMm);
  s_prefs.putInt("pulses",    c.pulsesPerRev);
  s_prefs.putInt("pitRpm",    c.pitRpm);
  s_prefs.end();
}

LicenseTier storage::loadTier() {
  Guard g;
  s_prefs.begin(NS, true);
  int t = s_prefs.getInt("licenseTier", 0);
  s_prefs.end();
  if (t < 0 || t >= kLicenseTierCount) t = 0;
  return static_cast<LicenseTier>(t);
}

String storage::loadLicenseKey() {
  Guard g;
  s_prefs.begin(NS, true);
  String k = s_prefs.getString("licenseKey", "");
  s_prefs.end();
  return k;
}

void storage::saveLicense(const String& key, LicenseTier tier) {
  Guard g;
  s_prefs.begin(NS, false);
  s_prefs.putString("licenseKey", key);
  s_prefs.putInt("licenseTier", static_cast<int>(tier));
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
