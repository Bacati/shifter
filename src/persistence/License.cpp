#include "License.h"
#include "../core/Config.h"
#include "Storage.h"
#include "../sim/SimEngine.h"
#include <WiFi.h>
#include <atomic>
#include <string.h>
#include <mbedtls/md.h>

// ---- Resolution du secret ---------------------------------------------------
#if __has_include("secrets.h")
  #include "secrets.h"
#endif
#ifndef LICENSE_HMAC_SECRET
  #ifdef ALLOW_INSECURE_DEFAULT_SECRET
    #warning "LICENSE_HMAC_SECRET par defaut : NE PAS EXPEDIER (voir BUILD.md)"
    #define LICENSE_HMAC_SECRET "insecure-default-change-me"
  #else
    #error "src/persistence/secrets.h absent : copiez secrets.example.h -> secrets.h (voir BUILD.md)"
  #endif
#endif

namespace {
std::atomic<uint8_t> s_tier{static_cast<uint8_t>(LicenseTier::Basic)};
uint8_t              s_fails     = 0;
uint32_t             s_lockUntil = 0;

void computeKey(LicenseTier t, char* out /* >= LICENSE_KEY_HEX_LEN + 1 */) {
  String payload = WiFi.macAddress() + ":" + String(static_cast<int>(t));

  uint8_t h[32];
  const mbedtls_md_info_t* info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  mbedtls_md_hmac(info,
                  reinterpret_cast<const uint8_t*>(LICENSE_HMAC_SECRET), strlen(LICENSE_HMAC_SECRET),
                  reinterpret_cast<const uint8_t*>(payload.c_str()), payload.length(),
                  h);

  static const char kHex[] = "0123456789ABCDEF";
  for (int i = 0; i < LICENSE_KEY_HEX_LEN; ++i) {
    uint8_t b = h[i / 2];
    out[i] = (i & 1) ? kHex[b & 0x0F] : kHex[b >> 4];
  }
  out[LICENSE_KEY_HEX_LEN] = '\0';
}

// Comparaison a temps constant (les deux buffers font n octets).
bool ctEquals(const char* a, const char* b, size_t n) {
  uint8_t diff = 0;
  for (size_t i = 0; i < n; ++i) diff |= static_cast<uint8_t>(a[i]) ^ static_cast<uint8_t>(b[i]);
  return diff == 0;
}

bool matchesTier(const String& key, LicenseTier t) {
  if (key.length() != LICENSE_KEY_HEX_LEN) return false;
  char expected[LICENSE_KEY_HEX_LEN + 1];
  computeKey(t, expected);
  return ctEquals(key.c_str(), expected, LICENSE_KEY_HEX_LEN);
}
}  // namespace

String license::expectedKey(LicenseTier t) {
  char b[LICENSE_KEY_HEX_LEN + 1];
  computeKey(t, b);
  return String(b);
}

LicenseTier license::tier() {
  return static_cast<LicenseTier>(s_tier.load());
}

void license::begin() {
  const int forced = sim::forcedTier();  // >= 0 uniquement en mode simulation
  if (forced >= 0) {
    s_tier.store(static_cast<uint8_t>(forced));
    Serial.printf("[LIC] SIMULATION -> tier %d\n", forced);
    return;
  }

  const LicenseTier stored = storage::loadTier();
  String key = storage::loadLicenseKey();
  key.trim();
  key.toUpperCase();

  // On revalide la cle : un putInt("licenseTier", 2) seul ne debloque rien.
  if (matchesTier(key, stored)) {
    s_tier.store(static_cast<uint8_t>(stored));
    Serial.printf("[LIC] tier %d (%s)\n", static_cast<int>(stored), toString(stored));
  } else {
    s_tier.store(static_cast<uint8_t>(LicenseTier::Basic));
    Serial.println("[LIC] BASIC (aucune licence valide)");
  }

#ifdef LICENSE_PRINT_KEYS_ON_BOOT
  for (int t = 0; t < kLicenseTierCount; ++t)
    Serial.printf("[LIC] cle tier %d = %s\n", t, expectedKey(static_cast<LicenseTier>(t)).c_str());
#endif
}

bool license::tryUnlock(const String& keyIn, uint32_t& retryAfterMs) {
  retryAfterMs = 0;
  const uint32_t now = millis();

  if (s_lockUntil && static_cast<int32_t>(s_lockUntil - now) > 0) {
    retryAfterMs = s_lockUntil - now;
    return false;
  }

  String key = keyIn;
  key.trim();
  key.toUpperCase();

  for (int t = kLicenseTierCount - 1; t >= 0; --t) {
    if (matchesTier(key, static_cast<LicenseTier>(t))) {
      s_fails = 0;
      // Jamais de baisse de niveau par saisie de cle : re-saisir une ancienne cle TRACK sur une
      // carte PRO ne doit pas faire perdre le PRO. (Retour en arriere : WIPE usine.)
      if (t < s_tier.load()) {
        Serial.printf("[LIC] cle tier %d valide mais niveau actuel %d conserve\n", t, static_cast<int>(s_tier.load()));
        return true;
      }
      s_tier.store(static_cast<uint8_t>(t));
      storage::saveLicense(key, static_cast<LicenseTier>(t));
      Serial.printf("[LIC] deblocage -> tier %d\n", t);
      return true;
    }
  }

  if (++s_fails >= cfg::LICENSE_MAX_FAILS) {
    s_lockUntil  = now + cfg::LICENSE_LOCKOUT_MS;
    s_fails      = 0;
    retryAfterMs = cfg::LICENSE_LOCKOUT_MS;
  }
  return false;
}
