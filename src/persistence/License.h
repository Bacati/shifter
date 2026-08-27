#pragma once
#include <Arduino.h>
#include "../core/Types.h"

// ============================================================================
//  License : DRM lie au HWID (MAC).
//
//   cle = HEX( HMAC-SHA256( secret , "<MAC>:<tier>" ) )  tronque a
//         LICENSE_KEY_HEX_LEN caracteres.
//
//   - comparaison a temps constant
//   - anti-brute-force (lockout apres cfg::LICENSE_MAX_FAILS)
//   - le secret vient de secrets.h (gitignore) ; cf. secrets.example.h
//
//   LIMITE : sans Flash Encryption + Secure Boot v2, le secret reste
//   extractible par dump flash. Voir BUILD.md.
// ============================================================================

#ifndef LICENSE_KEY_HEX_LEN
#define LICENSE_KEY_HEX_LEN 16  // 64 bits
#endif

namespace license {

void        begin();  // charge + revalide le tier stocke en NVS (WiFi doit etre init)
LicenseTier tier();   // atomique

// Tente un deblocage. true si accepte. 'retryAfterMs' > 0 si lockout actif.
bool tryUnlock(const String& key, uint32_t& retryAfterMs);

// Diagnostic usine : cle attendue pour un tier sur CE materiel.
String expectedKey(LicenseTier t);

}  // namespace license
