#pragma once
#include <Arduino.h>
#include "../core/Types.h"

// ============================================================================
//  Storage : acces NVS (Preferences) serialise par un mutex.
//  Source unique des cles NVS et de la fonction de clamp (sanitize).
// ============================================================================

namespace storage {

void begin();  // cree le mutex ; a appeler avant tout autre appel

// Clamp toutes les valeurs dans leurs bornes (cfg::*_MIN / *_MAX).
EcuConfig sanitize(const EcuConfig& in);

EcuConfig loadConfig();               // NVS -> sanitize (defauts si absent)
void      saveConfig(const EcuConfig& c);

LicenseTier loadTier();               // 0 (BASIC) si absent / invalide
String      loadLicenseKey();         // "" si absente
void        saveLicense(const String& key, LicenseTier tier);

String loadWifiPassword();            // "" si absent
void   saveWifiPassword(const String& pw);

}  // namespace storage
