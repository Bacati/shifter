#pragma once
#include <Arduino.h>

// ============================================================================
//  SimEngine : banc d'essai pour le simulateur Wokwi (aucun materiel).
//
//  Actif UNIQUEMENT si compile avec -DWOKWI_SIM (environnement [env:wokwi]).
//
//  Quand actif :
//   - WiFi / serveur web / SSE desactives (Wokwi tourne alors a ~100 %) ;
//   - licence forcee a PRO ;
//   - faux moteur : cycle regime / vitesses / glisse en boucle
//     -> LAUNCH / SHIFT / SLIP / OVERREV visibles sur la LED CUT (D4) ;
//   - 6 clignotements de la LED au boot (diagnostic) ;
//   - console serie : voir sim::update() / commande 'help'.
//
//  Quand inactif : toutes les fonctions sont neutres, firmware normal.
// ============================================================================

// Echelle temporelle des coupures d'allumage (rend les cuts visibles a l'oeil).
//  - build normal     : 1  (aucun effet)
//  - build -DWOKWI_SIM : 40 par defaut  -> 50 ms de cut = 2 s de LED
//  Valeur initiale ; ajustable a chaud en simu via la commande serie 'scale <n>'.
#ifndef IGNITION_SIM_SCALE
  #ifdef WOKWI_SIM
    #define IGNITION_SIM_SCALE 40
  #else
    #define IGNITION_SIM_SCALE 1
  #endif
#endif

// Reglages simu pilotes depuis platformio.ini (pas besoin du moniteur serie) :
//  SIM_SHIFT_CUT_MS : > 0 force la duree de coupure du quickshifter (ms).
//                     0 = laisse la valeur par defaut / NVS.
//  SIM_FREEZE       : 1 = demarre avec le faux moteur fige
//                     (la LED ne s'allume plus qu'avec les boutons).
#ifndef SIM_SHIFT_CUT_MS
  #define SIM_SHIFT_CUT_MS 0
#endif
#ifndef SIM_FREEZE
  #define SIM_FREEZE 0
#endif
// Tier de licence force au boot en simu : 0 = BASIC, 1 = TRACK, 2 = PRO.
//  Sert a tester le bridage : en BASIC, Launch/Slip ne doivent PAS s'activer ;
//  en TRACK, Launch oui / Slip non ; en PRO, tout. (Modifiable a chaud : 'key <cle>'.)
#ifndef SIM_TIER
  #define SIM_TIER 2
#endif

namespace sim {

void        begin();
void        update();               // appele apres sensors::update() dans loop()
bool        active();

String      apPasswordOverride();   // "" si inactif
const char* pinOverride();          // nullptr si inactif
int         forcedTier();           // -1 si inactif, sinon 0/1/2
int         timeScale();            // 1 si inactif ; sinon echelle courante (>=1)

}  // namespace sim
