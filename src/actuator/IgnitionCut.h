#pragma once
#include <Arduino.h>
#include "../core/Types.h"

// ============================================================================
//  IgnitionCut : unique proprietaire de la broche de coupure (pins::CUT).
//
//   CUT = allumage mis a la masse (le PVL genere sa propre etincelle).
//   Ecriture registre directe (GPIO.out_w1ts / w1tc), sous section critique.
//
//  La PRIORITE STRICTE de l'arbitre est appliquee ici :
//   - un mode de priorite >= au mode courant (ou aucune coupure active)
//     peut (re)armer la coupure et la PROLONGER ;
//   - il ne la RACCOURCIT jamais (une coupure programmee va a son terme) ;
//   - un mode de priorite inferieure est ignore tant qu'une coupure
//     prioritaire est active.
// ============================================================================

namespace ignition {

void       begin();
void       request(uint16_t durationMs, AssistMode mode);
void       service();      // a appeler tres souvent depuis loop()
bool       isActive();
AssistMode activeMode();
void       forceClear();   // fail-safe superviseur : rend l'etincelle immediatement

}  // namespace ignition
