#pragma once

// ============================================================================
//  Arbiter : arbitre central a priorite stricte.
//  Evalue les assistances dans l'ordre et emet au plus une demande de coupure
//  par tour vers IgnitionCut (qui applique la priorite fine).
//
//   1. OVERREV  (survie, RPM instantane)      - toujours
//   2. PIT      (reglement)                    - toujours
//   3. SHIFT    (quickshifter)                 - toujours
//   4. LAUNCH   (depart arrete)                - licence TRACK+
//   5. SLIP     (motricite ligne droite)       - licence PRO
// ============================================================================

namespace arbiter {
void update();  // appele chaque tour de loop() (Core 1)
}
