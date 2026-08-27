#pragma once
// ============================================================================
//  Version du firmware + matrice des fonctions par licence.
// ============================================================================

#define FW_VERSION "23.0"
#define FW_BUILD   (__DATE__ " " __TIME__)

// Matrice fonctions / licence (voir control/Arbiter.cpp) :
//
//   BASIC : Overrev limiter + Pit limiter + Quickshifter
//   TRACK : + Launch control            (2-step limiter sur la grille)
//   PRO   : + Predictive Slip control   (motricite en ligne droite)
//
// Cible de compilation : Arduino-ESP32 2.0.x (ESP-IDF 4.4).
// Voir BUILD.md pour la liste des bibliotheques et le cablage.
