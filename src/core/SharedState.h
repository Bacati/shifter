#pragma once
#include <Arduino.h>
#include "Types.h"

// ============================================================================
//  SharedState : UNIQUE frontiere de donnees entre le Core 1 (moteur) et le
//  Core 0 (WiFi / telemetrie). Toute la synchronisation vit ici.
//
//   - scalaires  : std::atomic (lecture/ecriture 32 bits atomiques)
//   - EcuConfig  : copie protegee par spinlock (ecrite rarement par le web)
// ============================================================================

namespace state {

void begin();  // a appeler en tout debut de setup()

// ---- Cartographie (Core 0 ecrit via web, Core 1 lit chaque tour) ----
EcuConfig getConfig();
void      setConfig(const EcuConfig& c);

// ---- Telemetrie moteur (Core 1 ecrit, Core 0 lit) ----
void setRpmFiltered(int rpm);     // EWMA : telemetrie + logique non critique
int  getRpmFiltered();
void setRpmInstant(int rpm);      // instantane : limiteur d'overrev (aucun retard)
int  getRpmInstant();
void setSpeeds(int frontKmhX10, int rearKmhX10);
void getSpeeds(int& frontKmhX10, int& rearKmhX10);
void setSlip(int slipPctX10, int slipRatePctX10);
void getSlip(int& slipPctX10, int& slipRatePctX10);

void       setMode(AssistMode m); // ecrit uniquement par IgnitionCut
AssistMode getMode();

// ---- IMU / angle de roulis (tache IMU ou sim ecrit ; Core 1 lit) ----
//  rollDegX10  : angle de roulis * 10 (signe)
//  lateralGx100: |acceleration| / g * 100
//  ok          : l'IMU repond et les donnees sont plausibles
void     setLean(int rollDegX10, int lateralGx100, bool ok);
void     getLean(int& rollDegX10, int& lateralGx100, bool& ok);
uint32_t lastLeanMs();

// ---- Heartbeat de loop() (surveille par le superviseur, Core 0) ----
void     beatEngineLoop();
uint32_t lastEngineBeatMs();

}  // namespace state
