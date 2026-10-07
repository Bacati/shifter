#pragma once
#include <Arduino.h>

// ============================================================================
//  WaterTemp : DS18B20 (1-Wire) -> temperature eau / liquide de refroidissement.
//
//  Purement informatif pour l'instant : dashboard + logs (telemetrie SSE).
//  N'influence aucune assistance (pas de limiteur base dessus a ce stade).
//
//  Capteur DIGITAL choisi plutot qu'une thermistance + diviseur sur l'ADC :
//  le bus 1-Wire transporte une trame avec CRC (comme le filtre EMI RPM, on
//  prefere un signal verifiable a une lecture analogique exposee au bruit du
//  harnais pres de l'allumage PVL).
//
//  Lecture sur une tache dediee (Core reseau) : la conversion DS18B20 prend
//  ~750 ms, ce qui ne doit JAMAIS bloquer loop() (Core 1, temps-reel).
// ============================================================================

namespace watertemp {

bool begin();      // detecte le capteur (OneWire). true si present.
void startTask();  // tache periodique Core 0 ; no-op si capteur absent au boot
                   // (retente la detection periodiquement)
bool present();

}  // namespace watertemp
