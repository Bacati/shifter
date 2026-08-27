#pragma once
#include <Arduino.h>

// ============================================================================
//  Imu : MPU6050 (I2C) -> estimation de l'angle de roulis.
//
//  Sert de GARDE-FOU : le slip control ne doit pas couper l'allumage quand la
//  moto est sur l'angle (une micro-coupure sur l'angle = retour d'adherence
//  brutal = high-side). Voir Arbiter::evalSlip().
//
//  - pilote registre (pas de librairie), Wire.
//  - filtre complementaire gyro + accelero.
//  - specificite moto : en virage stabilise l'accelero pointe le long de l'axe
//    de la moto (il "voit a plat"). On ne recale donc le gyro sur l'accelero
//    que lorsque |accel| est proche de 1 g (moto quasi non sollicitee).
//  - pousse le resultat dans state::setLean(). Si l'IMU ne repond pas / donnees
//    aberrantes -> state::setLean(..., ok=false) -> slip control desactive.
// ============================================================================

namespace imu {

bool begin();      // true si le MPU6050 repond (WHO_AM_I)
void startTask();  // tache 200 Hz sur le Core reseau
bool present();    // etat courant (apres begin)

}  // namespace imu
