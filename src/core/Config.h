#pragma once
#include <Arduino.h>
#include "Types.h"

// ============================================================================
//  Configuration statique : broches + tous les reglages compiles.
//  Rien de "magique" ne doit trainer ailleurs dans le code.
// ============================================================================

namespace pins {
  // /!\ PIN_CUT : PULL-DOWN 10k physique OBLIGATOIRE (allumage rendu au boot / reset).
  constexpr int CUT         = 4;
  constexpr int RPM         = 5;   // trigger allumage : 1 impulsion / tour vilebrequin
  constexpr int WHEEL_FRONT = 14;
  constexpr int WHEEL_REAR  = 27;
  constexpr int SHIFTER     = 26;  // /!\ filtre RC 100nF materiel OBLIGATOIRE
  constexpr int CLUTCH_HALL = 25;
  constexpr int PIT_BTN     = 33;
  constexpr int IMU_SDA     = 21;  // MPU6050 (I2C) - angle de roulis, garde-fou anti-angle
  constexpr int IMU_SCL     = 22;
  constexpr int WATER_TEMP  = 32;  // DS18B20 (1-Wire) - temperature eau/liquide refroidissement
                                    // /!\ resistance de tirage 4,7k vers 3V3 OBLIGATOIRE sur le bus
  // Bus CAN (controleur TWAI integre + transceiver externe 3,3 V). RESERVEES, pas encore
  // utilisees : proposition a valider avec le bureau d'etude (voir AUDIT.md §CAN).
  // Evite : strapping (0,2,5,12,15), PSRAM WROVER (16,17), entrees seules (34..39).
  constexpr int CAN_TX      = 18;
  constexpr int CAN_RX      = 19;
}

// Les macros GPIO.out_w1ts / w1tc n'adressent que les GPIO 0..31.
static_assert(pins::CUT >= 0 && pins::CUT < 32,
              "pins::CUT doit etre < 32 (registre GPIO.out_w1ts/w1tc)");

// Carte mere commune : deux fonctions ne doivent JAMAIS partager une broche.
namespace pins_check {
  constexpr int kAll[] = { pins::CUT, pins::RPM, pins::WHEEL_FRONT, pins::WHEEL_REAR,
                           pins::SHIFTER, pins::CLUTCH_HALL, pins::PIT_BTN, pins::IMU_SDA,
                           pins::IMU_SCL, pins::WATER_TEMP, pins::CAN_TX, pins::CAN_RX };
  constexpr int kCount = sizeof(kAll) / sizeof(kAll[0]);
  constexpr bool allDistinct(int i, int j) {   // C++11 : recursion, pas de boucle
    return i >= kCount ? true
         : j >= kCount ? allDistinct(i + 1, i + 2)
         : (kAll[i] != kAll[j]) && allDistinct(i, j + 1);
  }
}
static_assert(pins_check::allDistinct(0, 1), "deux fonctions utilisent la meme broche (pins::*)");
static_assert(pins::CAN_TX < 34, "CAN_TX doit etre une broche de sortie (GPIO34..39 = entrees seules)");

namespace cfg {
  // ---- RPM / filtre EMI --------------------------------------------------
  constexpr int      OVERREV_RPM        = 16000;   // limiteur dur "survie moteur"
  constexpr int      RPM_MAX_PLAUSIBLE  = 20000;   // au-dela = parasite EMI (spec)
  constexpr uint32_t RPM_MIN_DELTA_US   = (60UL * 1000000UL) / RPM_MAX_PLAUSIBLE; // ~3000 us
  constexpr float    RPM_SLEW_RPM_PER_S = 120000.0f; // pente max admise (tr/min/s) : clamp, pas rejet
  constexpr uint32_t RPM_STALL_US       = 500000;  // sans impulsion VALIDEE => moteur cale
  constexpr int      RPM_FILTER_SHIFT   = 2;       // EWMA : alpha = 1 / 2^shift = 0.25
  // Verrouillage : depuis l'arret, le RPM n'est publie qu'apres RPM_LOCK_CONSISTENT periodes
  // successives COHERENTES (+/- RPM_LOCK_TOL_PCT) ET compatibles avec un moteur reel (<= OVERREV).
  // Un parasite aleatoire ne fabrique donc plus de "moteur fantome".
  constexpr int      RPM_LOCK_CONSISTENT = 5;
  constexpr int      RPM_LOCK_TOL_PCT    = 30;
  // Deverrouillage : RPM_UNLOCK_BAD periodes consecutives hors de [1/RPM_BAD_LO .. RPM_BAD_HI] x
  // la periode attendue = signal qui n'est plus un moteur (arret + bruit). Une impulsion
  // manquee isolee (x2) ne deverrouille pas.
  constexpr int      RPM_UNLOCK_BAD      = 6;
  constexpr float    RPM_BAD_LO          = 0.5f;
  constexpr float    RPM_BAD_HI          = 1.6f;

  // ---- Overrev limiter -------------------------------------------------
  constexpr uint16_t OVERREV_CUT_MS     = 50;
  constexpr uint32_t OVERREV_PERIOD_MS  = 40;

  // ---- Pit limiter ---------------------------------------------------
  constexpr uint16_t PIT_CUT_MS         = 20;
  constexpr uint32_t PIT_PERIOD_MS      = 50;
  constexpr int      PIT_HARD_BAND_RPM  = 400;     // repli regime : au-dela de pitRpm + bande, coupure continue
  constexpr float    PIT_HARD_BAND_KMH  = 3.0f;    // au-dela de pitKmh + bande : coupure continue (vrai limiteur)
  constexpr float    PIT_SPEED_VALID_KMH = 1.0f;   // sous cette vitesse (les 2 roues) la mesure est jugee absente
                                                   // -> repli sur le regime

  // ---- Garde-fou ultime sur toute coupure (defense en profondeur) -------
  constexpr uint16_t CUT_ABS_MAX_MS     = 200;     // aucune coupure unique ne depasse cette duree

  // ---- Quickshifter ------------------------------------------------
  constexpr uint32_t SHIFT_WINDOW_US       = 25000; // fenetre d'analyse de l'appui
  constexpr uint32_t SHIFT_ISR_DEBOUNCE_US = 30000; // anti-rebond materiel (ISR)
  constexpr uint32_t SHIFT_LOCKOUT_MS      = 250;   // verrou anti double-coupure
  constexpr uint32_t SHIFT_CONFIRM_US      = 2000;  // la broche doit etre ENCORE basse apres ce delai : rejette
                                                    // les parasites EMI de quelques us/ms (latence +2 ms)

  // ---- Launch control -------------------------------------------
  constexpr float    LAUNCH_SPEED_MAX_KMH = 5.0f;  // au-dela : on passe en roulage
  constexpr int      LAUNCH_ARM_BAND_RPM  = 200;   // s'active des launchRpm - 200
  constexpr uint16_t LAUNCH_CUT_MIN_MS    = 5;
  constexpr uint16_t LAUNCH_CUT_MAX_MS    = 20;
  constexpr uint32_t LAUNCH_PERIOD_MS     = 50;
  constexpr int      LAUNCH_HARD_BAND_RPM = 300;   // au-dela de launchRpm + bande : coupure continue (2-step)

  // ---- Slip control (motricite ligne droite) -----------------
  constexpr float    SLIP_MIN_SPEED_KMH     = 5.0f;
  constexpr float    SLIP_LOWSPEED_RAMP_KMH = 5.0f;  // rampe d'adoucissement 5 -> 10 km/h
  constexpr float    SLIP_HYSTERESIS_PCT    = 3.0f;
  constexpr float    SLIP_PREDICT_SEC       = 0.03f; // avance sur derivee de glisse (30 ms)
  constexpr float    SLIP_PREDICT_CLAMP_PCT = 4.0f;  // borne du terme d'avance
  constexpr float    SLIP_RATE_FILTER_A     = 0.35f; // EWMA derivee (vitesse roue propre => filtre plus leger)
  constexpr uint16_t SLIP_CUT_MIN_MS        = 5;
  constexpr uint16_t SLIP_CUT_MAX_MS        = 25;
  constexpr uint32_t SLIP_PERIOD_MS         = 50;

  // ---- Garde-fou anti-angle (IMU) : le slip control ne coupe pas sur l'angle -
  //  Autorite pleine sous LEAN_TAPER_DEG, degressive jusqu'a LEAN_CUTOFF_DEG, nulle
  //  au-dela. Nulle aussi si |accel| > LEAN_LATERAL_G_HARD (virage/freinage fort) ou
  //  si l'IMU n'est pas fiable (voir IMU_REQUIRED).
  constexpr float LEAN_TAPER_DEG      = 12.0f;
  constexpr float LEAN_CUTOFF_DEG     = 28.0f;
  constexpr float LEAN_REARM_DEG      = 8.0f;   // hysteresis : re-autorisation sous cet angle
  constexpr float LEAN_LATERAL_G_HARD = 1.30f;  // |accel|/g au-dela -> blocage dur

  // ---- IMU (MPU6050) ----------------------------------------
  constexpr uint8_t  IMU_ADDR           = 0x68;
  constexpr uint32_t IMU_I2C_HZ         = 400000;
  constexpr uint32_t IMU_TASK_PERIOD_MS = 5;      // 200 Hz
  constexpr uint32_t IMU_STALE_MS       = 120;    // au-dela : IMU non fiable
  constexpr uint8_t  IMU_MAX_FAILS      = 12;     // lectures I2C ratees consecutives -> KO
  constexpr uint32_t IMU_BIAS_SETTLE_MS = 800;    // moyennage gyro au boot (bike suppose immobile)
  constexpr float    IMU_COMP_ALPHA     = 0.985f; // poids gyro dans le filtre complementaire
  constexpr float    IMU_ACC_TRUST_LO_G = 0.85f;  // |accel| fiable pour recaler le gyro
  constexpr float    IMU_ACC_TRUST_HI_G = 1.12f;
  // Mapping axes selon le montage physique (roll = atan2(ACC[num], ACC[den]), rate = GYRO[g]).
  //  0 = X (avant), 1 = Y (gauche), 2 = Z (haut).
  constexpr int      IMU_ACC_ROLL_NUM   = 1;
  constexpr int      IMU_ACC_ROLL_DEN   = 2;
  constexpr int      IMU_GYRO_ROLL_AXIS = 0;
  constexpr float    IMU_ROLL_SIGN      = 1.0f;
  // Recalage accelero INTERDIT des que la moto tourne (lacet > seuil) : en virage stabilise
  // l'accelero "voit a plat" et ramenerait l'angle estime vers 0 alors que la moto est couchee
  // (mesure : |acc| reste dans la plage de confiance jusqu'a ~26 deg de roulis).
  constexpr int      IMU_GYRO_YAW_AXIS     = 2;      // axe vertical de la moto (Z)
  constexpr float    IMU_RECAL_MAX_YAW_DPS = 3.0f;
  // L'angle n'est declare fiable (ok=true) qu'apres un recalage moto quasi droite : apres un
  // (re)demarrage de l'IMU ou de l'ECU l'angle initial (0) est une hypothese, pas une mesure.
  constexpr float    IMU_VALID_MAX_DEG     = 5.0f;
  // ... et CONVERGEE : angle accelero ET angle estime tous deux sous IMU_VALID_MAX_DEG, de facon
  // soutenue (score +1 par echantillon bon, -1 si l'information manque, -2 si l'estimation n'a pas converge ; valide a IMU_VALID_SCORE).
  // Sans cela, apres une re-init en virage l'estimation vaut -angle (le gyro integre la remise a
  // plat depuis 0) et serait declaree fiable des le 1er echantillon "moto droite".
  constexpr int      IMU_VALID_SCORE       = 100;
  // Detection d'un MPU6050 endormi / fige apres brownout : |accel| ~ 0 ou trames identiques.
  constexpr float    IMU_ACC_SANE_LO_G     = 0.5f;
  constexpr uint8_t  IMU_FROZEN_SAMPLES    = 10;     // 10 x 5 ms = 50 ms de trames brutes identiques
  constexpr uint32_t IMU_REINIT_MS         = 500;    // periode des tentatives de (re)init
  // Biais du gyro (zero-rate jusqu'a +/-20 deg/s sur un MPU6050 !) : mesure valable uniquement si le
  // signal est CALME (crete-a-crete <= IMU_BIAS_QUIET_DPS). Mesure en roulant (re-init de l'IMU en
  // course) = moyenne du mouvement reel, pas un biais -> rejetee. Tant qu'aucun biais valide n'a
  // ete mesure, l'angle est declare non fiable (slip coupe).
  //  La mesure n'est faite QUE roues arretees (vitesses avant ET arriere < IMU_BIAS_STOP_KMH) : un
  //  signal constant en roulant (entree en virage lente a 3 deg/s) est indiscernable d'un biais.
  constexpr float    IMU_BIAS_QUIET_DPS    = 2.5f;   // ecart-type maximal du gyro de roulis sur la fenetre
  constexpr float    IMU_BIAS_STOP_KMH     = 1.0f;
  constexpr int      IMU_BIAS_WINDOW_SAMPLES = 200;  // 1 s a 200 Hz : fenetre de re-mesure en continu
  constexpr float    IMU_BIAS_MAX_STEP_DPS = 3.0f;   // une fois valide, saut maximal accepte par re-mesure
  // 1 = IMU obligatoire : pas d'IMU fiable -> slip control DESACTIVE (securite).
  // 0 = tolere l'absence d'IMU (banc / cartes sans capteur) -> slip sans garde-fou.
  #ifndef IMU_REQUIRED
    #define IMU_REQUIRED 1
  #endif

  // ---- Temperature eau (DS18B20, 1-Wire) ---------------------
  //  Purement informatif pour l'instant (dashboard / logs) : n'influence
  //  aucune assistance. Tache dediee Core 0, lecture non bloquante pour loop().
  constexpr uint32_t WATER_TEMP_PERIOD_MS = 1000;  // cycle conversion+lecture (12 bits ~750 ms)
  constexpr float    WATER_TEMP_MIN_C     = -20.0f; // plausibilite capteur
  constexpr float    WATER_TEMP_MAX_C     = 150.0f;
  constexpr float    WATER_TEMP_WARN_C    = 95.0f;  // seuil d'alerte (affichage ; pas de coupure)
  constexpr uint32_t WATER_TEMP_STALE_MS  = 3000;   // au-dela : capteur considere en panne
  constexpr uint32_t WATER_TEMP_PROBE_MS  = 2000;   // re-essai de detection si capteur absent
  constexpr uint8_t  WATER_TEMP_MAX_FAILS = 3;      // lectures ratees consecutives tolerees avant "absent"
  constexpr float    WATER_TEMP_POR_C     = 85.0f;  // valeur d'usine du DS18B20 (conversion non faite)

  // ---- Capteurs de roue / entrees -------------------------
  constexpr uint32_t WHEEL_MIN_PULSE_US   = 0;    // 0 = pas d'anti-rebond (VR/Hall). >0 pour reed.
  constexpr uint32_t SPEED_CALC_MS        = 40;   // periode de calcul vitesse / glisse
  // Roue avant "vivante" pour calculer une glisse : derniere impulsion il y a moins de
  // max(SLIP_FRONT_FRESH_MIN_US, SLIP_FRONT_FRESH_PERIODS x periode mesuree). Un capteur avant
  // debranche fait sinon decroitre la vitesse avant estimee et fabrique une fausse glisse.
  // Une glisse n'est retenue que si elle est vue sur SLIP_CONFIRM_WINDOWS fenetres consecutives de
  // SPEED_CALC_MS : une impulsion parasite au milieu d'une periode fausse DEUX intervalles de suite.
  constexpr int      SLIP_CONFIRM_WINDOWS      = 4;
  constexpr uint32_t SLIP_FRONT_FRESH_MIN_US   = 60000;
  constexpr float    SLIP_FRONT_FRESH_PERIODS  = 1.5f;
  constexpr float    WHEEL_SPEED_MAX_KMH  = 160.0f; // vitesse impossible pour un 50cc -> rejet glitch
  constexpr float    WHEEL_SPEED_SLEW_KMH = 25.0f;  // variation max plausible entre 2 fenetres
  constexpr uint32_t WHEEL_STOP_US        = 400000; // sans impulsion depuis -> roue arretee (plancher)
  constexpr float    WHEEL_STOP_KMH       = 2.0f;   // la roue est "arretee" sous cette vitesse : le delai
                                                    // d'arret s'allonge si peu d'impulsions/tour (voir Sensors.cpp)
  constexpr uint32_t WHEEL_STOP_MAX_US    = 3000000;
  constexpr uint32_t INPUT_DEBOUNCE_MS    = 15;   // anti-rebond bouton pit / contacteur embrayage

  // ---- Bornes de validation NVS (partagees web + chargement) --------
  constexpr int SHIFT_CUT_MIN = 10,   SHIFT_CUT_MAX = 150;
  constexpr int MIN_RPM_MIN   = 2000, MIN_RPM_MAX   = 15000;
  constexpr int LAUNCH_MIN    = 2000, LAUNCH_MAX    = 15000;
  constexpr int SLIP_MIN      = 5,    SLIP_MAX      = 50;   // > SLIP_HYSTERESIS_PCT (sinon jamais de desarmement)
  constexpr int WHEEL_MIN     = 1000, WHEEL_MAX     = 2500;
  constexpr int PULSES_MIN    = 1,    PULSES_MAX    = 20;
  constexpr int PIT_MIN       = 2000, PIT_MAX       = 10000;
  constexpr int PITKMH_MIN    = 10,   PITKMH_MAX    = 100;

  // ---- Cartographie par defaut ---------------------------------------
  // Ordre = declaration de EcuConfig :
  // shiftCutMs, minShiftRpm, launchRpm, maxSlipPercent, wheelCircumMm, pulsesPerRev, pitRpm, pitKmh
  constexpr EcuConfig DEFAULTS = { 50, 6000, 8000, 15, 1800, 3, 4000, 60 };

  // ---- RTOS -----------------------------------------------------------
  constexpr int      WDT_TIMEOUT_S        = 3;
  constexpr int      CORE_NET             = 0;   // WiFi + telemetrie
  constexpr int      CORE_ENGINE          = 1;   // sanctuaire calcul moteur (loop())
  constexpr uint32_t SSE_TASK_STACK       = 6144;
  constexpr uint32_t SSE_PERIOD_MS        = 250;
  constexpr uint32_t SUPERVISOR_STACK     = 3072;
  constexpr uint32_t SUPERVISOR_PERIOD_MS = 200;
  constexpr uint32_t LOOP_STALL_MS        = 400;  // loop() (Core 1 dedie) fige au-dela => fail-safe + reboot

  // ---- Reseau / auth ------------------------------------------------
  constexpr uint8_t  AP_MAX_CLIENTS      = 2;
  constexpr uint8_t  AP_CHANNEL          = 6;
  constexpr uint32_t AUTH_TOKEN_TTL_MS   = 15UL * 60UL * 1000UL;
  constexpr uint8_t  AUTH_MAX_FAILS      = 5;
  constexpr uint32_t AUTH_LOCKOUT_MS     = 60000;
  constexpr uint8_t  LICENSE_MAX_FAILS   = 8;
  constexpr uint32_t LICENSE_LOCKOUT_MS  = 300000;
}
