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
}

// Les macros GPIO.out_w1ts / w1tc n'adressent que les GPIO 0..31.
static_assert(pins::CUT >= 0 && pins::CUT < 32,
              "pins::CUT doit etre < 32 (registre GPIO.out_w1ts/w1tc)");

namespace cfg {
  // ---- RPM / filtre EMI --------------------------------------------------
  constexpr int      OVERREV_RPM        = 16000;   // limiteur dur "survie moteur"
  constexpr int      RPM_MAX_PLAUSIBLE  = 20000;   // au-dela = parasite EMI (spec)
  constexpr uint32_t RPM_MIN_DELTA_US   = (60UL * 1000000UL) / RPM_MAX_PLAUSIBLE; // ~3000 us
  constexpr float    RPM_SLEW_RPM_PER_S = 120000.0f; // pente max admise (tr/min/s) : clamp, pas rejet
  constexpr uint32_t RPM_STALL_US       = 500000;  // sans impulsion VALIDEE => moteur cale
  constexpr int      RPM_FILTER_SHIFT   = 2;       // EWMA : alpha = 1 / 2^shift = 0.25

  // ---- Overrev limiter -------------------------------------------------
  constexpr uint16_t OVERREV_CUT_MS     = 50;
  constexpr uint32_t OVERREV_PERIOD_MS  = 40;

  // ---- Pit limiter ---------------------------------------------------
  constexpr uint16_t PIT_CUT_MS         = 20;
  constexpr uint32_t PIT_PERIOD_MS      = 50;

  // ---- Quickshifter ------------------------------------------------
  constexpr uint32_t SHIFT_WINDOW_US       = 25000; // fenetre d'analyse de l'appui
  constexpr uint32_t SHIFT_ISR_DEBOUNCE_US = 30000; // anti-rebond materiel (ISR)
  constexpr uint32_t SHIFT_LOCKOUT_MS      = 250;   // verrou anti double-coupure

  // ---- Launch control -------------------------------------------
  constexpr float    LAUNCH_SPEED_MAX_KMH = 5.0f;  // au-dela : on passe en roulage
  constexpr int      LAUNCH_ARM_BAND_RPM  = 200;   // s'active des launchRpm - 200
  constexpr uint16_t LAUNCH_CUT_MIN_MS    = 5;
  constexpr uint16_t LAUNCH_CUT_MAX_MS    = 20;
  constexpr uint32_t LAUNCH_PERIOD_MS     = 50;

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

  // ---- Capteurs de roue / entrees -------------------------
  constexpr uint32_t WHEEL_MIN_PULSE_US   = 0;    // 0 = pas d'anti-rebond (VR/Hall). >0 pour reed.
  constexpr uint32_t SPEED_CALC_MS        = 40;   // periode de calcul vitesse / glisse
  constexpr float    WHEEL_SPEED_MAX_KMH  = 160.0f; // vitesse impossible pour un 50cc -> rejet glitch
  constexpr float    WHEEL_SPEED_SLEW_KMH = 25.0f;  // variation max plausible entre 2 fenetres
  constexpr uint32_t WHEEL_STOP_US        = 400000; // sans impulsion depuis -> roue arretee
  constexpr uint32_t INPUT_DEBOUNCE_MS    = 15;   // anti-rebond bouton pit / contacteur embrayage

  // ---- Bornes de validation NVS (partagees web + chargement) --------
  constexpr int SHIFT_CUT_MIN = 10,   SHIFT_CUT_MAX = 150;
  constexpr int MIN_RPM_MIN   = 2000, MIN_RPM_MAX   = 15000;
  constexpr int LAUNCH_MIN    = 2000, LAUNCH_MAX    = 15000;
  constexpr int SLIP_MIN      = 0,    SLIP_MAX      = 50;
  constexpr int WHEEL_MIN     = 1000, WHEEL_MAX     = 2500;
  constexpr int PULSES_MIN    = 1,    PULSES_MAX    = 20;
  constexpr int PIT_MIN       = 2000, PIT_MAX       = 10000;

  // ---- Cartographie par defaut ---------------------------------------
  // Ordre = declaration de EcuConfig :
  // shiftCutMs, minShiftRpm, launchRpm, maxSlipPercent, wheelCircumMm, pulsesPerRev, pitRpm
  constexpr EcuConfig DEFAULTS = { 50, 6000, 8000, 15, 1800, 3, 4000 };

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
