#include "Arbiter.h"
#include "../core/Config.h"
#include "../core/Types.h"
#include "../core/SharedState.h"
#include "../sensors/Sensors.h"
#include "../persistence/License.h"
#include "../actuator/IgnitionCut.h"
#include <math.h>

namespace {

struct Decision {
  uint16_t   cutMs;
  AssistMode mode;
};
constexpr Decision NONE{0, AssistMode::Idle};

// Limiteurs de cadence par assistance (Core 1 uniquement).
uint32_t s_lastOverrevMs = 0;
uint32_t s_lastPitMs     = 0;
uint32_t s_lastShiftMs   = 0;
uint32_t s_lastLaunchMs  = 0;
uint32_t s_lastSlipMs    = 0;
bool     s_slipArmed     = false;
bool     s_leanBlocked   = false;   // hysteresis du garde-fou anti-angle

// Autorite du slip control selon l'angle / l'IMU : 1 = pleine, 0 = interdit.
// C'est LA securite : pas de coupure de motricite sur l'angle (high-side).
float leanAuthority() {
  int  rollX10, latGx100;
  bool imuOk;
  state::getLean(rollX10, latGx100, imuOk);
  const bool  imuFresh = imuOk && (millis() - state::lastLeanMs() < cfg::IMU_STALE_MS);
  const float absRoll  = fabsf(rollX10 / 10.0f);
  const float latG     = latGx100 / 100.0f;

#if IMU_REQUIRED
  if (!imuFresh) { s_leanBlocked = true; return 0.0f; }   // fail-safe : pas d'IMU -> pas de slip
#else
  if (!imuFresh) return 1.0f;                              // tolere l'absence d'IMU (banc)
#endif

  if (latG > cfg::LEAN_LATERAL_G_HARD || absRoll >= cfg::LEAN_CUTOFF_DEG) {
    s_leanBlocked = true;
    return 0.0f;
  }
  // une fois bloque, il faut repasser bien a plat pour re-autoriser
  if (s_leanBlocked) {
    if (absRoll < cfg::LEAN_REARM_DEG && latG < cfg::LEAN_LATERAL_G_HARD) s_leanBlocked = false;
    else return 0.0f;
  }
  if (absRoll <= cfg::LEAN_TAPER_DEG) return 1.0f;
  return 1.0f - (absRoll - cfg::LEAN_TAPER_DEG) / (cfg::LEAN_CUTOFF_DEG - cfg::LEAN_TAPER_DEG);
}

// ------------------------------------------------------------------ OVERREV
Decision evalOverrev() {
  if (state::getRpmInstant() <= cfg::OVERREV_RPM) return NONE;
  const uint32_t now = millis();
  if (now - s_lastOverrevMs < cfg::OVERREV_PERIOD_MS) return NONE;
  s_lastOverrevMs = now;
  return {cfg::OVERREV_CUT_MS, AssistMode::Overrev};
}

// ---------------------------------------------------------------------- PIT
// Limiteur de voie des stands : en VITESSE (km/h, roue la plus rapide des deux -> une roue qui
// patine ne laisse pas passer la moto). Repli sur le REGIME (pitRpm) si aucune roue ne donne de
// vitesse (capteurs HS, ou moto a l'arret en train d'emballer le moteur dans le box).
Decision evalPit(const EcuConfig& c, int rpm, float frontKmh, float rearKmh) {
  if (!sensors::pitButton()) return NONE;

  const float speed = (frontKmh > rearKmh) ? frontKmh : rearKmh;
  bool over, hard;
  if (speed > cfg::PIT_SPEED_VALID_KMH) {
    over = speed > static_cast<float>(c.pitKmh);
    hard = speed > static_cast<float>(c.pitKmh) + cfg::PIT_HARD_BAND_KMH;
  } else {
    over = rpm > c.pitRpm;
    hard = rpm > c.pitRpm + cfg::PIT_HARD_BAND_RPM;
  }
  if (!over) return NONE;

  const uint32_t now = millis();
  if (now - s_lastPitMs < cfg::PIT_PERIOD_MS) return NONE;
  s_lastPitMs = now;
  // Au-dela de la bande dure : coupure continue (le limiteur doit limiter meme plein gaz).
  if (hard) return {static_cast<uint16_t>(cfg::PIT_PERIOD_MS), AssistMode::PitLimit};
  return {cfg::PIT_CUT_MS, AssistMode::PitLimit};
}

// -------------------------------------------------------------------- SHIFT
Decision evalShift(const EcuConfig& c, int rpm) {
  const sensors::ShiftRequest s = sensors::peekShift();
  if (!s.pending) return NONE;

  const uint32_t age = static_cast<uint32_t>(micros() - s.triggeredMicros);

  // Fenetre d'analyse ecoulee : on abandonne cet appui.
  if (age >= cfg::SHIFT_WINDOW_US) {
    sensors::consumeShift();
    return NONE;
  }
  // Confirmation anti-parasite : la broche doit etre BASSE au moins SHIFT_CONFIRM_US apres le front.
  // Un pic EMI de quelques us a declenche l'ISR mais la broche est deja repassee a HIGH : il expire
  // avec la fenetre sans jamais couper. Un vrai appui rebondissant finit par etre vu bas.
  if (age < cfg::SHIFT_CONFIRM_US) return NONE;
  if (!sensors::shifterHeld()) return NONE;   // pas (ou plus) appuye : on garde le latch jusqu'a la fin de
                                              // la fenetre (rebond de contact = la broche rebaisse ; parasite = non)

  const uint32_t now = millis();
  if (rpm >= c.minShiftRpm && (now - s_lastShiftMs) >= cfg::SHIFT_LOCKOUT_MS) {
    s_lastShiftMs = now;
    sensors::consumeShift();
    return {static_cast<uint16_t>(c.shiftCutMs), AssistMode::Shift};
  }
  return NONE;  // on garde le latch : nouvelle tentative dans la fenetre
}

// ------------------------------------------------------------------- LAUNCH
Decision evalLaunch(const EcuConfig& c, int rpm, float speedFrontKmh, float speedRearKmh) {
  // Les DEUX roues doivent etre quasi arretees : un capteur avant mort/debranche lit 0 et
  // ferait passer une moto en mouvement (embrayage tire) pour une moto a l'arret.
  if (!sensors::clutchPulled() || speedFrontKmh >= cfg::LAUNCH_SPEED_MAX_KMH ||
      speedRearKmh >= cfg::LAUNCH_SPEED_MAX_KMH) return NONE;
  s_slipArmed = false;  // en zone launch, on desarme le slip control

  const uint32_t now = millis();
  if (now - s_lastLaunchMs < cfg::LAUNCH_PERIOD_MS) return NONE;

  const int diff = rpm - c.launchRpm;
  if (diff <= -cfg::LAUNCH_ARM_BAND_RPM) return NONE;  // trop loin sous la cible

  s_lastLaunchMs = now;
  int ms = constrain(cfg::LAUNCH_CUT_MIN_MS + diff / 50,
                     static_cast<int>(cfg::LAUNCH_CUT_MIN_MS),
                     static_cast<int>(cfg::LAUNCH_CUT_MAX_MS));
  // Bande dure : le regime depasse nettement la cible -> coupure continue (2-step), sinon un
  // moteur a vide gagne plus de regime que la modulation (<= 40 % de coupure) n'en retire.
  if (diff > cfg::LAUNCH_HARD_BAND_RPM) ms = static_cast<int>(cfg::LAUNCH_PERIOD_MS);
  return {static_cast<uint16_t>(ms), AssistMode::Launch};
}

// --------------------------------------------------------------------- SLIP
Decision evalSlip(const EcuConfig& c, float speedFrontKmh) {
  // GARDE-FOU ANTI-ANGLE (securite) : evalue en premier.
  const float authority = leanAuthority();
  if (authority <= 0.0f) { s_slipArmed = false; return NONE; }

  int slipX10, rateX10;
  state::getSlip(slipX10, rateX10);
  const float slip = slipX10 / 10.0f;
  const float rate = rateX10 / 10.0f;

  // Terme d'avance sur la derivee de glisse (borne serre : voir critique dans le
  // README - avec la vitesse roue mesuree en periode, la derivee est exploitable).
  const float predictionDelta = constrain(rate * cfg::SLIP_PREDICT_SEC,
                                          -cfg::SLIP_PREDICT_CLAMP_PCT, cfg::SLIP_PREDICT_CLAMP_PCT);
  const float predictedSlip   = slip + predictionDelta;
  const float excess          = predictedSlip - static_cast<float>(c.maxSlipPercent);

  // Hysterese d'armement / desarmement.
  if (!s_slipArmed) {
    if (excess > 0.0f && speedFrontKmh > cfg::SLIP_MIN_SPEED_KMH) s_slipArmed = true;
  } else if (slip < (c.maxSlipPercent - cfg::SLIP_HYSTERESIS_PCT) ||
             speedFrontKmh <= cfg::SLIP_MIN_SPEED_KMH) {
    s_slipArmed = false;
  }
  if (!s_slipArmed || excess <= 0.0f) return NONE;

  const uint32_t now = millis();
  if (now - s_lastSlipMs < cfg::SLIP_PERIOD_MS) return NONE;

  const float speedFactor  = constrain(speedFrontKmh / 50.0f, 0.5f, 1.5f);
  const float lowSpeedRamp  = constrain(
      (speedFrontKmh - cfg::SLIP_MIN_SPEED_KMH) / cfg::SLIP_LOWSPEED_RAMP_KMH, 0.0f, 1.0f);
  // authority (<1 quand la moto s'incline) reduit progressivement la coupure.
  const float cutF = (cfg::SLIP_CUT_MIN_MS + excess * 1.5f * speedFactor) * lowSpeedRamp * authority;
  if (cutF < cfg::SLIP_CUT_MIN_MS) return NONE;

  s_lastSlipMs = now;
  const int ms = constrain(static_cast<int>(lroundf(cutF)),
                           static_cast<int>(cfg::SLIP_CUT_MIN_MS),
                           static_cast<int>(cfg::SLIP_CUT_MAX_MS));
  return {static_cast<uint16_t>(ms), AssistMode::Slip};
}

}  // namespace

void arbiter::update() {
  const EcuConfig   c   = state::getConfig();
  const LicenseTier lic = license::tier();
  const int         rpm = state::getRpmFiltered();

  int frontX10, rearX10;
  state::getSpeeds(frontX10, rearX10);
  const float speedFrontKmh = frontX10 / 10.0f;
  const float speedRearKmh  = rearX10 / 10.0f;

  Decision d = evalOverrev();
  if (d.cutMs == 0) d = evalPit(c, rpm, speedFrontKmh, speedRearKmh);
  if (d.cutMs == 0) d = evalShift(c, rpm);
  if (d.cutMs == 0 && lic >= LicenseTier::Track) d = evalLaunch(c, rpm, speedFrontKmh, speedRearKmh);
  if (d.cutMs == 0 && lic >= LicenseTier::Pro)   d = evalSlip(c, speedFrontKmh);

  if (d.cutMs > 0) ignition::request(d.cutMs, d.mode);
}
