#include "Sensors.h"
#include "../core/Config.h"
#include "../core/SharedState.h"
#include <math.h>

namespace {

// ---- Spinlock partage ISR <-> tache --------------------------------------
portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// ---- RPM (ecrit par isr_rpm) -------------------------------------------
//  s_rpmLastAcceptedUs sert A LA FOIS de reference d'intervalle ET de detection
//  de calage : une impulsion EMI rejetee ne doit pas faire croire que le moteur
//  "vit". Il n'y a donc volontairement pas de "s_rpmLastEdgeUs" separe.
volatile uint32_t s_rpmLastAcceptedUs = 0;  // derniere impulsion VALIDEE
volatile uint32_t s_rpmLastDeltaUs    = 0;
volatile uint32_t s_rpmEdgeSeq        = 0;

// ---- Roues (ecrit par isr_wheel_*) : comptage + horodatage 1ere/derniere impulsion
volatile uint32_t s_frontCount   = 0;
volatile uint32_t s_frontFirstUs = 0;
volatile uint32_t s_frontLastUs  = 0;
volatile uint32_t s_rearCount    = 0;
volatile uint32_t s_rearFirstUs  = 0;
volatile uint32_t s_rearLastUs   = 0;

// ---- Shifter (ecrit par isr_shifter) ----------------------------------
volatile bool     s_shiftPending = false;
volatile uint32_t s_shiftAtUs    = 0;
volatile uint32_t s_shiftLastUs  = 0;

// ---- Etat tache (Core 1 uniquement) ----------------------------------
int      s_rpmFiltered  = 0;
int      s_rpmInstant   = 0;
uint32_t s_seenSeq      = 0;
int      s_lockCount    = 0;   // periodes coherentes consecutives depuis l'arret
uint32_t s_lockPrev     = 0;   // periode precedente (us)
int      s_badStreak    = 0;   // periodes consecutives incoherentes alors que verrouille

uint32_t s_lastSpeedMs  = 0;
bool     s_speedSeeded  = false;
float    s_lastSlip     = 0.0f;
float    s_slipRateFilt = 0.0f;
uint32_t s_frontPrevLastUs = 0;
uint32_t s_rearPrevLastUs  = 0;
float    s_frontSpeedKmh   = 0.0f;
float    s_rearSpeedKmh    = 0.0f;
float    s_frontMeasKmh    = 0.0f;   // derniere vitesse MESUREE (non decroissante) de la roue avant
uint32_t s_frontPulseUs    = 0;      // instant de la derniere impulsion avant vue par la tache
float    s_slipHist[cfg::SLIP_CONFIRM_WINDOWS] = {0};   // glisses brutes recentes (filtre anti-parasite)
uint8_t  s_slipHistIdx     = 0;

struct Debounced {
  bool     stable  = false;
  bool     lastRaw = false;
  uint32_t since   = 0;
};
Debounced s_pit;
Debounced s_clutch;

// ============================ ISR ======================================

void IRAM_ATTR isr_rpm() {
  const uint32_t now = micros();
  portENTER_CRITICAL_ISR(&s_mux);
  const uint32_t delta = now - s_rpmLastAcceptedUs;
  if (delta >= cfg::RPM_MIN_DELTA_US) {  // rejet EMI etage 1 : frequence impossible
    s_rpmLastAcceptedUs = now;           // MAJ uniquement sur impulsion validee
    s_rpmLastDeltaUs    = delta;
    s_rpmEdgeSeq++;
  }
  portEXIT_CRITICAL_ISR(&s_mux);
}

void IRAM_ATTR isr_wheel_front() {
  const uint32_t now = micros();
  portENTER_CRITICAL_ISR(&s_mux);
  if (cfg::WHEEL_MIN_PULSE_US == 0 ||
      static_cast<uint32_t>(now - s_frontLastUs) >= cfg::WHEEL_MIN_PULSE_US) {
    if (s_frontCount == 0) s_frontFirstUs = now;
    s_frontLastUs = now;
    s_frontCount++;
  }
  portEXIT_CRITICAL_ISR(&s_mux);
}

void IRAM_ATTR isr_wheel_rear() {
  const uint32_t now = micros();
  portENTER_CRITICAL_ISR(&s_mux);
  if (cfg::WHEEL_MIN_PULSE_US == 0 ||
      static_cast<uint32_t>(now - s_rearLastUs) >= cfg::WHEEL_MIN_PULSE_US) {
    if (s_rearCount == 0) s_rearFirstUs = now;
    s_rearLastUs = now;
    s_rearCount++;
  }
  portEXIT_CRITICAL_ISR(&s_mux);
}

void IRAM_ATTR isr_shifter() {
  const uint32_t now = micros();
  portENTER_CRITICAL_ISR(&s_mux);
  if (static_cast<uint32_t>(now - s_shiftLastUs) > cfg::SHIFT_ISR_DEBOUNCE_US) {
    s_shiftLastUs = now;
    if (!s_shiftPending) {  // latch : un seul evenement jusqu'a consommation
      s_shiftPending = true;
      s_shiftAtUs    = now;
    }
  }
  portEXIT_CRITICAL_ISR(&s_mux);
}

// ======================= Traitement (tache) ============================

void resetDynamics() {
  s_lastSlip     = 0.0f;
  s_slipRateFilt = 0.0f;
}

void updateRpm() {
  uint32_t seq, delta, lastAcceptedUs;
  portENTER_CRITICAL(&s_mux);
  seq            = s_rpmEdgeSeq;
  delta          = s_rpmLastDeltaUs;
  lastAcceptedUs = s_rpmLastAcceptedUs;
  portEXIT_CRITICAL(&s_mux);

  // Moteur cale : aucune impulsion VALIDEE depuis RPM_STALL_US (les parasites EMI
  // rejetes ne comptent pas -> une tempete EMI ne maintient pas le moteur "vivant").
  if (static_cast<uint32_t>(micros() - lastAcceptedUs) > cfg::RPM_STALL_US) {
    if (s_rpmFiltered != 0 || s_rpmInstant != 0) {
      s_rpmFiltered = 0;
      s_rpmInstant  = 0;
      state::setRpmFiltered(0);
      state::setRpmInstant(0);
      resetDynamics();
    }
    s_lockCount = 0;
    s_lockPrev  = 0;
    s_badStreak = 0;
    s_seenSeq = seq;
    return;
  }

  if (seq == s_seenSeq || delta < cfg::RPM_MIN_DELTA_US) return;  // pas de nouvelle impulsion
  s_seenSeq = seq;

  int inst = static_cast<int>(60000000UL / delta);
  if (inst > cfg::RPM_MAX_PLAUSIBLE) inst = cfg::RPM_MAX_PLAUSIBLE;  // borne dure

  if (s_rpmInstant == 0) {
    // Pas encore verrouille : on exige des periodes coherentes ET plausibles pour un moteur
    // (<= OVERREV_RPM). Du bruit aleatoire (ou continu, ~19 000 tr/min) ne verrouille jamais.
    const bool plausible = delta >= 60000000UL / cfg::OVERREV_RPM;
    if (!plausible) {
      s_lockCount = 0;
      s_lockPrev  = 0;
      return;
    }
    const uint32_t diff = (delta > s_lockPrev) ? delta - s_lockPrev : s_lockPrev - delta;
    const bool consistent = s_lockPrev != 0 && diff * 100UL <= s_lockPrev * cfg::RPM_LOCK_TOL_PCT;
    s_lockCount = consistent ? s_lockCount + 1 : 0;
    s_lockPrev  = delta;
    if (s_lockCount < cfg::RPM_LOCK_CONSISTENT) return;
    s_rpmInstant  = inst;
    s_rpmFiltered = inst;
    s_badStreak = 0;
  } else {
    // Verrouille : le signal reste-t-il compatible avec un moteur ? (arret + bruit -> non)
    const float expected = 60000000.0f / static_cast<float>(s_rpmFiltered > 0 ? s_rpmFiltered : 1);
    if (delta < cfg::RPM_BAD_LO * expected || delta > cfg::RPM_BAD_HI * expected) {
      if (++s_badStreak >= cfg::RPM_UNLOCK_BAD) {
        s_rpmFiltered = s_rpmInstant = 0;
        state::setRpmFiltered(0);
        state::setRpmInstant(0);
        resetDynamics();
        s_lockCount = 0; s_lockPrev = 0; s_badStreak = 0;
        return;
      }
    } else {
      s_badStreak = 0;
    }
    // Limitation de pente TEMPORELLE : la variation admise est proportionnelle a
    // la duree ecoulee, pas fixe par impulsion. On CLAMPE (on ne rejette pas) ->
    // un spike EMI est borne, et une impulsion manquee ne "latche" pas
    // l'estimation a une valeur fausse.
    const float dtSec   = delta / 1000000.0f;
    const int   maxStep = static_cast<int>(cfg::RPM_SLEW_RPM_PER_S * dtSec) + 50;
    s_rpmInstant = constrain(inst, s_rpmInstant - maxStep, s_rpmInstant + maxStep);
    s_rpmFiltered += (s_rpmInstant - s_rpmFiltered) / (1 << cfg::RPM_FILTER_SHIFT);
  }

  state::setRpmInstant(s_rpmInstant);
  state::setRpmFiltered(s_rpmFiltered);
}

// Vitesse d'une roue par MESURE DE PERIODE (pas de quantification de fenetre) :
//  - >= 2 impulsions : v sur (count-1) intervalles exacts.
//  - 1 impulsion     : intervalle depuis la derniere impulsion connue.
//  - 0 impulsion     : la vitesse ne peut pas depasser "une impulsion serait deja
//                      arrivee" ; au-dela de WHEEL_STOP_US -> roue arretee.
//  + rejet des valeurs impossibles (glitch) et limitation de pente (slew).
float wheelSpeedKmh(uint32_t count, uint32_t firstUs, uint32_t lastUs,
                    uint32_t& prevLastUs, float& lastSpeed,
                    float distPerPulseMm, uint32_t nowUs, uint32_t stopUs) {
  const float k = distPerPulseMm / 1000.0f * 3.6f;   // mm/us -> km/h : (m) / (s) * 3.6
  float v;

  if (count >= 2) {
    const float dtSec = (lastUs - firstUs) / 1000000.0f;
    v = (dtSec > 0.0f) ? (count - 1) * k / dtSec : lastSpeed;
    prevLastUs = lastUs;
  } else if (count == 1) {
    const float dtSec = (lastUs - prevLastUs) / 1000000.0f;
    v = (dtSec > 0.0f) ? k / dtSec : lastSpeed;
    prevLastUs = lastUs;
  } else {
    const uint32_t sinceUs = nowUs - prevLastUs;
    if (sinceUs > stopUs) { lastSpeed = 0.0f; return 0.0f; }
    const float vmax = (sinceUs > 0) ? k / (sinceUs / 1000000.0f) : lastSpeed;
    v = fminf(lastSpeed, vmax);
  }

  if (v > cfg::WHEEL_SPEED_MAX_KMH) return lastSpeed;          // impossible pour un 50cc
  if (lastSpeed > 1.0f && fabsf(v - lastSpeed) > cfg::WHEEL_SPEED_SLEW_KMH)
    v = lastSpeed + copysignf(cfg::WHEEL_SPEED_SLEW_KMH, v - lastSpeed);  // impulsion manquee/doublee

  lastSpeed = v;
  return v;
}

void updateDynamics() {
  const uint32_t nowMs = millis();
  const uint32_t nowUs = micros();

  if (!s_speedSeeded) {  // ignore la 1ere fenetre (dt indefini au demarrage)
    portENTER_CRITICAL(&s_mux);
    s_frontCount = s_rearCount = 0;
    portEXIT_CRITICAL(&s_mux);
    s_frontPrevLastUs = s_rearPrevLastUs = nowUs;
    s_lastSpeedMs = nowMs;
    s_speedSeeded = true;
    return;
  }

  const uint32_t dtMs = nowMs - s_lastSpeedMs;
  if (dtMs < cfg::SPEED_CALC_MS) return;
  s_lastSpeedMs = nowMs;

  const EcuConfig c   = state::getConfig();
  const int       ppr = (c.pulsesPerRev < 1) ? 1 : c.pulsesPerRev;
  const float distPerPulseMm = static_cast<float>(c.wheelCircumMm) / static_cast<float>(ppr);

  // Delai avant de declarer la roue arretee : au moins WHEEL_STOP_US, et assez long pour que
  // WHEEL_STOP_KMH produise encore des impulsions (sinon, avec peu d'impulsions/tour, une roue
  // qui tourne lentement serait lue comme arretee -- ex. 1 impulsion/tour : < 15 km/h = "0").
  const float stopF = distPerPulseMm * (3.6f / cfg::WHEEL_STOP_KMH) * 1000.0f;   // us
  uint32_t stopUs = (stopF > static_cast<float>(cfg::WHEEL_STOP_MAX_US)) ? cfg::WHEEL_STOP_MAX_US
                                                                          : static_cast<uint32_t>(stopF);
  if (stopUs < cfg::WHEEL_STOP_US) stopUs = cfg::WHEEL_STOP_US;

  uint32_t fc, ff, fl, rc, rf, rl;
  portENTER_CRITICAL(&s_mux);
  fc = s_frontCount; ff = s_frontFirstUs; fl = s_frontLastUs; s_frontCount = 0;
  rc = s_rearCount;  rf = s_rearFirstUs;  rl = s_rearLastUs;  s_rearCount  = 0;
  portEXIT_CRITICAL(&s_mux);

  const float vFront = wheelSpeedKmh(fc, ff, fl, s_frontPrevLastUs, s_frontSpeedKmh, distPerPulseMm, nowUs, stopUs);
  if (fc > 0) { s_frontMeasKmh = vFront; s_frontPulseUs = fl; }
  const float vRear  = wheelSpeedKmh(rc, rf, rl, s_rearPrevLastUs,  s_rearSpeedKmh,  distPerPulseMm, nowUs, stopUs);

  // Roue avant "vivante" ? Sinon (capteur debranche) sa vitesse estimee decroit et ferait une fausse glisse.
  bool frontFresh = false;
  if (s_frontMeasKmh > 0.5f) {
    const float periodUs = (distPerPulseMm / 1000.0f) / (s_frontMeasKmh / 3.6f) * 1000000.0f;
    float limitUs = periodUs * cfg::SLIP_FRONT_FRESH_PERIODS;
    if (limitUs < static_cast<float>(cfg::SLIP_FRONT_FRESH_MIN_US)) limitUs = static_cast<float>(cfg::SLIP_FRONT_FRESH_MIN_US);
    frontFresh = static_cast<float>(static_cast<uint32_t>(nowUs - s_frontPulseUs)) < limitUs;
  }

  float slipRaw = 0.0f;
  if (frontFresh && vFront > cfg::SLIP_MIN_SPEED_KMH)
    slipRaw = (vRear - vFront) / vFront * 100.0f;
  // Glisse retenue = minimum des SLIP_CONFIRM_WINDOWS dernieres fenetres : un parasite sur une roue
  // (qui fausse 2 intervalles consecutifs) ne produit pas de coupure, une vraie perte
  // d'adherence (plusieurs centaines de ms) est vue avec ~SLIP_CONFIRM_WINDOWS x SPEED_CALC_MS de retard.
  s_slipHist[s_slipHistIdx] = slipRaw;
  s_slipHistIdx = static_cast<uint8_t>((s_slipHistIdx + 1) % cfg::SLIP_CONFIRM_WINDOWS);
  float slip = s_slipHist[0];
  for (int i = 1; i < cfg::SLIP_CONFIRM_WINDOWS; ++i) slip = fminf(slip, s_slipHist[i]);

  const float dtSec   = dtMs / 1000.0f;
  const float rawRate = (slip - s_lastSlip) / dtSec;
  s_slipRateFilt = rawRate * cfg::SLIP_RATE_FILTER_A +
                   s_slipRateFilt * (1.0f - cfg::SLIP_RATE_FILTER_A);
  s_lastSlip = slip;

  state::setSpeeds(static_cast<int>(vFront * 10.0f), static_cast<int>(vRear * 10.0f));
  state::setSlip(static_cast<int>(slip * 10.0f), static_cast<int>(s_slipRateFilt * 10.0f));
}

bool pollDebounce(Debounced& d, int pin) {
  const bool     raw = (digitalRead(pin) == LOW);
  const uint32_t now = millis();
  if (raw != d.lastRaw) {
    d.lastRaw = raw;
    d.since   = now;
  } else if (raw != d.stable &&
             static_cast<uint32_t>(now - d.since) >= cfg::INPUT_DEBOUNCE_MS) {
    d.stable = raw;
  }
  return d.stable;
}

}  // namespace

// ============================== API ===================================

void sensors::begin() {
  pinMode(pins::RPM,         INPUT_PULLUP);
  pinMode(pins::WHEEL_FRONT, INPUT_PULLUP);
  pinMode(pins::WHEEL_REAR,  INPUT_PULLUP);
  pinMode(pins::SHIFTER,     INPUT_PULLUP);
  pinMode(pins::CLUTCH_HALL, INPUT_PULLUP);
  pinMode(pins::PIT_BTN,     INPUT_PULLUP);

  s_pit.stable    = s_pit.lastRaw    = (digitalRead(pins::PIT_BTN)     == LOW);
  s_clutch.stable = s_clutch.lastRaw = (digitalRead(pins::CLUTCH_HALL) == LOW);

  attachInterrupt(digitalPinToInterrupt(pins::RPM),         isr_rpm,         FALLING);
  attachInterrupt(digitalPinToInterrupt(pins::WHEEL_FRONT), isr_wheel_front, RISING);
  attachInterrupt(digitalPinToInterrupt(pins::WHEEL_REAR),  isr_wheel_rear,  RISING);
  attachInterrupt(digitalPinToInterrupt(pins::SHIFTER),     isr_shifter,     FALLING);
}

void sensors::update() {
  updateRpm();
  updateDynamics();
  pollDebounce(s_pit,    pins::PIT_BTN);
  pollDebounce(s_clutch, pins::CLUTCH_HALL);
}

sensors::ShiftRequest sensors::peekShift() {
  ShiftRequest r;
  portENTER_CRITICAL(&s_mux);
  r.pending         = s_shiftPending;
  r.triggeredMicros = s_shiftAtUs;
  portEXIT_CRITICAL(&s_mux);
  return r;
}

void sensors::consumeShift() {
  portENTER_CRITICAL(&s_mux);
  s_shiftPending = false;
  portEXIT_CRITICAL(&s_mux);
}

bool sensors::shifterHeld()  { return digitalRead(pins::SHIFTER) == LOW; }
bool sensors::pitButton()    { return s_pit.stable; }
bool sensors::clutchPulled() { return s_clutch.stable; }
bool sensors::engineStalled() { return state::getRpmFiltered() == 0; }
