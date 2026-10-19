#include "Imu.h"
#include "../core/Config.h"
#include "../core/SharedState.h"
#include <Wire.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace {

// ---- Registres MPU6050 ----
constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
constexpr uint8_t REG_CONFIG       = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t REG_WHO_AM_I     = 0x75;

constexpr float ACC_LSB_PER_G      = 16384.0f;  // +/- 2 g
constexpr float GYRO_LSB_PER_DPS   = 131.0f;    // +/- 250 deg/s
constexpr float RAD2DEG            = 57.29578f;

bool     s_present   = false;
uint8_t  s_failCount = 0;
float    s_roll      = 0.0f;     // deg, estimation courante
float    s_gyroBias  = 0.0f;     // deg/s sur l'axe de roulis
uint32_t s_lastUs    = 0;
bool     s_rollValid = false;    // true apres un recalage moto quasi droite (voir Config.h)
int      s_validScore = 0;       // score de convergence de l'angle estime
bool     s_biasValid = false;    // true apres une mesure de biais gyro CALME
float    s_winSum = 0, s_winSq = 0;   // fenetre de re-mesure du biais (somme, somme des carres)
int      s_winN = 0;
uint8_t  s_raw[14]   = {0};      // derniere trame brute
uint8_t  s_prevRaw[14] = {0};
uint8_t  s_sameCount = 0;        // trames identiques consecutives (capteur fige)

bool wr(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(cfg::IMU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool rd(uint8_t reg, uint8_t* buf, size_t n) {
  Wire.beginTransmission(cfg::IMU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  const size_t got = Wire.requestFrom(static_cast<int>(cfg::IMU_ADDR), static_cast<int>(n));
  if (got != n) return false;
  for (size_t i = 0; i < n; ++i) buf[i] = Wire.read();
  return true;
}

int16_t be16(const uint8_t* p) { return static_cast<int16_t>((p[0] << 8) | p[1]); }

// axe 0/1/2 -> valeur
float pick(const float v[3], int axis) { return v[(axis < 0 || axis > 2) ? 0 : axis]; }

// Lecture brute -> accel (g) et gyro (deg/s), true si OK.
bool sample(float acc[3], float gyro[3]) {
  uint8_t* b = s_raw;
  if (!rd(REG_ACCEL_XOUT_H, b, 14)) return false;
  acc[0]  = be16(b + 0) / ACC_LSB_PER_G;
  acc[1]  = be16(b + 2) / ACC_LSB_PER_G;
  acc[2]  = be16(b + 4) / ACC_LSB_PER_G;
  gyro[0] = be16(b + 8)  / GYRO_LSB_PER_DPS;
  gyro[1] = be16(b + 10) / GYRO_LSB_PER_DPS;
  gyro[2] = be16(b + 12) / GYRO_LSB_PER_DPS;
  // plausibilite : le MPU6050 sature a +/-2 g / +/-250 dps -> hors de ca = bus KO
  for (int i = 0; i < 3; ++i)
    if (!isfinite(acc[i]) || !isfinite(gyro[i]) || fabsf(acc[i]) > 4.0f || fabsf(gyro[i]) > 300.0f)
      return false;
  return true;
}

// Mesure du biais au (re)demarrage. N'est ACCEPTEE que si le signal est calme (ecart-type <=
// IMU_BIAS_QUIET_DPS) ; sinon (IMU reinitialisee en roulant) on garde le biais precedent, et
// s_biasValid reste ce qu'il etait.
bool wheelsStopped() {
  int f, r;
  state::getSpeeds(f, r);
  const int lim = static_cast<int>(cfg::IMU_BIAS_STOP_KMH * 10.0f);
  return f < lim && r < lim;
}

bool quiet(float sum, float sq, int n, float* mean) {
  if (n < 50) return false;
  *mean = sum / n;
  const float var = sq / n - (*mean) * (*mean);
  return var <= cfg::IMU_BIAS_QUIET_DPS * cfg::IMU_BIAS_QUIET_DPS;
}

void estimateBias() {
  float sum = 0.0f, sq = 0.0f;
  int   n   = 0;
  bool  moved = false;
  const uint32_t end = millis() + cfg::IMU_BIAS_SETTLE_MS;
  while (millis() < end) {
    float a[3], g[3];
    if (!wheelsStopped()) moved = true;
    if (sample(a, g)) {
      const float v = pick(g, cfg::IMU_GYRO_ROLL_AXIS);
      sum += v; sq += v * v; ++n;
    }
    delay(3);
  }
  float mean;
  if (!moved && quiet(sum, sq, n, &mean)) {
    s_gyroBias  = mean;
    s_biasValid = true;
  }
  s_winSum = 0; s_winSq = 0; s_winN = 0;
}

// Re-mesure continue : si, sur 1 s, le gyro de roulis est calme, sa moyenne EST le biais (en
// virage stabilise la vitesse de roulis est nulle). Rend valide un biais inconnu (demarrage en
// mouvement) et suit la derive ; un saut > IMU_BIAS_MAX_STEP_DPS sur un biais deja valide est rejete.
void trackBias(float rawRollGyro) {
  if (!wheelsStopped()) { s_winSum = 0; s_winSq = 0; s_winN = 0; return; }   // en roulant : pas de mesure de biais
  s_winSum += rawRollGyro; s_winSq += rawRollGyro * rawRollGyro; ++s_winN;
  if (s_winN < cfg::IMU_BIAS_WINDOW_SAMPLES) return;
  float mean;
  if (quiet(s_winSum, s_winSq, s_winN, &mean) &&
      (!s_biasValid || fabsf(mean - s_gyroBias) <= cfg::IMU_BIAS_MAX_STEP_DPS)) {
    s_gyroBias  = s_biasValid ? 0.5f * (s_gyroBias + mean) : mean;
    s_biasValid = true;
  }
  s_winSum = 0; s_winSq = 0; s_winN = 0;
}

// Identification + configuration du MPU6050. Sert a l'init ET a la re-init apres panne
// (un brownout remet le composant en sleep : il faut le reveiller).
bool configure(uint8_t& who) {
  who = 0;
  if (!rd(REG_WHO_AM_I, &who, 1) || (who != 0x68 && who != 0x70 && who != 0x72)) return false;

  wr(REG_PWR_MGMT_1,   0x01);  // sort du sleep, horloge sur gyro X
  delay(20);
  wr(REG_CONFIG,       0x03);  // DLPF ~44 Hz
  wr(REG_SMPLRT_DIV,   0x04);  // 200 Hz
  wr(REG_GYRO_CONFIG,  0x00);  // +/- 250 deg/s
  wr(REG_ACCEL_CONFIG, 0x00);  // +/- 2 g
  delay(20);

  s_present   = true;
  s_failCount = 0;
  s_sameCount = 0;
  s_roll      = 0.0f;
  s_rollValid = false;          // angle initial = hypothese, pas une mesure
  s_validScore = 0;
  return true;
}

// Compte comme "echec" : bus KO, valeurs impossibles, capteur endormi (|acc| ~ 0)
// ou trame figee. IMU_MAX_FAILS echecs consecutifs -> IMU declaree KO puis re-init.
bool sampleHealthy(float acc[3], float gyro[3]) {
  if (!sample(acc, gyro)) return false;

  const float mag = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
  if (mag < cfg::IMU_ACC_SANE_LO_G) return false;     // un MPU actif lit ~1 g au repos

  if (memcmp(s_raw, s_prevRaw, sizeof(s_raw)) == 0) {
    if (s_sameCount < 255) ++s_sameCount;
  } else {
    s_sameCount = 0;
    memcpy(s_prevRaw, s_raw, sizeof(s_raw));
  }
  return s_sameCount < cfg::IMU_FROZEN_SAMPLES;
}

void task(void*) {
  if (s_present) estimateBias();
  s_lastUs = micros();

  for (;;) {
    // Absent au boot, ou perdu en route : on retente periodiquement (hot-plug, brownout).
    if (!s_present) {
      state::setLean(0, 100, false);   // -> slip control desactive (fail-safe)
      vTaskDelay(pdMS_TO_TICKS(cfg::IMU_REINIT_MS));
      uint8_t who;
      if (configure(who)) {
        Serial.println("[IMU] MPU6050 (re)initialise");
        estimateBias();
        s_lastUs = micros();
      }
      continue;
    }

    vTaskDelay(pdMS_TO_TICKS(cfg::IMU_TASK_PERIOD_MS));

    float acc[3], gyro[3];
    if (!sampleHealthy(acc, gyro)) {
      if (s_failCount < 255) ++s_failCount;
      if (s_failCount >= cfg::IMU_MAX_FAILS) {
        s_present = false;
        Serial.println("[IMU] perdu -> slip control desactive, re-init en cours");
        state::setLean(0, 100, false);
      }
      continue;
    }
    s_failCount = 0;

    const uint32_t now = micros();
    float dt = (now - s_lastUs) / 1000000.0f;
    s_lastUs = now;
    if (dt <= 0.0f || dt > 0.2f) dt = cfg::IMU_TASK_PERIOD_MS / 1000.0f;

    // Roulis par gyro (integration, biais retire).
    trackBias(pick(gyro, cfg::IMU_GYRO_ROLL_AXIS));
    const float rollRate = (pick(gyro, cfg::IMU_GYRO_ROLL_AXIS) - s_gyroBias) * cfg::IMU_ROLL_SIGN;
    s_roll += rollRate * dt;

    // |accel| : sert au blocage dur ET a decider si on peut recaler le gyro.
    const float accMag = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);

    // Recalage sur l'accelero UNIQUEMENT si la moto n'est pas en train de tourner : |acc|
    // dans la plage de confiance ET lacet quasi nul. En virage stabilise l'accelero "voit a
    // plat" meme couche a 25 deg (|acc| = 1/cos(angle) reste < 1,12 g) -> sans le test de
    // lacet l'angle estime serait ramene vers 0 en ~1 s et le garde-fou deviendrait aveugle.
    const bool straight = fabsf(pick(gyro, cfg::IMU_GYRO_YAW_AXIS)) < cfg::IMU_RECAL_MAX_YAW_DPS;
    const bool trusted  = straight && accMag > cfg::IMU_ACC_TRUST_LO_G && accMag < cfg::IMU_ACC_TRUST_HI_G;
    if (trusted) {
      const float rollAcc = atan2f(pick(acc, cfg::IMU_ACC_ROLL_NUM),
                                   pick(acc, cfg::IMU_ACC_ROLL_DEN)) * RAD2DEG * cfg::IMU_ROLL_SIGN;
      s_roll = cfg::IMU_COMP_ALPHA * s_roll + (1.0f - cfg::IMU_COMP_ALPHA) * rollAcc;

      // Fiable seulement quand l'estimation a CONVERGE vers l'accelero, moto droite, de facon soutenue.
      // (s_roll est deja l'accelero filtre passe-bas : robuste aux vibrations, contrairement a rollAcc)
      if (fabsf(s_roll) < cfg::IMU_VALID_MAX_DEG) {
        if (s_validScore < cfg::IMU_VALID_SCORE) ++s_validScore;
      } else if (s_validScore > 0) {
        s_validScore = (s_validScore > 2) ? s_validScore - 2 : 0;
      }
      if (s_validScore >= cfg::IMU_VALID_SCORE) s_rollValid = true;

      // Suivi lent du biais gyro quand tout est calme et coherent.
      if (fabsf(rollRate) < 2.0f && fabsf(rollAcc - s_roll) < 2.0f)
        s_gyroBias += 0.0005f * (pick(gyro, cfg::IMU_GYRO_ROLL_AXIS) - s_gyroBias);
    } else if (!s_rollValid && s_validScore > 0) {
      --s_validScore;                                             // information perdue : on recule
    }

    // Anti-derive : borne l'estimation.
    if (s_roll > 120.0f)  s_roll = 120.0f;
    if (s_roll < -120.0f) s_roll = -120.0f;

    // ok=false tant que l'angle n'a pas ete valide moto droite ET le biais gyro mesure au calme :
    // le slip reste coupe.
    state::setLean(static_cast<int>(s_roll * 10.0f),
                   static_cast<int>(accMag * 100.0f),
                   s_rollValid && s_biasValid);
  }
}

}  // namespace

bool imu::begin() {
  Wire.begin(pins::IMU_SDA, pins::IMU_SCL, cfg::IMU_I2C_HZ);

  uint8_t who = 0;
  if (!configure(who)) {
    s_present = false;
    Serial.printf("[IMU] absent / WHO_AM_I=0x%02X (nouvelles tentatives en tache)\n", who);
    state::setLean(0, 100, false);
    return false;
  }
  Serial.println("[IMU] MPU6050 OK");
  return true;
}

void imu::startTask() {
  // Lancee meme si l'IMU est absente au boot : la tache retente la detection.
  xTaskCreatePinnedToCore(task, "imu", 3072, nullptr, 3, nullptr, cfg::CORE_NET);
}

bool imu::present() { return s_present; }
