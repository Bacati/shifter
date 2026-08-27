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
  uint8_t b[14];
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

void estimateBias() {
  float sum = 0.0f;
  int   n   = 0;
  const uint32_t end = millis() + cfg::IMU_BIAS_SETTLE_MS;
  while (millis() < end) {
    float a[3], g[3];
    if (sample(a, g)) { sum += pick(g, cfg::IMU_GYRO_ROLL_AXIS); ++n; }
    delay(3);
  }
  s_gyroBias = (n > 0) ? sum / n : 0.0f;
}

void task(void*) {
  estimateBias();
  s_lastUs = micros();

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(cfg::IMU_TASK_PERIOD_MS));

    float acc[3], gyro[3];
    if (!sample(acc, gyro)) {
      if (s_failCount < 255) ++s_failCount;
      if (s_failCount >= cfg::IMU_MAX_FAILS) {
        s_present = false;
        state::setLean(0, 100, false);   // -> slip control desactive (fail-safe)
      }
      continue;
    }
    s_failCount = 0;

    const uint32_t now = micros();
    float dt = (now - s_lastUs) / 1000000.0f;
    s_lastUs = now;
    if (dt <= 0.0f || dt > 0.2f) dt = cfg::IMU_TASK_PERIOD_MS / 1000.0f;

    // Roulis par gyro (integration, biais retire).
    const float rollRate = (pick(gyro, cfg::IMU_GYRO_ROLL_AXIS) - s_gyroBias) * cfg::IMU_ROLL_SIGN;
    s_roll += rollRate * dt;

    // |accel| : sert au blocage dur ET a decider si on peut recaler le gyro.
    const float accMag = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);

    // Recalage sur l'accelero UNIQUEMENT si la moto est quasi non sollicitee
    // (sinon l'accelero "voit a plat" en virage stabilise -> il fausserait tout).
    if (accMag > cfg::IMU_ACC_TRUST_LO_G && accMag < cfg::IMU_ACC_TRUST_HI_G) {
      const float rollAcc = atan2f(pick(acc, cfg::IMU_ACC_ROLL_NUM),
                                   pick(acc, cfg::IMU_ACC_ROLL_DEN)) * RAD2DEG * cfg::IMU_ROLL_SIGN;
      s_roll = cfg::IMU_COMP_ALPHA * s_roll + (1.0f - cfg::IMU_COMP_ALPHA) * rollAcc;

      // Suivi lent du biais gyro quand tout est calme et cohERENT.
      if (fabsf(rollRate) < 2.0f && fabsf(rollAcc - s_roll) < 2.0f)
        s_gyroBias += 0.0005f * (pick(gyro, cfg::IMU_GYRO_ROLL_AXIS) - s_gyroBias);
    }

    // Anti-derive : borne l'estimation.
    if (s_roll > 120.0f)  s_roll = 120.0f;
    if (s_roll < -120.0f) s_roll = -120.0f;

    state::setLean(static_cast<int>(s_roll * 10.0f),
                   static_cast<int>(accMag * 100.0f),
                   true);
  }
}

}  // namespace

bool imu::begin() {
  Wire.begin(pins::IMU_SDA, pins::IMU_SCL, cfg::IMU_I2C_HZ);

  uint8_t who = 0;
  if (!rd(REG_WHO_AM_I, &who, 1) || (who != 0x68 && who != 0x70 && who != 0x72)) {
    s_present = false;
    Serial.printf("[IMU] absent / WHO_AM_I=0x%02X\n", who);
    state::setLean(0, 100, false);
    return false;
  }

  wr(REG_PWR_MGMT_1,   0x01);  // sort du sleep, horloge sur gyro X
  delay(20);
  wr(REG_CONFIG,       0x03);  // DLPF ~44 Hz
  wr(REG_SMPLRT_DIV,   0x04);  // 200 Hz
  wr(REG_GYRO_CONFIG,  0x00);  // +/- 250 deg/s
  wr(REG_ACCEL_CONFIG, 0x00);  // +/- 2 g
  delay(20);

  s_present   = true;
  s_failCount = 0;
  s_roll      = 0.0f;
  Serial.println("[IMU] MPU6050 OK");
  return true;
}

void imu::startTask() {
  if (!s_present) return;
  xTaskCreatePinnedToCore(task, "imu", 3072, nullptr, 3, nullptr, cfg::CORE_NET);
}

bool imu::present() { return s_present; }
