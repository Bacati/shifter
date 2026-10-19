#include "WaterTemp.h"
#include "../core/Config.h"
#include "../core/SharedState.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace {
constexpr uint32_t CONVERSION_MS = 800;   // DS18B20 12 bits : 750 ms max + marge

OneWire           s_bus(pins::WATER_TEMP);
DallasTemperature s_sensors(&s_bus);
DeviceAddress     s_addr;
bool              s_present = false;
uint8_t           s_fails   = 0;       // lectures ratees consecutives
bool              s_haveGood = false;  // au moins une lecture valide depuis la detection
float             s_lastGood = 0.0f;

bool probe() {
  s_sensors.begin();
  s_present = s_sensors.getDeviceCount() > 0 && s_sensors.getAddress(s_addr, 0);
  if (s_present) {
    s_sensors.setResolution(s_addr, 12);
    // Conversion NON bloquante : l'attente se fait par vTaskDelay (une seule fois, par la tache).
    s_sensors.setWaitForConversion(false);
  }
  s_fails    = 0;
  s_haveGood = false;
  return s_present;
}

// Valeur plausible ET pas la valeur d'usine : un DS18B20 dont la conversion n'a pas eu lieu
// (alim parasite faible, reset) renvoie exactement 85.0 C -- c'est une plage d'eau normale,
// donc invisible au test de plage. On ne l'accepte que si la lecture precedente en est proche.
bool plausible(float c) {
  if (c == DEVICE_DISCONNECTED_C || !isfinite(c)) return false;
  if (c < cfg::WATER_TEMP_MIN_C || c > cfg::WATER_TEMP_MAX_C) return false;
  if (fabsf(c - cfg::WATER_TEMP_POR_C) < 0.01f)
    return s_haveGood && fabsf(s_lastGood - cfg::WATER_TEMP_POR_C) < 5.0f;
  return true;
}

void task(void*) {
  for (;;) {
    if (!s_present) {
      state::setWaterTemp(0, false);
      vTaskDelay(pdMS_TO_TICKS(cfg::WATER_TEMP_PROBE_MS));
      probe();
      continue;
    }

    s_sensors.requestTemperatures();     // retourne immediatement (setWaitForConversion(false))
    vTaskDelay(pdMS_TO_TICKS(CONVERSION_MS));   // ne bloque jamais loop() (Core 1)

    const float c = s_sensors.getTempC(s_addr);   // CRC verifie par la bibliotheque
    if (plausible(c)) {
      s_fails = 0;
      s_haveGood = true;
      s_lastGood = c;
      state::setWaterTemp(static_cast<int>(lroundf(c * 10.0f)), true);
    } else if (++s_fails >= cfg::WATER_TEMP_MAX_FAILS) {
      // Defaut confirme. Un echec isole (glitch bus, CRC) est tolere : la derniere valeur
      // reste affichee, et SharedState l'invalide de toute facon si elle devient trop vieille.
      state::setWaterTemp(0, false);
      s_present = false;                 // retente la detection
    }

    const uint32_t cycleMs = cfg::WATER_TEMP_PERIOD_MS;
    if (cycleMs > CONVERSION_MS) vTaskDelay(pdMS_TO_TICKS(cycleMs - CONVERSION_MS));
  }
}
}  // namespace

bool watertemp::begin() {
  const bool ok = probe();
  Serial.println(ok ? "[TEMP] DS18B20 OK" : "[TEMP] DS18B20 absent (reessaiera periodiquement)");
  state::setWaterTemp(0, false);
  return ok;
}

void watertemp::startTask() {
  xTaskCreatePinnedToCore(task, "watertemp", 2560, nullptr, 1, nullptr, cfg::CORE_NET);
}

bool watertemp::present() { return s_present; }
