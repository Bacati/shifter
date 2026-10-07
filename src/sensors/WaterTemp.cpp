#include "WaterTemp.h"
#include "../core/Config.h"
#include "../core/SharedState.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace {
OneWire           s_bus(pins::WATER_TEMP);
DallasTemperature s_sensors(&s_bus);
bool              s_present = false;

bool probe() {
  s_sensors.begin();
  s_present = s_sensors.getDeviceCount() > 0;
  if (s_present) s_sensors.setResolution(12);
  return s_present;
}

void task(void*) {
  for (;;) {
    if (!s_present) {
      state::setWaterTemp(0, false);
      vTaskDelay(pdMS_TO_TICKS(cfg::WATER_TEMP_PROBE_MS));
      probe();
      continue;
    }

    s_sensors.requestTemperatures();
    vTaskDelay(pdMS_TO_TICKS(750));  // conversion DS18B20 12 bits : propre a cette
                                     // tache, ne bloque jamais loop() (Core 1)

    const float c  = s_sensors.getTempCByIndex(0);
    const bool  ok = (c != DEVICE_DISCONNECTED_C) &&
                     c >= cfg::WATER_TEMP_MIN_C && c <= cfg::WATER_TEMP_MAX_C;

    state::setWaterTemp(ok ? static_cast<int>(c * 10.0f) : 0, ok);
    if (!ok) s_present = false;  // capteur debranche/en erreur -> retente la detection

    if (ok) {
      const uint32_t cycleMs = cfg::WATER_TEMP_PERIOD_MS;
      if (cycleMs > 750) vTaskDelay(pdMS_TO_TICKS(cycleMs - 750));
    }
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
