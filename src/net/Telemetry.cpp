#include "Telemetry.h"
#include "../core/Config.h"
#include "../core/Types.h"
#include "../core/SharedState.h"
#include "../persistence/License.h"
#include "../actuator/IgnitionCut.h"
#include "WebServer.h"
#include <Arduino.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace {

#ifndef WOKWI_SIM
void sseTask(void*) {
  esp_task_wdt_add(NULL);
  char buf[256];

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(cfg::SSE_PERIOD_MS));

    if (web::sseClients() > 0) {
      int frontX10, rearX10, slipX10, slipRateX10, rollX10, latGx100;
      bool imuOk;
      state::getSpeeds(frontX10, rearX10);
      state::getSlip(slipX10, slipRateX10);
      state::getLean(rollX10, latGx100, imuOk);

      snprintf(buf, sizeof(buf),
        "{\"rpm\":%d,\"speed\":%.1f,\"slip\":%.1f,\"mode\":\"%s\",\"tier\":\"%s\","
        "\"lean\":%.1f,\"imu\":%s}",
        state::getRpmFiltered(),
        frontX10 / 10.0f,
        slipX10 / 10.0f,
        toString(state::getMode()),
        toString(license::tier()),
        rollX10 / 10.0f,
        imuOk ? "true" : "false");

      web::sseSend(buf, "telemetry");
    }
    esp_task_wdt_reset();
  }
}
#endif  // !WOKWI_SIM

void supervisorTask(void*) {
  esp_task_wdt_add(NULL);
  const uint32_t startedAt = millis();

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(cfg::SUPERVISOR_PERIOD_MS));
    esp_task_wdt_reset();

    // Grace apres le boot : laisse loop() demarrer et battre au moins une fois
    // (l'init WiFi de setup() peut durer plusieurs secondes, surtout en simu).
    if (millis() - startedAt < 3000) continue;

    const uint32_t sinceBeat = millis() - state::lastEngineBeatMs();
    if (sinceBeat > cfg::LOOP_STALL_MS) {
      ignition::forceClear();  // fail-safe : rend l'etincelle au pilote
      Serial.printf("[SUPERVISOR] loop() fige depuis %u ms -> reboot\n",
                    static_cast<unsigned>(sinceBeat));
      Serial.flush();
      esp_restart();
    }
  }
}

}  // namespace

void telemetry::startTasks() {
#ifndef WOKWI_SIM
  xTaskCreatePinnedToCore(sseTask, "sse", cfg::SSE_TASK_STACK, nullptr, 2, nullptr, cfg::CORE_NET);
#endif
  xTaskCreatePinnedToCore(supervisorTask, "supervisor", cfg::SUPERVISOR_STACK, nullptr, 1, nullptr,
                          cfg::CORE_NET);
}
