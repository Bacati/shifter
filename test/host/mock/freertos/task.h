#pragma once
#include "FreeRTOS.h"
#include "../../host.h"
typedef void* TaskHandle_t;
typedef void (*TaskFunction_t)(void*);

inline BaseType_t xTaskCreatePinnedToCore(TaskFunction_t fn, const char* name, uint32_t /*stack*/,
                                          void* arg, UBaseType_t /*prio*/, TaskHandle_t* h, BaseType_t /*core*/) {
  host::createTask(fn, arg, name);
  if (h) *h = nullptr;
  return pdTRUE;
}
inline void vTaskDelay(TickType_t ticks) { host::sleepUs((uint64_t)ticks * 1000); }
