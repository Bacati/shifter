#pragma once
#include "FreeRTOS.h"
typedef int* SemaphoreHandle_t;
inline SemaphoreHandle_t xSemaphoreCreateMutex() { static int x; return &x; }
inline BaseType_t xSemaphoreTake(SemaphoreHandle_t, TickType_t) { return pdTRUE; }
inline BaseType_t xSemaphoreGive(SemaphoreHandle_t) { return pdTRUE; }
