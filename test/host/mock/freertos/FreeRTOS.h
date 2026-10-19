#pragma once
#include <cstdint>
typedef int BaseType_t;
typedef unsigned int UBaseType_t;
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define portTICK_PERIOD_MS 1
#define portMAX_DELAY 0xFFFFFFFFu
#define pdTRUE 1
#define pdFALSE 0

// Un seul thread logique s'execute a la fois (ordonnanceur cooperatif) : les sections
// critiques n'ont rien a proteger sur PC. (Les courses ISR/tache ne sont donc PAS testees.)
struct portMUX_TYPE { int x; };
#define portMUX_INITIALIZER_UNLOCKED {0}
#define portENTER_CRITICAL(m) ((void)(m))
#define portEXIT_CRITICAL(m) ((void)(m))
#define portENTER_CRITICAL_ISR(m) ((void)(m))
#define portEXIT_CRITICAL_ISR(m) ((void)(m))
