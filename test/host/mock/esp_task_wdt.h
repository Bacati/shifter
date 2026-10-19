#pragma once
#include <cstdint>
namespace host { extern int wdtResets; extern bool wdtInit; extern int wdtTimeoutS; }
inline int esp_task_wdt_init(int s, bool) { host::wdtInit = true; host::wdtTimeoutS = s; return 0; }
inline int esp_task_wdt_add(void*) { return 0; }
inline int esp_task_wdt_reset() { host::wdtResets++; return 0; }
