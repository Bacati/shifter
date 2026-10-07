#include "SharedState.h"
#include "Config.h"
#include <atomic>

namespace {
portMUX_TYPE s_cfgMux = portMUX_INITIALIZER_UNLOCKED;
EcuConfig    s_cfg     = cfg::DEFAULTS;

std::atomic<int>      s_rpmFiltered{0};
std::atomic<int>      s_rpmInstant{0};
std::atomic<int>      s_frontX10{0};
std::atomic<int>      s_rearX10{0};
std::atomic<int>      s_slipX10{0};
std::atomic<int>      s_slipRateX10{0};
std::atomic<uint8_t>  s_mode{static_cast<uint8_t>(AssistMode::Idle)};
std::atomic<uint32_t> s_engineBeat{0};

std::atomic<int>      s_rollX10{0};
std::atomic<int>      s_latGx100{100};
std::atomic<int>      s_imuOk{0};
std::atomic<uint32_t> s_leanMs{0};

std::atomic<int>      s_waterTempX10{0};
std::atomic<int>      s_waterTempOk{0};
std::atomic<uint32_t> s_waterTempMs{0};
}  // namespace

void state::begin() {
  s_engineBeat.store(millis());
}

EcuConfig state::getConfig() {
  portENTER_CRITICAL(&s_cfgMux);
  EcuConfig c = s_cfg;
  portEXIT_CRITICAL(&s_cfgMux);
  return c;
}

void state::setConfig(const EcuConfig& c) {
  portENTER_CRITICAL(&s_cfgMux);
  s_cfg = c;
  portEXIT_CRITICAL(&s_cfgMux);
}

void state::setRpmFiltered(int v) { s_rpmFiltered.store(v); }
int  state::getRpmFiltered()      { return s_rpmFiltered.load(); }
void state::setRpmInstant(int v)  { s_rpmInstant.store(v); }
int  state::getRpmInstant()       { return s_rpmInstant.load(); }

void state::setSpeeds(int f, int r) { s_frontX10.store(f); s_rearX10.store(r); }
void state::getSpeeds(int& f, int& r) { f = s_frontX10.load(); r = s_rearX10.load(); }

void state::setSlip(int s, int sr) { s_slipX10.store(s); s_slipRateX10.store(sr); }
void state::getSlip(int& s, int& sr) { s = s_slipX10.load(); sr = s_slipRateX10.load(); }

void       state::setMode(AssistMode m) { s_mode.store(static_cast<uint8_t>(m)); }
AssistMode state::getMode()             { return static_cast<AssistMode>(s_mode.load()); }

void state::setLean(int rollX10, int latGx100, bool ok) {
  s_rollX10.store(rollX10);
  s_latGx100.store(latGx100);
  s_imuOk.store(ok ? 1 : 0);
  s_leanMs.store(millis());
}
void state::getLean(int& rollX10, int& latGx100, bool& ok) {
  rollX10  = s_rollX10.load();
  latGx100 = s_latGx100.load();
  ok       = s_imuOk.load() != 0;
}
uint32_t state::lastLeanMs() { return s_leanMs.load(); }

void state::setWaterTemp(int tempX10, bool ok) {
  s_waterTempX10.store(tempX10);
  s_waterTempOk.store(ok ? 1 : 0);
  s_waterTempMs.store(millis());
}
void state::getWaterTemp(int& tempX10, bool& ok) {
  tempX10 = s_waterTempX10.load();
  ok      = s_waterTempOk.load() != 0;
}
uint32_t state::lastWaterTempMs() { return s_waterTempMs.load(); }

void     state::beatEngineLoop()  { s_engineBeat.store(millis()); }
uint32_t state::lastEngineBeatMs() { return s_engineBeat.load(); }
