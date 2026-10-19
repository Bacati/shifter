#include "host.h"
#include "mock/Arduino.h"
#include "mock/Wire.h"
#include "mock/DallasTemperature.h"
#include "mock/WiFi.h"
#include "mock/Update.h"
#include "mock/soc/gpio_struct.h"
#include "mock/esp_task_wdt.h"
#include <atomic>
#include <new>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

namespace host {

uint64_t nowUs = 0, baseUs = 0, baseMs = 0;
int restartCount = 0; uint64_t restartAtUs = 0;
std::vector<Edge> cutEdges;
std::string serialOut; bool verbose = false;
Mpu6050 mpu; Ds18b20 ds;
std::map<std::string, std::string> nvs; int nvsWrites = 0; int nvsFailAfter = -1;
std::string wifiMac = "AA:BB:CC:DD:EE:FF"; uint32_t randomSeed = 12345;
int failures = 0;
int wdtResets = 0; bool wdtInit = false; int wdtTimeoutS = 0;

// ---- aleatoire deterministe --------------------------------------------------
static uint64_t rs = 0;
double rnd() {
  if (!rs) rs = 0x9E3779B97F4A7C15ull ^ randomSeed;
  rs ^= rs << 13; rs ^= rs >> 7; rs ^= rs << 17;
  return (double)(rs >> 11) / 9007199254740992.0;
}
double gauss() {
  double u = rnd(), v = rnd(); if (u < 1e-12) u = 1e-12;
  return std::sqrt(-2.0 * std::log(u)) * std::cos(6.283185307179586 * v);
}

// ---- evenements + taches ----------------------------------------------------------
static std::multimap<uint64_t, std::function<void()>> events;

struct Task {
  std::string name; void (*fn)(void*); void* arg; uint64_t wake; bool done; std::thread th;
  Task(const char* n, void (*f)(void*), void* a, uint64_t w) : name(n), fn(f), arg(a), wake(w), done(false) {}
};
static std::vector<Task*> tasks;
static std::mutex g_m; static std::condition_variable g_cv; static Task* g_running = nullptr;
static thread_local Task* t_cur = nullptr;

void at(uint64_t t, std::function<void()> fn) { events.emplace(t, std::move(fn)); }

void every(uint64_t start, uint64_t period, std::function<bool()> fn) {
  auto f = std::make_shared<std::function<bool()>>(std::move(fn));
  auto running = std::make_shared<bool>(false);
  auto nominal = std::make_shared<uint64_t>(start);
  auto tick = std::make_shared<std::function<void()>>();
  *tick = [=]() {
    bool cont = true;
    if (!*running) {
      *running = true;
      try { cont = (*f)(); } catch (...) { *running = false; throw; }
      *running = false;
    }
    if (!cont) return;
    *nominal += period;
    if (*nominal < nowUs) *nominal = nowUs;   // tick(s) manque(s) pendant un blocage
    at(*nominal, *tick);
  };
  at(start, *tick);
}

static void resume(Task* t) {
  std::unique_lock<std::mutex> l(g_m);
  g_running = t; g_cv.notify_all();
  g_cv.wait(l, [] { return g_running == nullptr; });
}

void createTask(void (*fn)(void*), void* arg, const char* name) {
  Task* t = new Task{name, fn, arg, nowUs};
  tasks.push_back(t);
  t->th = std::thread([t] {
    t_cur = t;
    { std::unique_lock<std::mutex> l(g_m); g_cv.wait(l, [&] { return g_running == t; }); }
    try { t->fn(t->arg); } catch (RestartException&) {} catch (...) { fprintf(stderr, "tache %s: exception\n", t->name.c_str()); }
    { std::lock_guard<std::mutex> l(g_m); t->done = true; g_running = nullptr; g_cv.notify_all(); }
  });
  t->th.detach();
}

void runUntil(uint64_t target) {
  for (;;) {
    Task* best = nullptr; uint64_t tw = UINT64_MAX;
    for (Task* t : tasks) if (!t->done && t->wake < tw) { tw = t->wake; best = t; }
    uint64_t te = events.empty() ? UINT64_MAX : events.begin()->first;
    uint64_t t = std::min(tw, te);
    if (t > target) { if (nowUs < target) nowUs = target; return; }
    if (t > nowUs) nowUs = t;
    if (tw <= te) resume(best);
    else { auto fn = events.begin()->second; events.erase(events.begin()); fn(); }
  }
}

void sleepUs(uint64_t us) {
  if (!t_cur) { runUntil(nowUs + us); return; }
  t_cur->wake = nowUs + us;
  std::unique_lock<std::mutex> l(g_m);
  g_running = nullptr; g_cv.notify_all();
  g_cv.wait(l, [] { return g_running == t_cur; });
}
int taskCount() { return (int)tasks.size(); }
int tasksAlive() { int n = 0; for (Task* t : tasks) if (!t->done) ++n; return n; }

// ---- broches --------------------------------------------------------------------
static int lvl[40]; static int pmode[40]; static void (*isr[40])(); static int isrMode[40]; static bool inited = false;
static void initPins() { if (inited) return; inited = true; for (int i = 0; i < 40; ++i) lvl[i] = 1; lvl[4] = 0; }
int pinLevel(int p) { initPins(); return lvl[p]; }
bool isrFire(int p) { initPins(); if (!isr[p]) return false; isr[p](); return true; }
void setPin(int p, int level) {
  initPins();
  const int old = lvl[p]; lvl[p] = level;
  if (old == level || !isr[p]) return;
  const bool rising = level == 1;
  if ((isrMode[p] == RISING && rising) || (isrMode[p] == FALLING && !rising) || isrMode[p] == CHANGE) isr[p]();
}
void pulse(int p, uint64_t t, uint32_t width) {
  initPins();
  const int idle = (isrMode[p] == FALLING) ? 1 : 0;
  at(t > 0 ? t - 1 : 0, [=] { setPin(p, idle); });
  at(t, [=] { setPin(p, !idle); });
  at(t + width, [=] { setPin(p, idle); });
}
void driveOut(int p, int level) {
  initPins();
  if (lvl[p] == level && !(p == 4 && cutEdges.empty() && level)) { lvl[p] = level; return; }
  lvl[p] = level;
  if (p == 4) cutEdges.push_back({nowUs, level != 0});
}
bool cutHigh() { return pinLevel(4) != 0; }

static bool cutAt(uint64_t t) { bool s = false; for (auto& e : cutEdges) { if (e.t <= t) s = e.high; else break; } return s; }
uint64_t cutHighTotalUs(uint64_t from, uint64_t to) {
  uint64_t tot = 0, last = from; bool s = cutAt(from);
  for (auto& e : cutEdges) { if (e.t <= from) continue; if (e.t >= to) break; if (s) tot += e.t - last; last = e.t; s = e.high; }
  if (s) tot += to - last;
  return tot;
}
uint64_t cutMaxRunUs(uint64_t from, uint64_t to) {
  uint64_t best = 0, start = from; bool s = cutAt(from);
  for (auto& e : cutEdges) {
    if (e.t <= from) continue; if (e.t >= to) break;
    if (s && !e.high) best = std::max(best, e.t - start);
    if (!s && e.high) start = e.t;
    s = e.high;
  }
  if (s) best = std::max(best, to - start);
  return best;
}
int cutPulses(uint64_t from, uint64_t to) {
  int n = 0; for (auto& e : cutEdges) if (e.t >= from && e.t <= to && e.high) ++n; return n;
}

// ---- serie ----------------------------------------------------------------------
static std::string serialInBuf;
void serialIn(const std::string& s) { serialInBuf += s; }
bool serialContains(const char* n) { return serialOut.find(n) != std::string::npos; }

// ---- framework ------------------------------------------------------------------
std::vector<TestCase>& registry() { static std::vector<TestCase> r; return r; }
void fail(const char* file, int line, const std::string& msg) {
  ++failures;
  if (failures <= 25) { const char* f = std::strrchr(file, '/'); if (!f) f = std::strrchr(file, '\\'); printf("    FAIL %s:%d %s\n", f ? f + 1 : file, line, msg.c_str()); }
}
int mainEntry(int argc, char** argv) {
  const char* which = nullptr;
  if (const char* sd = std::getenv("HOST_SEED")) randomSeed = (uint32_t)std::strtoul(sd, nullptr, 10);
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--list")) { for (auto& t : registry()) printf("%s\n", t.name); return 0; }
    else if (!std::strcmp(argv[i], "--verbose")) verbose = true;
    else which = argv[i];
  }
  if (!which) { fprintf(stderr, "usage: test [--verbose] <nom> | --list\n"); return 2; }
  for (auto& t : registry()) {
    if (std::strcmp(t.name, which)) continue;
    try { t.fn(); }
    catch (RestartException&) { fail(__FILE__, __LINE__, "redemarrage inattendu (esp_restart)"); }
    catch (PowerLoss&) { fail(__FILE__, __LINE__, "coupure de courant NVS non geree par le test"); }
    printf("%s %s\n", failures ? "[FAIL]" : "[PASS]", t.name);
    fflush(stdout); std::_Exit(failures ? 1 : 0);
  }
  fprintf(stderr, "test inconnu: %s\n", which); return 2;
}

}  // namespace host

// ---- Arduino ------------------------------------------------------------------------
uint32_t millis() { return (uint32_t)(host::nowUs / 1000 + host::baseMs); }
uint32_t micros() { return (uint32_t)(host::nowUs + host::baseUs); }
void delay(uint32_t ms) { host::sleepUs((uint64_t)ms * 1000); }
void delayMicroseconds(uint32_t us) { host::sleepUs(us); }
void pinMode(int p, int m) { host::pinLevel(p); /* init */ (void)p; (void)m; }
void digitalWrite(int p, int l) { host::driveOut(p, l); }
int  digitalRead(int p) { return host::pinLevel(p); }
void attachInterrupt(int p, void (*fn)(), int mode) { host::pinLevel(p); host::isr[p] = fn; host::isrMode[p] = mode; }

int SerialMock::available() { return (int)host::serialInBuf.size(); }
int SerialMock::read() { if (host::serialInBuf.empty()) return -1; int c = (unsigned char)host::serialInBuf[0]; host::serialInBuf.erase(0, 1); return c; }
void SerialMock::write(const std::string& s0) {
  static const bool stamp = std::getenv("HOST_STAMP") != nullptr;
  std::string s = s0;
  if (stamp) { char b[32]; snprintf(b, sizeof b, "@%.3f ", host::nowUs / 1e6); s = b + s0; }
  host::serialOut += s; if (host::verbose) fputs(s.c_str(), stdout);
}
void SerialMock::printf(const char* fmt, ...) {
  char b[2048]; va_list a; va_start(a, fmt); vsnprintf(b, sizeof(b), fmt, a); va_end(a); write(b);
}
SerialMock Serial;
void EspClass::restart() { host::restartCount++; host::restartAtUs = host::nowUs; throw host::RestartException{}; }
EspClass ESP;
gpio_dev_t GPIO;
UpdateClass Update;
WiFiClass WiFi;
TwoWire Wire;

// ---- Wire <-> MPU6050 -------------------------------------------------------------------
bool TwoWire::begin(int, int, uint32_t) { return true; }
uint8_t TwoWire::endTransmission(bool) {
  auto& m = host::mpu;
  if (addr_ != 0x68 || !m.present || m.busStuck) { m.nacks++; return 2; }
  if (txn_ >= 1) m.reg = tx_[0];
  if (txn_ >= 2 && m.reg == 0x6B) {                 // PWR_MGMT_1
    if (tx_[1] & 0x80) { m.sleepBit = true; m.resets++; }
    else m.sleepBit = (tx_[1] & 0x40) != 0;
  }
  return 0;
}
size_t TwoWire::requestFrom(int addr, int n) {
  auto& m = host::mpu;
  rxn_ = rxi_ = 0;
  if (addr != 0x68 || !m.present || m.busStuck || n > (int)sizeof(rx_)) { m.nacks++; return 0; }
  if (m.reg == 0x75) { rx_[0] = m.who; rxn_ = 1; return rxn_ >= (size_t)n ? (size_t)n : 0; }
  if (m.reg != 0x3B || n != 14) { std::memset(rx_, 0, n); rxn_ = n; return n; }

  static uint64_t cachedTs = ~0ull; static uint8_t cached[14];
  const uint64_t ts = (host::nowUs / 5000) * 5000;           // MPU echantillonne a 200 Hz
  m.reads++;
  if (m.frozen && m.haveFrame) { std::memcpy(rx_, m.lastFrame, 14); }
  else if (m.sleepBit) { std::memset(rx_, 0, 14); }
  else {
    if (ts != cachedTs) {
      float a[3] = {0, 0, 1}, g[3] = {0, 0, 0};
      if (m.motion) m.motion(ts, a, g);
      for (int i = 0; i < 3; ++i) {
        a[i] += (float)host::gauss() * m.accNoiseG;
        g[i] += m.gyroBiasDps[i] + (float)host::gauss() * m.gyroNoiseDps;
      }
      auto q = [](float v, float lsb) { float x = v * lsb; if (x > 32767) x = 32767; if (x < -32768) x = -32768; return (int16_t)std::lround(x); };
      int16_t w[7] = {q(a[0], 16384), q(a[1], 16384), q(a[2], 16384), 0, q(g[0], 131), q(g[1], 131), q(g[2], 131)};
      for (int i = 0; i < 7; ++i) { cached[2 * i] = (uint8_t)((uint16_t)w[i] >> 8); cached[2 * i + 1] = (uint8_t)(w[i] & 0xFF); }
      cachedTs = ts;
    }
    std::memcpy(rx_, cached, 14);
    std::memcpy(m.lastFrame, cached, 14); m.haveFrame = true;
  }
  rxn_ = 14; return 14;
}

// ---- DS18B20 ----------------------------------------------------------------------------
void DallasTemperature::requestTemperatures() {
  auto& d = host::ds;
  d.requests++;
  if (!d.present) return;
  if (d.waitConv) {                               // comportement par defaut de la bibliotheque
    host::sleepUs(750000); d.reportBlockingMs += 750;
    d.scratch = d.temp ? d.temp(host::nowUs) : 20.0f; d.converting = false;
  } else { d.convStart = host::nowUs; d.converting = true; }
}
float DallasTemperature::getTempC(const uint8_t*, byte) {
  auto& d = host::ds;
  d.reads++;
  if (!d.present) return DEVICE_DISCONNECTED_C;
  if (d.failReads > 0) { d.failReads--; return DEVICE_DISCONNECTED_C; }
  if (d.converting && host::nowUs - d.convStart >= 750000) {
    d.scratch = d.temp ? d.temp(d.convStart) : 20.0f; d.converting = false;
  }
  return d.scratch;                                // 85.0 tant qu'aucune conversion n'a abouti
}

int main(int argc, char** argv) { return host::mainEntry(argc, argv); }

// ---- suivi du tas ------------------------------------------------------------------------------
static std::atomic<long> g_allocs{0}, g_bytes{0};
long host::liveAllocs() { return g_allocs.load(); }
long host::liveBytes() { return g_bytes.load(); }
namespace {
struct Hdr { size_t n; size_t pad; };      // 16 octets : garde l'alignement
inline void* tracked_alloc(size_t n) {
  Hdr* h = static_cast<Hdr*>(std::malloc(n + sizeof(Hdr)));
  if (!h) return nullptr;
  h->n = n; g_allocs++; g_bytes += (long)n;
  return h + 1;
}
inline void tracked_free(void* p) {
  if (!p) return;
  Hdr* h = static_cast<Hdr*>(p) - 1;
  g_allocs--; g_bytes -= (long)h->n;
  std::free(h);
}
}  // namespace
void* operator new(size_t n) { void* p = tracked_alloc(n); if (!p) throw std::bad_alloc(); return p; }
void* operator new[](size_t n) { void* p = tracked_alloc(n); if (!p) throw std::bad_alloc(); return p; }
void* operator new(size_t n, const std::nothrow_t&) noexcept { return tracked_alloc(n); }
void* operator new[](size_t n, const std::nothrow_t&) noexcept { return tracked_alloc(n); }
void operator delete(void* p) noexcept { tracked_free(p); }
void operator delete[](void* p) noexcept { tracked_free(p); }
void operator delete(void* p, size_t) noexcept { tracked_free(p); }
void operator delete[](void* p, size_t) noexcept { tracked_free(p); }

