#pragma once
// ============================================================================
//  Banc de test PC : horloge virtuelle, ordonnanceur cooperatif de taches FreeRTOS,
//  broches, composants simules (MPU6050, DS18B20, NVS), mini framework de test.
//  Le VRAI code de src/ est compile contre ces faux (voir run_tests.py).
// ============================================================================
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace host {

// ---- horloge virtuelle (microsecondes) -------------------------------------
extern uint64_t nowUs;
extern uint64_t baseUs;   // decalage de micros()  (tester le debordement 32 bits)
extern uint64_t baseMs;   // decalage de millis()

struct RestartException {};          // lancee par esp_restart() / ESP.restart()
extern int      restartCount;
extern uint64_t restartAtUs;

// ---- evenements + taches ----------------------------------------------------
void at(uint64_t tUs, std::function<void()> fn);
// periodique ; fn retourne false pour s'arreter. Non reentrant (un tick manque si fn bloque).
void every(uint64_t startUs, uint64_t periodUs, std::function<bool()> fn);
void runUntil(uint64_t tUs);
inline void runFor(uint64_t us) { runUntil(nowUs + us); }
void createTask(void (*fn)(void*), void* arg, const char* name);
void sleepUs(uint64_t us);           // delay()/vTaskDelay() : cede la main
int  taskCount();
int  tasksAlive();

// ---- broches --------------------------------------------------------------
int  pinLevel(int pin);
void setPin(int pin, int level);                 // pilote externe ; declenche les ISR
void pulse(int pin, uint64_t tEdge, uint32_t widthUs);  // front ISR (selon attachInterrupt) a tEdge, retour au repos a tEdge+width
void driveOut(int pin, int level);               // sortie ECU (digitalWrite / registre GPIO)
bool isrFire(int pin);                           // appelle l'ISR sans changer le niveau

struct Edge { uint64_t t; bool high; };
extern std::vector<Edge> cutEdges;               // transitions de la broche CUT (D4)
bool     cutHigh();
uint64_t cutHighTotalUs(uint64_t from, uint64_t to);
uint64_t cutMaxRunUs(uint64_t from, uint64_t to);
int      cutPulses(uint64_t from, uint64_t to);

// ---- serie ----------------------------------------------------------------
extern std::string serialOut;
void serialIn(const std::string& s);
bool serialContains(const char* needle);
extern bool verbose;

// ---- composants simules ---------------------------------------------------------
struct Mpu6050 {
  bool present = true;
  bool busStuck = false;      // SDA bloque : toute transaction echoue
  bool frozen = false;        // renvoie toujours la meme trame
  uint8_t who = 0x68;
  float accNoiseG = 0.01f, gyroNoiseDps = 0.25f;
  float gyroBiasDps[3] = {1.5f, -0.8f, 0.4f};
  // (t_us) -> accel corps (g) et gyro corps (deg/s), AVANT bruit/biais
  std::function<void(uint64_t, float*, float*)> motion;
  // interne
  // sleepBit=true : registres a zero (etat apres brownout / au reset du composant)
  uint8_t reg = 0; bool sleepBit = true; uint8_t lastFrame[14] = {0}; bool haveFrame = false;
  uint32_t reads = 0, resets = 0, nacks = 0;
};
extern Mpu6050 mpu;

struct Ds18b20 {
  bool present = true;
  bool parasiteFlaky = false; // renvoie 85.0 si lu trop tot / apres reset
  std::function<float(uint64_t)> temp;     // vraie temperature
  int failReads = 0;                       // prochaines lectures -> -127 (CRC/deconnexion)
  int reportBlockingMs = 0;                // cumul des attentes bloquantes
  // interne
  float scratch = 85.0f; uint64_t convStart = 0; bool converting = false; bool waitConv = true;
  uint32_t requests = 0, reads = 0;
};
extern Ds18b20 ds;

extern std::map<std::string, std::string> nvs;     // survit aux "reboots" du test
extern int  nvsWrites;
extern int  nvsFailAfter;                          // >=0 : coupure de courant apres N ecritures
struct PowerLoss {};

extern std::string wifiMac;
extern uint32_t    randomSeed;

// ---- mini framework -----------------------------------------------------------
struct TestCase { const char* name; void (*fn)(); };
std::vector<TestCase>& registry();
struct Registrar { Registrar(const char* n, void (*f)()) { registry().push_back({n, f}); } };
extern int failures;
void fail(const char* file, int line, const std::string& msg);
int  mainEntry(int argc, char** argv);
double rnd();                 // [0,1) deterministe
// Suivi du tas (operator new/delete remplaces) : detection de fuites
long liveAllocs();
long liveBytes();
double gauss();

}  // namespace host

#define TEST(name) \
  static void test_##name(); \
  static host::Registrar reg_##name(#name, test_##name); \
  static void test_##name()

#define CHECK(cond) do { if (!(cond)) host::fail(__FILE__, __LINE__, "CHECK(" #cond ")"); } while (0)
#define CHECK_MSG(cond, ...) do { if (!(cond)) { char _b[512]; snprintf(_b, sizeof(_b), __VA_ARGS__); \
  host::fail(__FILE__, __LINE__, std::string("CHECK(" #cond ") ") + _b); } } while (0)
#define CHECK_NEAR(a, b, tol) do { double _a = (a), _b2 = (b); if (!(std::fabs(_a - _b2) <= (tol))) { \
  char _m[256]; snprintf(_m, sizeof(_m), #a " = %g, attendu %g (+/- %g)", _a, _b2, (double)(tol)); \
  host::fail(__FILE__, __LINE__, _m); } } while (0)
#define INFO(...) do { if (host::verbose) { printf("    "); printf(__VA_ARGS__); printf("\n"); } } while (0)
#define SEC(x) ((uint64_t)((x) * 1000000.0))
#define MS(x)  ((uint64_t)((x) * 1000.0))
