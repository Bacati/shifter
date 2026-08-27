#include "SimEngine.h"
#include "../core/SharedState.h"
#include "../core/Config.h"
#include "../persistence/License.h"

#ifdef WOKWI_SIM
// ---------------------------------------------------------------------------
//  Version SIMULATION
// ---------------------------------------------------------------------------

namespace {
uint32_t s_t0        = 0;
int      s_timeScale = (IGNITION_SIM_SCALE < 1) ? 1 : IGNITION_SIM_SCALE;  // jamais 0
bool     s_frozen    = false;      // true : faux moteur fige (LED = boutons seuls)
int      s_fixedRpm  = 9000;       // regime tenu quand fige
int      s_fixedSlip = 0;          // % de glisse injecte quand fige (test SLIP CTL)
uint32_t s_hangUntil = 0;          // blocage volontaire de loop() (test superviseur)
int      s_leanX10   = 0;          // angle de roulis simule x10 (garde-fou anti-angle)
int      s_latGx100  = 100;        // |accel|/g x100 simule
bool     s_imuOk     = true;       // IMU simule present/fiable
String   s_line;

void printHelp() {
  Serial.println("[SIM] commandes serie :");
  Serial.println("  cut <ms>    duree coupure quickshifter (10..2000)");
  Serial.println("  scale <n>   echelle temps de TOUTES les coupures (1 = temps reel)");
  Serial.println("  freeze      fige le faux moteur : la LED ne repond plus qu'aux boutons");
  Serial.println("  run         relance le cycle automatique (SLIP + OVERREV auto)");
  Serial.println("  rpm <n>     regime tenu quand fige (0..20000)");
  Serial.println("  slip <pct>  glisse injectee quand fige (0..50) -> test SLIP CTL");
  Serial.println("  pit <rpm>   seuil du pit limiter");
  Serial.println("  launch <rpm> regime cible du launch control");
  Serial.println("  slipmax <pct> tolerance de glisse avant coupure");
  Serial.println("  lean <deg>  angle de roulis simule (garde-fou : slip coupe > 28 deg)");
  Serial.println("  latg <x100> |accel|/g x100 simule (>130 -> blocage dur du slip)");
  Serial.println("  imu <0|1>   1 = IMU present ; 0 = IMU muet (fail-safe : slip desactive)");
  Serial.println("  hang <s>    bloque loop() <s> secondes -> le superviseur doit rebooter la carte");
  Serial.println("  keys        affiche les 3 cles de licence attendues pour cette carte");
  Serial.println("  key <cle>   teste un deblocage de licence (change le tier a chaud)");
  Serial.println("  show        affiche la config, le tier et l'etat");
  Serial.println("  help        cette aide");
}

void printShow() {
  const EcuConfig   c = state::getConfig();
  const LicenseTier t = license::tier();
  Serial.printf("[SIM] tier=%d (%s)  |  cut=%d ms  scale=x%d  ->  LED shifter ~%d ms\n",
                static_cast<int>(t), toString(t), c.shiftCutMs, s_timeScale,
                c.shiftCutMs * s_timeScale);
  Serial.printf("[SIM] moteur=%s  rpm_fige=%d  slip_injecte=%d%%  |  pitRpm=%d  launchRpm=%d  slipMax=%d%%\n",
                s_frozen ? "FIGE" : "cycle auto", s_fixedRpm, s_fixedSlip,
                c.pitRpm, c.launchRpm, c.maxSlipPercent);
  Serial.printf("[SIM] roulis=%.1f deg  latG=%.2f  IMU=%s\n",
                s_leanX10 / 10.0f, s_latGx100 / 100.0f, s_imuOk ? "OK" : "MUET");
}

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  const int    sp  = line.indexOf(' ');
  const String cmd = (sp < 0) ? line : line.substring(0, sp);
  const String arg = (sp < 0) ? String("") : line.substring(sp + 1);
  const long   val = arg.toInt();

  EcuConfig c = state::getConfig();

  if      (cmd == "cut")    { c.shiftCutMs = constrain((int)val, 10, 2000); state::setConfig(c); printShow(); }
  else if (cmd == "scale")  { s_timeScale  = constrain((int)val, 1, 500);   printShow(); }
  else if (cmd == "freeze") { s_frozen = true;  printShow(); }
  else if (cmd == "run")    { s_frozen = false; printShow(); }
  else if (cmd == "rpm")    { s_fixedRpm = constrain((int)val, 0, 20000);   printShow(); }
  else if (cmd == "slip")   { s_fixedSlip = constrain((int)val, 0, 50);     printShow(); }
  else if (cmd == "pit")     { c.pitRpm         = constrain((int)val, 2000, 12000); state::setConfig(c); printShow(); }
  else if (cmd == "launch")  { c.launchRpm      = constrain((int)val, 2000, 15000); state::setConfig(c); printShow(); }
  else if (cmd == "slipmax") { c.maxSlipPercent = constrain((int)val, 0, 50);       state::setConfig(c); printShow(); }
  else if (cmd == "lean")   { s_leanX10 = constrain((int)val, -90, 90) * 10;        printShow(); }
  else if (cmd == "latg")   { s_latGx100 = constrain((int)val, 0, 400);             printShow(); }
  else if (cmd == "imu")    { s_imuOk = (val != 0);                                 printShow(); }
  else if (cmd == "hang")   {
    const int s = constrain((int)val, 1, 10);
    Serial.printf("[SIM] blocage de loop() pendant %d s : le superviseur doit rebooter (6 blinks)\n", s);
    s_hangUntil = millis() + static_cast<uint32_t>(s) * 1000UL;
  }
  else if (cmd == "keys")   {
    for (int i = 0; i < 3; ++i)
      Serial.printf("[SIM] cle tier %d (%s) = %s\n", i, toString((LicenseTier)i),
                    license::expectedKey((LicenseTier)i).c_str());
  }
  else if (cmd == "key")    {
    uint32_t retry = 0;
    if (license::tryUnlock(arg, retry))
      Serial.printf("[SIM] OK -> tier %d (%s)\n", (int)license::tier(), toString(license::tier()));
    else if (retry)
      Serial.printf("[SIM] VERROUILLE, reessayer dans %lu ms\n", (unsigned long)retry);
    else
      Serial.println("[SIM] cle REFUSEE");
  }
  else if (cmd == "show")   { printShow(); }
  else                      { printHelp(); }
}

void pollSerial() {
  while (Serial.available()) {
    const char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { handleLine(s_line); s_line = ""; }
    else if (s_line.length() < 60) s_line += ch;
  }
}
}  // namespace

void sim::begin() {
  s_t0 = millis();

#if SIM_SHIFT_CUT_MS > 0
  {
    EcuConfig c = state::getConfig();
    c.shiftCutMs = constrain((int)SIM_SHIFT_CUT_MS, 10, 2000);
    state::setConfig(c);
  }
#endif
#if SIM_FREEZE
  s_frozen = true;
#endif

  const EcuConfig c = state::getConfig();
  Serial.println("=================================================");
  Serial.println("  MODE SIMULATION WOKWI (-DWOKWI_SIM)");
  Serial.printf ("  Echelle temps coupures : x%d\n", s_timeScale);
  Serial.printf ("  Cut shifter            : %d ms  (LED ~%d ms)\n",
                 c.shiftCutMs, c.shiftCutMs * s_timeScale);
  Serial.printf ("  Faux moteur            : %s\n", s_frozen ? "FIGE (boutons seuls)" : "cycle auto");
  Serial.printf ("  Tier force au boot     : %d (SIM_TIER)\n", sim::forcedTier());
  Serial.println("  Console serie (si dispo) : 'help' + Entree  ('keys' / 'key <cle>' pour tester les licences)");
  Serial.println("=================================================");
}

bool        sim::active()             { return true; }
String      sim::apPasswordOverride() { return "wokwi1234"; }
const char* sim::pinOverride()        { return "123456"; }
int         sim::forcedTier()         { return constrain((int)SIM_TIER, 0, 2); }
int         sim::timeScale()          { return s_timeScale; }

void sim::update() {
  pollSerial();

  // Blocage volontaire : loop() ne bat plus -> le superviseur (Core 0) coupe
  // l'allumage et redemarre. Sert a verifier le fail-safe.
  if (s_hangUntil) {
    const uint32_t target = s_hangUntil;
    s_hangUntil = 0;
    while (static_cast<int32_t>(millis() - target) < 0) delay(100);
  }

  int   rpm    = s_fixedRpm;
  float vFront = 0.0f, vRear = 0.0f, slip = 0.0f;

  if (!s_frozen) {
    // Scenario cyclique ~20 s : 5 phases de 4 s.
    switch (((millis() - s_t0) / 4000UL) % 5UL) {
      case 0: rpm = 8000;                                      break; // zone LAUNCH (EMBRAYAGE)
      case 1: rpm = 7000;  vFront = 30; vRear = 31;            break; // roulage stable
      case 2: rpm = 9000;  vFront = 40; vRear = 49; slip = 22; break; // patinage -> SLIP (auto)
      case 3: rpm = 12000; vFront = 62; vRear = 63;            break; // haut regime (SHIFTER)
      case 4: rpm = 17000; vFront = 65; vRear = 66;            break; // -> OVERREV (auto)
    }
  } else if (s_fixedSlip > 0) {
    // Fige + glisse injectee : de quoi tester SLIP CTL a la demande.
    vFront = 40.0f;
    vRear  = 40.0f * (1.0f + s_fixedSlip / 100.0f);
    slip   = static_cast<float>(s_fixedSlip);
  }

  state::setRpmInstant(rpm);
  state::setRpmFiltered(rpm);
  state::setSpeeds(static_cast<int>(vFront * 10.0f), static_cast<int>(vRear * 10.0f));
  state::setSlip(static_cast<int>(slip * 10.0f), 0);

  // IMU simule (pas de MPU6050 en simu) : entretient l'etat "frais".
  state::setLean(s_leanX10, s_latGx100, s_imuOk);
}

#else
// ---------------------------------------------------------------------------
//  Version NORMALE : tout est neutre.
// ---------------------------------------------------------------------------

void        sim::begin()              {}
void        sim::update()             {}
bool        sim::active()             { return false; }
String      sim::apPasswordOverride() { return String(); }
const char* sim::pinOverride()        { return nullptr; }
int         sim::forcedTier()         { return -1; }
int         sim::timeScale()          { return 1; }

#endif
