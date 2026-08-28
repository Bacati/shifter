/*
 * ============================================================================
 *  PROJET : Aptatio ECU 50cc 2T (PVL) - Shifter / Launch / Slip / Pit limiter
 *  VERSION: 23.0  (refonte modulaire + durcissement WiFi/licence + OTA)
 * ============================================================================
 *
 *  Point d'entree uniquement : setup() cable les modules, loop() est le
 *  "sanctuaire" temps-reel du Core 1.
 *
 *  Arborescence des modules (dossier src/) :
 *    src/core/       Config, Version, Types, SharedState (frontiere Core0<->Core1)
 *    src/sensors/    ISR, filtre RPM/EMI, vitesses, glisse, entrees debouncees
 *    src/actuator/   IgnitionCut : broche de coupure + priorite stricte
 *    src/control/    Arbiter : arbitre central des assistances
 *    src/persistence/ Storage (NVS + mutex), License (DRM HMAC), secrets.h
 *    src/net/        Network (AP WPA2), WebServer/WebPage (HTTP/SSE/OTA/auth),
 *                    Telemetry (taches Core 0 : SSE + superviseur fail-safe)
 *
 *  Details cablage / build / securite : voir BUILD.md.
 * ============================================================================
 */

#include <Arduino.h>
#include <esp_task_wdt.h>

#include "src/core/Version.h"
#include "src/core/Config.h"
#include "src/core/SharedState.h"
#include "src/persistence/Storage.h"
#include "src/actuator/IgnitionCut.h"
#include "src/sensors/Sensors.h"
#include "src/sensors/Imu.h"
#include "src/persistence/License.h"
#include "src/net/Network.h"
#include "src/net/WebServer.h"
#include "src/net/Telemetry.h"
#include "src/control/Arbiter.h"
#include "src/sim/SimEngine.h"
#include "src/factory/Factory.h"

void setup() {
#ifdef WOKWI_SIM
  // Diagnostic : 6 clignotements de la LED CUT des le boot (avant tout le reste).
  pinMode(pins::CUT, OUTPUT);
  for (int i = 0; i < 6; ++i) {
    digitalWrite(pins::CUT, HIGH); delay(120);
    digitalWrite(pins::CUT, LOW);  delay(120);
  }
#endif

  Serial.begin(115200);
  delay(50);
  Serial.printf("\n[BOOT] Aptatio ECU v%s (%s)\n", FW_VERSION, FW_BUILD);

  // Watchdog materiel : configure ici, mais on n'arme la tache loop()
  // qu'a la FIN de setup() (l'init WiFi peut etre lente et ne doit pas
  // declencher un reboot premature).
  esp_task_wdt_init(cfg::WDT_TIMEOUT_S, true);

  // La broche de coupure est mise en securite au plus tot.
  ignition::begin();

  state::begin();
  storage::begin();
  state::setConfig(storage::loadConfig());

  sensors::begin();
  sim::begin();       // no-op sauf si -DWOKWI_SIM

#ifndef WOKWI_SIM
  imu::begin();       // MPU6050 : angle de roulis (garde-fou anti-angle du slip control)
  imu::startTask();

  net::begin();       // WiFi AP AVANT license (WiFi.macAddress() requis)
  Serial.printf("[BOOT] HWID: %s\n", net::hwid().c_str());  // MAC lisible pour le QC / provisioning
  license::begin();
  web::begin();
#else
  // Simulation Wokwi : pas de WiFi (trop lent a emuler) -> test logique rapide.
  license::begin();   // force le tier, ne touche pas au WiFi
  Serial.println("[SIM] WiFi / serveur web desactives (build -DWOKWI_SIM)");
#endif

  factory::begin();        // no-op sauf si -DFACTORY_TOOLS

  state::beatEngineLoop();  // heartbeat frais AVANT de lancer le superviseur
  telemetry::startTasks();

  esp_task_wdt_add(NULL);   // arme le watchdog de loop() maintenant que tout est pret
  Serial.println("[BOOT] pret.");
}

void loop() {
  esp_task_wdt_reset();
  net::serviceReboot();      // execute un reboot differe demande par le web
  factory::poll();           // no-op sauf si -DFACTORY_TOOLS

  sensors::update();         // RPM / vitesses / glisse / entrees
  sim::update();             // no-op sauf si -DWOKWI_SIM (ecrase rpm/vitesses simules)
  ignition::service();       // coupe l'allumage a l'echeance
  arbiter::update();         // decide de la prochaine coupure
  ignition::service();       // re-check immediat pour les coupures tres courtes

  // Heartbeat en FIN d'iteration : represente "un cycle complet a abouti".
  // Un blocage n'importe ou dans loop() -> plus de heartbeat -> le superviseur agit.
  state::beatEngineLoop();
}
