#include "Factory.h"

#ifdef FACTORY_TOOLS
// ---------------------------------------------------------------------------
//  Version PROVISIONING
// ---------------------------------------------------------------------------
#include <Arduino.h>
#include <WiFi.h>
#include "../core/Version.h"
#include "../core/Types.h"
#include "../persistence/License.h"
#include "../persistence/Storage.h"

namespace {
String s_line;

void printInfo() {
  Serial.printf("INFO fw=%s mac=%s tier=%d(%s)\n",
                FW_VERSION, WiFi.macAddress().c_str(),
                static_cast<int>(license::tier()), toString(license::tier()));
}

void handle(String line) {
  line.trim();
  if (line.length() == 0) return;

  const int    sp  = line.indexOf(' ');
  String       cmd = (sp < 0) ? line : line.substring(0, sp);
  const String arg = (sp < 0) ? ""   : line.substring(sp + 1);
  cmd.toUpperCase();

  if (cmd == "MAC?" || cmd == "MAC") {
    Serial.printf("MAC %s\n", WiFi.macAddress().c_str());
  } else if (cmd == "INFO" || cmd == "INFO?") {
    printInfo();
  } else if (cmd == "KEY") {
    uint32_t retry = 0;
    if (license::tryUnlock(arg, retry))
      Serial.printf("OK tier=%d\n", static_cast<int>(license::tier()));
    else if (retry)
      Serial.printf("LOCKED retry_ms=%lu\n", static_cast<unsigned long>(retry));
    else
      Serial.println("FAIL bad_key");
  } else if (cmd == "WIPE") {
    storage::saveLicense("", LicenseTier::Basic);
    Serial.println("OK wiped");
    Serial.flush();
    delay(50);
    ESP.restart();
  } else {
    Serial.println("ERR commands: MAC? | INFO | KEY <key> | WIPE");
  }
}
}  // namespace

void factory::begin() {
  Serial.println();
  Serial.println("### FACTORY MODE (build -DFACTORY_TOOLS) ###");
  printInfo();
  Serial.println("### commandes: MAC? | INFO | KEY <key> | WIPE ###");
}

void factory::poll() {
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '\n' || c == '\r') { handle(s_line); s_line = ""; }
    else if (s_line.length() < 80) s_line += c;
  }
}

#else
// ---------------------------------------------------------------------------
//  Version NORMALE : rien.
// ---------------------------------------------------------------------------
void factory::begin() {}
void factory::poll() {}
#endif
