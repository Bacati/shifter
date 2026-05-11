/*
 * ============================================================================
 * PROJET : Module Racing 50cc 2T (PVL) - Shifter, Launch, Slip & Pit Limiter
 * VERSION: 22.1 (Endgame Release - Watchdog IMU, Zero-Misfire Hysteresis)
 * ============================================================================
 */

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <atomic>
#include <esp_task_wdt.h> 
#include <mbedtls/md.h> 
#include <math.h> 

#define WDT_TIMEOUT_SECONDS 3 

// /!\ AVERTISSEMENT SÉCURITÉ (DRM) /!\
// En production, le Flash Encryption et le Secure Boot V2 de l'ESP32 DOIVENT 
// être activés via 'idf.py menuconfig' pour empêcher l'extraction de cette clé 
// par dump matériel (lecture directe de la mémoire Flash SPI).
const String COMPANY_SECRET_SALT = "AptatioRacing_TopSecret_2024";

// Définition des Licences
enum LicenseTier {
  TIER_BASIC = 0, // Shifter + Pit Limiter
  TIER_TRACK = 1, // + Launch Control
  TIER_PRO   = 2  // + Predictive Slip Control
};

std::atomic<int> currentLicense(TIER_BASIC);

// ============================================================================
// 1. DÉFINITION DES BROCHES & MACROS FAST GPIO
// ============================================================================
const int PIN_CUT          = 4;  // /!\ PULL-DOWN PHYSIQUE 10k OBLIGATOIRE /!\
const int PIN_RPM          = 5;  
const int PIN_WHEEL_FRONT  = 14; 
const int PIN_WHEEL_REAR   = 27; 
const int PIN_SHIFTER      = 26; // /!\ RC FILTER HARDWARE (100nF) OBLIGATOIRE /!\
const int PIN_CLUTCH_HALL  = 25; 
const int PIN_PIT_BTN      = 33; 

#define CUT_ON()  GPIO.out_w1ts = (1 << PIN_CUT)
#define CUT_OFF() GPIO.out_w1tc = (1 << PIN_CUT)

// ============================================================================
// 2. VARIABLES GLOBALES PARTAGÉES (std::atomic)
// ============================================================================
Preferences preferences;

std::atomic<int> cfg_shiftCutTimeMs(50);   
std::atomic<int> cfg_minShiftRpm(6000);  
std::atomic<int> cfg_launchRpm(8000);  
std::atomic<int> cfg_maxSlipPercent(15);    
std::atomic<int> cfg_wheelCircumMm(1800);  
std::atomic<int> cfg_pulsesPerRev(3);     
std::atomic<int> cfg_pitRpm(4000);  

volatile unsigned long lastRpmTime_isr = 0;
volatile int filteredRpm_isr = 0; 
volatile unsigned long frontPulses_isr = 0;
volatile unsigned long rearPulses_isr = 0;
volatile unsigned long shifterTriggerTime_isr = 0; 

std::atomic<int> speedFrontKmh_x10(0);
std::atomic<int> speedRearKmh_x10(0);
std::atomic<int> currentSlip_x10(0);
std::atomic<int> currentSlipRate_x10(0); 
std::atomic<int> liveRpm(0); 
std::atomic<int> currentMode(0); 
std::atomic<bool> isCutting(false); 

// État Machine (Core 1)
unsigned long cutEndTimeMicros = 0; 
unsigned long lastShiftTime = 0;
unsigned long lastSpeedCalcTime = 0;
unsigned long lastPitCutTime = 0; 
unsigned long lastTractionCutTime = 0; 
unsigned long lastOverrevTime = 0; 
bool slipActive = false; 
float lastSlipValue = 0.0f; 

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================================
// 3. INTERRUPTIONS (ISR)
// ============================================================================

void IRAM_ATTR isr_rpm() {
  unsigned long now = micros();
  unsigned long delta = now - lastRpmTime_isr;
  
  if (delta < 2000) return; 
  
  int instantRpm = 60000000 / delta; 
  
  if (filteredRpm_isr == 0) {
    filteredRpm_isr = instantRpm;
  } else {
    filteredRpm_isr = (instantRpm >> 2) + ((filteredRpm_isr * 3) >> 2);
  }
  lastRpmTime_isr = now;
}

void IRAM_ATTR isr_wheel_front() { 
  portENTER_CRITICAL_ISR(&mux);
  frontPulses_isr++; 
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR isr_wheel_rear() { 
  portENTER_CRITICAL_ISR(&mux);
  rearPulses_isr++; 
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR isr_shifter() {
  unsigned long now = micros();
  if (now - shifterTriggerTime_isr > 30000) { 
    shifterTriggerTime_isr = now;
  }
}

// ============================================================================
// 4. FONCTIONS DE CONTRÔLE MOTEUR (Core 1)
// ============================================================================

void triggerCut(int durationMs, int modeId) {
  unsigned long proposedEndMicros = micros() + (durationMs * 1000UL);
  
  portENTER_CRITICAL(&mux);
  if (!isCutting.load() || (long)(proposedEndMicros - cutEndTimeMicros) > 0) {
      cutEndTimeMicros = proposedEndMicros;
      currentMode.store(modeId);
  }
  isCutting.store(true);
  CUT_ON(); 
  portEXIT_CRITICAL(&mux);
}

void stopCut() {
  portENTER_CRITICAL(&mux);
  CUT_OFF(); 
  isCutting.store(false);
  currentMode.store(0); 
  portEXIT_CRITICAL(&mux);
}

void calculateDynamics() {
  unsigned long now = millis();
  unsigned long dt = now - lastSpeedCalcTime;

  if (dt < 50) return; 

  int ppr = cfg_pulsesPerRev.load();
  if (ppr < 1) return; 

  portENTER_CRITICAL(&mux);
  unsigned long fp = frontPulses_isr; frontPulses_isr = 0;
  unsigned long rp = rearPulses_isr;  rearPulses_isr = 0;
  portEXIT_CRITICAL(&mux);

  float distPerPulse = (float)cfg_wheelCircumMm.load() / (float)ppr; 
  float dtSec = dt / 1000.0f; 
  
  float vFront = ((fp * distPerPulse / 1000.0f) / dtSec) * 3.6f;
  float vRear  = ((rp * distPerPulse / 1000.0f) / dtSec) * 3.6f;
  float slip   = 0.0f;

  if (vFront > 5.0f) {
    slip = ((vRear - vFront) / vFront) * 100.0f;
  }

  float rawRate = (slip - lastSlipValue) / dtSec;
  static float filteredRate = 0.0f;
  filteredRate = (rawRate * 0.2f) + (filteredRate * 0.8f); 
  float slipRate = filteredRate;
  
  lastSlipValue = slip;
  
  speedFrontKmh_x10.store((int)(vFront * 10.0f));
  speedRearKmh_x10.store((int)(vRear * 10.0f));
  currentSlip_x10.store((int)(slip * 10.0f));
  currentSlipRate_x10.store((int)(slipRate * 10.0f));
  
  lastSpeedCalcTime = now;
}

// ============================================================================
// X. SYSTÈME DE GESTION DE LICENCE (DRM)
// ============================================================================

String generateExpectedKey(int tier) {
  String mac = WiFi.macAddress();
  String payload = mac + COMPANY_SECRET_SALT + String(tier);
  
  byte shaResult[32];
  mbedtls_md_context_t ctx;
  mbedtls_md_type_t md_type = MBEDTLS_MD_SHA256;
  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(md_type), 0);
  mbedtls_md_starts(&ctx);
  mbedtls_md_update(&ctx, (const unsigned char *)payload.c_str(), payload.length());
  mbedtls_md_finish(&ctx, shaResult);
  mbedtls_md_free(&ctx);

  String key = "";
  for(int i = 0; i < 6; i++) {
    char str[3];
    sprintf(str, "%02X", (int)shaResult[i]);
    key += str;
  }
  return key;
}

bool applyLicenseKey(String inputKey) {
  inputKey.toUpperCase();
  if (inputKey == generateExpectedKey(TIER_PRO)) {
    currentLicense.store(TIER_PRO);
    return true;
  } else if (inputKey == generateExpectedKey(TIER_TRACK)) {
    currentLicense.store(TIER_TRACK);
    return true;
  } else if (inputKey == generateExpectedKey(TIER_BASIC)) {
    currentLicense.store(TIER_BASIC);
    return true;
  }
  return false;
}

// ============================================================================
// 5. TÂCHES FreeRTOS (Core 0)
// ============================================================================
AsyncWebServer server(80);
AsyncEventSource events("/events"); 

const char* modeStrings[] = {"IDLE", "PIT LIMIT", "LAUNCH", "SLIP CTL", "SHIFTING", "OVERREV"};
const char* tierStrings[] = {"BASIC", "TRACK", "PRO (MotoGP)"};

void telemetryTask(void * pvParameters) {
  esp_task_wdt_add(NULL); 
  char jsonBuf[160]; 
  
  for(;;) {
    vTaskDelay(250 / portTICK_PERIOD_MS); 
    
    if (uxTaskGetStackHighWaterMark(NULL) < 200) {
       Serial.println("[WARN] SSE_Task RAM Watermark Critical!"); 
    }
    
    if (events.count() > 0) {
      int rpm = liveRpm.load();
      float spd = speedFrontKmh_x10.load() / 10.0f;
      float slp = currentSlip_x10.load() / 10.0f;
      int mode = constrain(currentMode.load(), 0, 5); 
      int tier = currentLicense.load();
      
      snprintf(jsonBuf, sizeof(jsonBuf), "{\"rpm\":%d,\"speed\":%.1f,\"slip\":%.1f,\"mode\":\"%s\",\"tier\":\"%s\"}", 
               rpm, spd, slp, modeStrings[mode], tierStrings[tier]);
      events.send(jsonBuf, "telemetry", millis());
    }
    esp_task_wdt_reset(); 
  }
}

void imuTask(void * pvParameters) {
  esp_task_wdt_add(NULL); 
  for(;;) {
    vTaskDelay(100 / portTICK_PERIOD_MS); 
    esp_task_wdt_reset();
  }
}

void rebootTask(void * pvParameters) {
  vTaskDelay(500 / portTICK_PERIOD_MS);
  ESP.restart();
}

// ============================================================================
// 6. SERVEUR WEB 
// ============================================================================

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Aptatio ECU 50cc</title>
  <style>
    body { font-family: Arial; text-align: center; background: #111; color: #fff; margin-top: 20px;}
    h1 { color: #f39c12; margin-bottom: 5px;}
    h3 { margin-top: 0; color: #aaa;}
    .card { background: #222; padding: 20px; border-radius: 10px; max-width: 400px; margin: auto; margin-bottom: 20px; box-shadow: 0px 4px 8px rgba(0,0,0,0.8);}
    .live-data { display: flex; justify-content: space-between; margin-top: 10px; border-top: 1px solid #444; padding-top: 10px;}
    .val { font-size: 24px; font-weight: bold; color: #00ffcc; }
    .status-badge { background: #555; padding: 5px 10px; border-radius: 20px; font-weight: bold; font-size: 14px; margin-top: 10px; display: inline-block;}
    .tier-badge { background: #8e44ad; padding: 3px 8px; border-radius: 5px; font-size: 12px; margin-bottom: 10px; display: inline-block;}
    label { display: block; margin-top: 15px; font-weight: bold; text-align: left; font-size: 14px;}
    input { width: 100%; padding: 10px; margin-top: 5px; border-radius: 5px; border: none; box-sizing: border-box; background: #eee; font-weight: bold;}
    input:disabled { background: #555; color: #888; cursor: not-allowed; }
    button { background: #f39c12; color: white; border: none; padding: 15px; margin-top: 20px; width: 100%; font-size: 16px; border-radius: 5px; cursor: pointer; font-weight: bold;}
    .btn-upgrade { background: #27ae60; margin-top: 10px;}
    .mac-addr { font-size: 10px; color: #666; margin-top: 10px; }
  </style>
</head>
<body>
  <h1>Aptatio ECU</h1>
  <div class="tier-badge" id="live_tier">Licence: BASIC</div>
  
  <div class="card">
    <h3 style="color:#fff;">Télémétrie Live</h3>
    <div class="status-badge" id="live_mode">IDLE</div>
    <div class="live-data">
      <div><span>RPM</span><br><span class="val" id="live_rpm">0</span></div>
      <div><span>Vitesse</span><br><span class="val" id="live_spd">0</span><small> km/h</small></div>
      <div><span>Slip</span><br><span class="val" id="live_slip">0</span><small> %</small></div>
    </div>
  </div>

  <div class="card">
    <h3 style="color:#fff;">Cartographie & Limites</h3>
    <label>Cut Shifter (ms):</label><input type="number" id="shiftCut" value="%SHIFT_CUT%">
    <label>RPM Min Shifter:</label><input type="number" id="minRpm" value="%MIN_RPM%">
    <label>Pit Limiter (RPM):</label><input type="number" id="pitRpm" value="%PIT_RPM%">
    
    <hr style="border-color: #444; margin-top: 20px;">
    <h4 style="color: #f39c12; text-align: left; margin-bottom: 5px;">🔓 Track & Pro Features</h4>
    
    <label>Launch RPM (Preload Zone) [TRACK+]:</label>
    <input type="number" id="launchRpm" value="%LAUNCH_RPM%" %TRACK_DISABLED%>
    
    <label>Slip Control Tolerance (%) [PRO]:</label>
    <input type="number" id="slipMax" value="%SLIP_MAX%" %PRO_DISABLED%>
    
    <div class="grid-2">
      <div><label>Circ. Roue (mm):</label><input type="number" id="wheelCirc" value="%WHEEL_CIRC%"></div>
      <div><label>Impulsions/Tour:</label><input type="number" id="pulses" value="%PULSES%"></div>
    </div>
    
    <button onclick="saveConfig()">Flasher NVRAM</button>
  </div>

  <div class="card">
    <h3 style="color:#fff;">Upgrade Licence</h3>
    <p style="font-size:12px; color:#aaa;">Passez à la version TRACK ou PRO pour débloquer le Launch et le Slip Control.</p>
    <input type="text" id="licenseKey" placeholder="Entrez la clé à 12 caractères" maxlength="12">
    <button class="btn-upgrade" onclick="upgradeLicense()">Activer</button>
    <div class="mac-addr">HWID: %MAC_ADDR%</div>
  </div>
  
  <script>
    function saveConfig() {
      const data = new URLSearchParams();
      data.append('shiftCut', document.getElementById('shiftCut').value);
      data.append('minRpm', document.getElementById('minRpm').value);
      data.append('pitRpm', document.getElementById('pitRpm').value);
      data.append('wheelCirc', document.getElementById('wheelCirc').value);
      data.append('pulses', document.getElementById('pulses').value);
      
      if(!document.getElementById('launchRpm').disabled) data.append('launchRpm', document.getElementById('launchRpm').value);
      if(!document.getElementById('slipMax').disabled) data.append('slipMax', document.getElementById('slipMax').value);
      
      fetch('/save', { method: 'POST', body: data })
        .then(response => {
          if(response.ok) alert("Paramètres flashés avec succès !");
        });
    }

    function upgradeLicense() {
      const key = document.getElementById('licenseKey').value;
      if(key.length !== 12) { alert("Format de clé invalide."); return; }
      
      const data = new URLSearchParams();
      data.append('key', key);
      
      fetch('/upgrade', { method: 'POST', body: data })
        .then(response => response.text())
        .then(text => {
          if(text === "OK") { 
            alert("Licence Upgradée ! L'ECU va redémarrer."); 
            setTimeout(() => location.reload(), 3000); 
          }
          else alert("Clé refusée.");
        });
    }

    if (!!window.EventSource) {
      var source = new EventSource('/events');
      source.addEventListener('telemetry', function(e) {
        try {
          const data = JSON.parse(e.data);
          document.getElementById('live_rpm').innerText = data.rpm;
          document.getElementById('live_spd').innerText = parseFloat(data.speed).toFixed(1);
          document.getElementById('live_slip').innerText = parseFloat(data.slip).toFixed(1);
          document.getElementById('live_tier').innerText = "Licence: " + data.tier;
          
          let modeBadge = document.getElementById('live_mode');
          modeBadge.innerText = data.mode;
          if(data.mode == "IDLE") modeBadge.style.background = "#555";
          else if(data.mode == "LAUNCH" || data.mode == "PIT LIMIT") modeBadge.style.background = "#e74c3c";
          else if(data.mode == "SHIFTING") modeBadge.style.background = "#3498db";
          else if(data.mode == "SLIP CTL") modeBadge.style.background = "#f1c40f";
          else if(data.mode == "OVERREV") modeBadge.style.background = "#8b0000"; 
        } catch(err) {}
      }, false);
    }
  </script>
</body>
</html>
)rawliteral";

String processor(const String& var){
  int tier = currentLicense.load();
  if(var == "SHIFT_CUT")  return String(cfg_shiftCutTimeMs.load());
  if(var == "MIN_RPM")    return String(cfg_minShiftRpm.load());
  if(var == "LAUNCH_RPM") return String(cfg_launchRpm.load());
  if(var == "SLIP_MAX")   return String(cfg_maxSlipPercent.load());
  if(var == "WHEEL_CIRC") return String(cfg_wheelCircumMm.load());
  if(var == "PIT_RPM")    return String(cfg_pitRpm.load());
  if(var == "PULSES")     return String(cfg_pulsesPerRev.load());
  if(var == "MAC_ADDR")   return WiFi.macAddress();
  
  if(var == "TRACK_DISABLED") return (tier < TIER_TRACK) ? "disabled" : "";
  if(var == "PRO_DISABLED")   return (tier < TIER_PRO) ? "disabled" : "";
  
  return String();
}

void setupWebServer() {
  WiFi.softAP("AptatioECU", "12345678");

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request){
    if(request->hasParam("shiftCut", true))  cfg_shiftCutTimeMs.store(constrain(request->getParam("shiftCut", true)->value().toInt(), 10, 150));
    if(request->hasParam("minRpm", true))    cfg_minShiftRpm.store(constrain(request->getParam("minRpm", true)->value().toInt(), 2000, 15000));
    if(request->hasParam("wheelCirc", true)) cfg_wheelCircumMm.store(constrain(request->getParam("wheelCirc", true)->value().toInt(), 1000, 2500));
    if(request->hasParam("pitRpm", true))    cfg_pitRpm.store(constrain(request->getParam("pitRpm", true)->value().toInt(), 2000, 10000));
    if(request->hasParam("pulses", true))    cfg_pulsesPerRev.store(constrain(request->getParam("pulses", true)->value().toInt(), 1, 20));
    
    if(currentLicense.load() >= TIER_TRACK && request->hasParam("launchRpm", true)) {
        cfg_launchRpm.store(constrain(request->getParam("launchRpm", true)->value().toInt(), 2000, 15000));
    }
    if(currentLicense.load() >= TIER_PRO && request->hasParam("slipMax", true)) {
        cfg_maxSlipPercent.store(constrain(request->getParam("slipMax", true)->value().toInt(), 0, 50));
    }
    
    preferences.begin("ecu_config", false);
    preferences.putInt("shiftCut", cfg_shiftCutTimeMs.load());
    preferences.putInt("minRpm", cfg_minShiftRpm.load());
    preferences.putInt("launchRpm", cfg_launchRpm.load());
    preferences.putInt("slipMax", cfg_maxSlipPercent.load());
    preferences.putInt("wheelCirc", cfg_wheelCircumMm.load());
    preferences.putInt("pitRpm", cfg_pitRpm.load());
    preferences.putInt("pulses", cfg_pulsesPerRev.load());
    preferences.end();
    
    request->send(200, "text/plain", "OK");
  });

  server.on("/upgrade", HTTP_POST, [](AsyncWebServerRequest *request){
    if(request->hasParam("key", true)) {
        String key = request->getParam("key", true)->value();
        if(applyLicenseKey(key)) {
            preferences.begin("ecu_config", false);
            preferences.putString("licenseKey", key); 
            preferences.putInt("licenseTier", currentLicense.load());
            preferences.end();
            
            request->send(200, "text/plain", "OK");
            
            // Délégation du reboot à une tâche FreeRTOS indépendante 
            // pour ne pas bloquer le thread de réponse AsyncWebServer.
            xTaskCreate(rebootTask, "Reboot_Task", 2048, NULL, 1, NULL);
            return;
        }
    }
    request->send(403, "text/plain", "REFUSED");
  });

  server.addHandler(&events);
  server.begin();
}

// ============================================================================
// 7. SETUP & MAIN LOOP (Core 1)
// ============================================================================

void setup() {
  Serial.begin(115200);
  
  esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);
  esp_task_wdt_add(NULL); 
  
  pinMode(PIN_CUT, OUTPUT);
  CUT_OFF(); 
  
  pinMode(PIN_RPM, INPUT_PULLUP);
  pinMode(PIN_WHEEL_FRONT, INPUT_PULLUP);
  pinMode(PIN_WHEEL_REAR, INPUT_PULLUP);
  pinMode(PIN_SHIFTER, INPUT_PULLUP); 
  pinMode(PIN_CLUTCH_HALL, INPUT_PULLUP);
  pinMode(PIN_PIT_BTN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_RPM), isr_rpm, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_WHEEL_FRONT), isr_wheel_front, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_WHEEL_REAR), isr_wheel_rear, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_SHIFTER), isr_shifter, FALLING);

  WiFi.mode(WIFI_AP);

  preferences.begin("ecu_config", true);
  cfg_shiftCutTimeMs.store(preferences.getInt("shiftCut", 50));
  cfg_minShiftRpm.store(preferences.getInt("minRpm", 6000));
  cfg_launchRpm.store(preferences.getInt("launchRpm", 8000));
  cfg_maxSlipPercent.store(preferences.getInt("slipMax", 15));
  cfg_wheelCircumMm.store(preferences.getInt("wheelCirc", 1800));
  cfg_pitRpm.store(preferences.getInt("pitRpm", 4000));
  cfg_pulsesPerRev.store(preferences.getInt("pulses", 3));
  if(cfg_pulsesPerRev.load() < 1) cfg_pulsesPerRev.store(3);
  
  String savedKey = preferences.getString("licenseKey", "");
  if (savedKey != "" && applyLicenseKey(savedKey)) {
      Serial.println("Licence Valide : Tier " + String(currentLicense.load()));
  } else {
      currentLicense.store(TIER_BASIC);
      Serial.println("Licence Base : Tier 0 (BASIC)");
  }
  preferences.end();

  setupWebServer();

  xTaskCreatePinnedToCore(telemetryTask, "SSE_Task", 6144, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(imuTask, "IMU_Task", 2048, NULL, 1, NULL, 0);
}

void loop() {
  esp_task_wdt_reset(); 

  int rpmCopy;
  unsigned long lastRpm;
  portENTER_CRITICAL(&mux);
  rpmCopy = filteredRpm_isr;
  lastRpm = lastRpmTime_isr;
  portEXIT_CRITICAL(&mux);
  
  liveRpm.store(rpmCopy);
  int tier = currentLicense.load(); 

  unsigned long st;
  portENTER_CRITICAL(&mux);
  st = shifterTriggerTime_isr;
  if (st != 0) shifterTriggerTime_isr = 0; 
  portEXIT_CRITICAL(&mux);

  // Moteur calé repassé à 500ms pour une détection plus sûre à très bas régime
  if (micros() - lastRpm > 500000) {
    liveRpm.store(0);
    portENTER_CRITICAL(&mux);
    filteredRpm_isr = 0; 
    portEXIT_CRITICAL(&mux);
  }

  unsigned long cutEndSnapshot;
  portENTER_CRITICAL(&mux);
  cutEndSnapshot = cutEndTimeMicros;
  portEXIT_CRITICAL(&mux);

  if (isCutting.load() && (long)(micros() - cutEndSnapshot) >= 0) {
    stopCut();
  }

  calculateDynamics();
  
  bool currentlyCutting = isCutting.load(); 
  int currentRpm = liveRpm.load();
  bool pitActive = digitalRead(PIN_PIT_BTN) == LOW;
  bool clutchPulled = digitalRead(PIN_CLUTCH_HALL) == LOW;
  float sf = speedFrontKmh_x10.load() / 10.0f; 

  int requestedCutMs = 0;
  int requestedMode = 0;

  if (currentRpm > 16000) {
      if (millis() - lastOverrevTime > 40) {
          requestedCutMs = 50; 
          requestedMode = 5;
          lastOverrevTime = millis();
      }
  }
  else if (pitActive && currentRpm > cfg_pitRpm.load()) {
      if (millis() - lastPitCutTime > 50) {
          requestedCutMs = 20; 
          requestedMode = 1;
          lastPitCutTime = millis();
      }
  }
  else if (st != 0 && (micros() - st) < 25000) {
      if (currentRpm >= cfg_minShiftRpm.load() && (millis() - lastShiftTime) > 250) {
          requestedCutMs = cfg_shiftCutTimeMs.load(); 
          requestedMode = 4;
          lastShiftTime = millis();
      }
  }
  else if (tier >= TIER_TRACK && clutchPulled && sf < 5.0f) {
      slipActive = false; 
      if (!currentlyCutting && (millis() - lastTractionCutTime) > 50) {
          int diff = currentRpm - cfg_launchRpm.load();
          if (diff > -200) {
              requestedCutMs = constrain(5 + (diff / 50), 5, 20); 
              requestedMode = 2;
              lastTractionCutTime = millis();
          }
      }
  }
  else if (tier >= TIER_PRO) {
      float slip = currentSlip_x10.load() / 10.0f;
      float sRate = currentSlipRate_x10.load() / 10.0f;
      
      float predictionDelta = constrain(sRate * 0.05f, -5.0f, 5.0f);
      float predictedSlip = slip + predictionDelta;
      
      float excess = predictedSlip - cfg_maxSlipPercent.load();
      float hysteresis = 3.0f; 

      if (!slipActive && excess > 0 && sf > 5.0f) {
          slipActive = true;
      } else if (slipActive && (slip < (cfg_maxSlipPercent.load() - hysteresis) || sf <= 5.0f)) {
          slipActive = false;
      }

      if (slipActive && !currentlyCutting && (millis() - lastTractionCutTime) > 50) {
          if (excess > 0) {
              float speedFactor = constrain(sf / 50.0f, 0.5f, 1.5f);
              float lowSpeedRamp = constrain((sf - 5.0f) / 5.0f, 0.0f, 1.0f);
              
              // Utilisation du Float complet pour l'arrondi (évite la troncature)
              float cutFloat = (5.0f + (excess * 1.5f * speedFactor)) * lowSpeedRamp;
              
              if (cutFloat >= 5.0f) {
                  requestedCutMs = constrain((int)roundf(cutFloat), 5, 25);
                  requestedMode = 3;
                  lastTractionCutTime = millis();
              } else {
                  requestedCutMs = 0; 
              }
          } else {
              // Zone d'hystérésis (Pneu a repris l'adhérence) : Maintien armé sans coupure réelle
              requestedCutMs = 0; 
          }
      }
  }

  if (requestedCutMs > 0) {
      if (requestedMode == 5 || currentMode.load() != requestedMode || !currentlyCutting) {
          triggerCut(requestedCutMs, requestedMode);
      }
  }
}