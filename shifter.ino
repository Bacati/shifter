/*
 * ============================================================================
 * PROJET : Module Racing 50cc 2T (PVL) - Shifter, Launch, Slip & Pit Limiter
 * VERSION: 18.0 (Immutable Release - Absolute Atomicity, EMI Safe, Zero Jitter)
 * ============================================================================
 */

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <atomic>
#include <esp_task_wdt.h> 

#define WDT_TIMEOUT_SECONDS 3 

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
std::atomic<int> currentMode(0); // 0=IDLE, 1=PIT, 2=LAUNCH, 3=SLIP, 4=SHIFT, 5=OVERREV
std::atomic<bool> isCutting(false); 

// État Machine (Core 1)
unsigned long cutEndTimeMicros = 0; 
unsigned long lastShiftTime = 0;
unsigned long lastSpeedCalcTime = 0;
unsigned long lastPitCutTime = 0; 
unsigned long lastTractionCutTime = 0; 
bool slipActive = false; 
float lastSlipValue = 0.0f; 

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================================
// 3. INTERRUPTIONS (ISR)
// ============================================================================

void IRAM_ATTR isr_rpm() {
  unsigned long now = micros();
  unsigned long delta = now - lastRpmTime_isr;
  
  // Rejet absolu EMI: 3000 µs = 20,000 RPM. (Tolérance augmentée)
  if (delta < 3000) return; 
  
  if (delta > 2000) { 
    int instantRpm = 60000000 / delta; 
    
    if (filteredRpm_isr == 0) {
      filteredRpm_isr = instantRpm;
    } else {
      filteredRpm_isr = (instantRpm >> 2) + ((filteredRpm_isr * 3) >> 2);
    }
    lastRpmTime_isr = now;
  }
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
  // Protection unifiée : Timer ET Télémétrie verrouillés simultanément
  if (!isCutting.load() || (long)(proposedEndMicros - cutEndTimeMicros) > 0) {
      cutEndTimeMicros = proposedEndMicros;
      currentMode.store(modeId);
  }
  isCutting.store(true);
  portEXIT_CRITICAL(&mux);
  
  CUT_ON(); 
}

void stopCut() {
  CUT_OFF();
  isCutting.store(false);
  currentMode.store(0); 
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
// 5. TÂCHES FreeRTOS (Core 0)
// ============================================================================
AsyncWebServer server(80);
AsyncEventSource events("/events"); 

const char* modeStrings[] = {"IDLE", "PIT LIMIT", "LAUNCH", "SLIP CTL", "SHIFTING", "OVERREV"};

void telemetryTask(void * pvParameters) {
  esp_task_wdt_add(NULL); 
  char jsonBuf[96]; 
  
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
      
      snprintf(jsonBuf, sizeof(jsonBuf), "{\"rpm\":%d,\"speed\":%.1f,\"slip\":%.1f,\"mode\":\"%s\"}", 
               rpm, spd, slp, modeStrings[mode]);
      events.send(jsonBuf, "telemetry", millis());
    }
    esp_task_wdt_reset(); 
  }
}

// ============================================================================
// 6. SERVEUR WEB 
// ============================================================================

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>MotoGP ECU 50cc</title>
  <style>
    body { font-family: Arial; text-align: center; background: #111; color: #fff; margin-top: 20px;}
    h1 { color: #f39c12; margin-bottom: 5px;}
    h3 { margin-top: 0; color: #aaa;}
    .card { background: #222; padding: 20px; border-radius: 10px; max-width: 400px; margin: auto; margin-bottom: 20px; box-shadow: 0px 4px 8px rgba(0,0,0,0.8);}
    .live-data { display: flex; justify-content: space-between; margin-top: 10px; border-top: 1px solid #444; padding-top: 10px;}
    .val { font-size: 24px; font-weight: bold; color: #00ffcc; }
    .status-badge { background: #555; padding: 5px 10px; border-radius: 20px; font-weight: bold; font-size: 14px; margin-top: 10px; display: inline-block;}
    label { display: block; margin-top: 15px; font-weight: bold; text-align: left; font-size: 14px;}
    input { width: 100%; padding: 10px; margin-top: 5px; border-radius: 5px; border: none; box-sizing: border-box; background: #eee; font-weight: bold;}
    button { background: #f39c12; color: white; border: none; padding: 15px; margin-top: 20px; width: 100%; font-size: 16px; border-radius: 5px; cursor: pointer; font-weight: bold;}
    .conn-status { font-size: 12px; color: #aaa; margin-top: 15px; }
    .grid-2 { display: grid; grid-template-columns: 1fr 1fr; gap: 10px;}
  </style>
</head>
<body>
  <h1>MotoGP ECU</h1>
  <h3>System 50cc</h3>
  
  <div class="card">
    <h3 style="color:#fff;">Télémétrie Live</h3>
    <div class="status-badge" id="live_mode">IDLE</div>
    <div class="live-data">
      <div><span>RPM</span><br><span class="val" id="live_rpm">0</span></div>
      <div><span>Vitesse</span><br><span class="val" id="live_spd">0</span><small> km/h</small></div>
      <div><span>Slip</span><br><span class="val" id="live_slip">0</span><small> %</small></div>
    </div>
    <div class="conn-status" id="conn_status">Connexion: En attente...</div>
  </div>

  <div class="card">
    <h3 style="color:#fff;">Cartographie & Limites</h3>
    <label>Cut Shifter (ms):</label><input type="number" id="shiftCut" value="%SHIFT_CUT%">
    <label>RPM Min Shifter:</label><input type="number" id="minRpm" value="%MIN_RPM%">
    <label>Launch RPM (Preload Zone):</label><input type="number" id="launchRpm" value="%LAUNCH_RPM%">
    <label>Slip Control Tolerance (%):</label><input type="number" id="slipMax" value="%SLIP_MAX%">
    <label>Pit Limiter (RPM):</label><input type="number" id="pitRpm" value="%PIT_RPM%">
    
    <div class="grid-2">
      <div><label>Circ. Roue (mm):</label><input type="number" id="wheelCirc" value="%WHEEL_CIRC%"></div>
      <div><label>Impulsions/Tour:</label><input type="number" id="pulses" value="%PULSES%"></div>
    </div>
    
    <button onclick="saveConfig()">Flasher NVRAM</button>
  </div>
  
  <script>
    function saveConfig() {
      const data = new URLSearchParams();
      data.append('shiftCut', document.getElementById('shiftCut').value);
      data.append('minRpm', document.getElementById('minRpm').value);
      data.append('launchRpm', document.getElementById('launchRpm').value);
      data.append('slipMax', document.getElementById('slipMax').value);
      data.append('wheelCirc', document.getElementById('wheelCirc').value);
      data.append('pitRpm', document.getElementById('pitRpm').value);
      data.append('pulses', document.getElementById('pulses').value);
      
      fetch('/save', { method: 'POST', body: data })
        .then(response => {
          if(response.ok) alert("Paramètres flashés avec succès !");
          else alert("Erreur lors de la sauvegarde.");
        })
        .catch(err => alert("Erreur de communication WiFi."));
    }

    if (!!window.EventSource) {
      var source = new EventSource('/events');
      
      source.addEventListener('open', function(e) {
        document.getElementById('conn_status').innerText = "Connexion: Link Actif";
        document.getElementById('conn_status').style.color = "#00ffcc";
      }, false);

      source.addEventListener('telemetry', function(e) {
        try {
          const data = JSON.parse(e.data);
          document.getElementById('live_rpm').innerText = data.rpm;
          document.getElementById('live_spd').innerText = parseFloat(data.speed).toFixed(1);
          document.getElementById('live_slip').innerText = parseFloat(data.slip).toFixed(1);
          
          let modeBadge = document.getElementById('live_mode');
          modeBadge.innerText = data.mode;
          if(data.mode == "IDLE") modeBadge.style.background = "#555";
          else if(data.mode == "LAUNCH" || data.mode == "PIT LIMIT") modeBadge.style.background = "#e74c3c";
          else if(data.mode == "SHIFTING") modeBadge.style.background = "#3498db";
          else if(data.mode == "SLIP CTL") modeBadge.style.background = "#f1c40f";
          else if(data.mode == "OVERREV") modeBadge.style.background = "#8b0000"; 
        } catch(err) {}
      }, false);

      source.addEventListener('error', function(e) {
        if (e.target.readyState != EventSource.OPEN) {
          document.getElementById('conn_status').innerText = "Connexion: Perdue. Reconnexion...";
          document.getElementById('conn_status').style.color = "orange";
          document.getElementById('live_mode').innerText = "OFFLINE";
          document.getElementById('live_mode').style.background = "#e74c3c";
        }
      }, false);
    }
  </script>
</body>
</html>
)rawliteral";

String processor(const String& var){
  if(var == "SHIFT_CUT")  return String(cfg_shiftCutTimeMs.load());
  if(var == "MIN_RPM")    return String(cfg_minShiftRpm.load());
  if(var == "LAUNCH_RPM") return String(cfg_launchRpm.load());
  if(var == "SLIP_MAX")   return String(cfg_maxSlipPercent.load());
  if(var == "WHEEL_CIRC") return String(cfg_wheelCircumMm.load());
  if(var == "PIT_RPM")    return String(cfg_pitRpm.load());
  if(var == "PULSES")     return String(cfg_pulsesPerRev.load());
  return String();
}

void setupWebServer() {
  WiFi.softAP("MotoGPECU", "12345678");

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request){
    if(request->hasParam("shiftCut", true))  cfg_shiftCutTimeMs.store(constrain(request->getParam("shiftCut", true)->value().toInt(), 10, 150));
    if(request->hasParam("minRpm", true))    cfg_minShiftRpm.store(constrain(request->getParam("minRpm", true)->value().toInt(), 2000, 15000));
    if(request->hasParam("launchRpm", true)) cfg_launchRpm.store(constrain(request->getParam("launchRpm", true)->value().toInt(), 2000, 15000));
    if(request->hasParam("slipMax", true))   cfg_maxSlipPercent.store(constrain(request->getParam("slipMax", true)->value().toInt(), 0, 50));
    if(request->hasParam("wheelCirc", true)) cfg_wheelCircumMm.store(constrain(request->getParam("wheelCirc", true)->value().toInt(), 1000, 2500));
    if(request->hasParam("pitRpm", true))    cfg_pitRpm.store(constrain(request->getParam("pitRpm", true)->value().toInt(), 2000, 10000));
    if(request->hasParam("pulses", true))    cfg_pulsesPerRev.store(constrain(request->getParam("pulses", true)->value().toInt(), 1, 20));
    
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

  preferences.begin("ecu_config", true);
  cfg_shiftCutTimeMs.store(preferences.getInt("shiftCut", 50));
  cfg_minShiftRpm.store(preferences.getInt("minRpm", 6000));
  cfg_launchRpm.store(preferences.getInt("launchRpm", 8000));
  cfg_maxSlipPercent.store(preferences.getInt("slipMax", 15));
  cfg_wheelCircumMm.store(preferences.getInt("wheelCirc", 1800));
  cfg_pitRpm.store(preferences.getInt("pitRpm", 4000));
  cfg_pulsesPerRev.store(preferences.getInt("pulses", 3));
  if(cfg_pulsesPerRev.load() < 1) cfg_pulsesPerRev.store(3);
  preferences.end();

  setupWebServer();

  xTaskCreatePinnedToCore(telemetryTask, "SSE_Task", 6144, NULL, 2, NULL, 0);
}

void loop() {
  esp_task_wdt_reset(); 

  // 1. MISE A JOUR DES CAPTEURS GLOBAUX
  int rpmCopy;
  unsigned long lastRpm;
  portENTER_CRITICAL(&mux);
  rpmCopy = filteredRpm_isr;
  lastRpm = lastRpmTime_isr;
  portEXIT_CRITICAL(&mux);
  
  liveRpm.store(rpmCopy);

  unsigned long st;
  portENTER_CRITICAL(&mux);
  st = shifterTriggerTime_isr;
  if (st != 0) shifterTriggerTime_isr = 0; 
  portEXIT_CRITICAL(&mux);

  // Sécurité Moteur Calé
  if (micros() - lastRpm > 250000) {
    liveRpm.store(0);
    portENTER_CRITICAL(&mux);
    filteredRpm_isr = 0; 
    portEXIT_CRITICAL(&mux);
  }

  // Libération logicielle de la coupure (Atomique)
  unsigned long cutEndSnapshot;
  portENTER_CRITICAL(&mux);
  cutEndSnapshot = cutEndTimeMicros;
  portEXIT_CRITICAL(&mux);

  if (isCutting.load() && (long)(micros() - cutEndSnapshot) >= 0) {
    stopCut();
  }

  calculateDynamics();
  
  // 2. RECUEIL DES ETATS
  bool currentlyCutting = isCutting.load(); 
  int currentRpm = liveRpm.load();
  bool pitActive = digitalRead(PIN_PIT_BTN) == LOW;
  bool clutchPulled = digitalRead(PIN_CLUTCH_HALL) == LOW;
  float sf = speedFrontKmh_x10.load() / 10.0f; 

  int requestedCutMs = 0;
  int requestedMode = 0;

  // 3. L'ARBITRE CENTRAL (Hiérarchie de Priorité Stricte)

  // Priorité 1 : OVERREV LIMITER (Sécurité mécanique)
  if (currentRpm > 16000) {
      requestedCutMs = 50; 
      requestedMode = 5;
  }
  // Priorité 2 : PIT LIMITER 
  else if (pitActive && currentRpm > cfg_pitRpm.load()) {
      if (millis() - lastPitCutTime > 50) {
          requestedCutMs = 20; 
          requestedMode = 1;
          lastPitCutTime = millis();
      }
  }
  // Priorité 3 : QUICKSHIFTER 
  else if (st != 0 && (micros() - st) < 25000) {
      if (currentRpm >= cfg_minShiftRpm.load() && (millis() - lastShiftTime) > 250) {
          requestedCutMs = cfg_shiftCutTimeMs.load(); 
          requestedMode = 4;
          lastShiftTime = millis();
      }
  }
  // Priorité 4 : LAUNCH CONTROL 
  else if (clutchPulled && sf < 5.0f) {
      slipActive = false; 
      if (!currentlyCutting && (millis() - lastTractionCutTime) > 30) {
          int diff = currentRpm - cfg_launchRpm.load();
          if (diff > -200) {
              requestedCutMs = constrain(5 + (diff / 50), 5, 20); 
              requestedMode = 2;
              lastTractionCutTime = millis();
          }
      }
  }
  // Priorité 5 : SLIP CONTROL 
  else {
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

      if (slipActive && !currentlyCutting && (millis() - lastTractionCutTime) > 30) {
          requestedCutMs = 5; 
          if (excess > 0) {
              float speedFactor = constrain(sf / 50.0f, 0.5f, 1.5f);
              float lowSpeedRamp = constrain((sf - 5.0f) / 5.0f, 0.0f, 1.0f);
              requestedCutMs = constrain(5 + (int)(excess * 1.5f * speedFactor), 5, 25);
              requestedCutMs = (int)(requestedCutMs * lowSpeedRamp); 
          }
          if (requestedCutMs >= 5) {
              requestedMode = 3;
              lastTractionCutTime = millis();
          } else {
              requestedCutMs = 0; 
          }
      }
  }

  // 4. EXECUTION DE L'ORDRE
  if (requestedCutMs > 0) {
      if (requestedMode == 5 || currentMode.load() != requestedMode || !currentlyCutting) {
          triggerCut(requestedCutMs, requestedMode);
      }
  }

}