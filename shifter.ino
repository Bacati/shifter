/*
 * ============================================================================
 * PROJET : Module Racing 50cc 2T (PVL) - Shifter, Launch, Slip & Pit Limiter
 * VERSION: 6.0 (Masterpiece - RTOS Multi-Core Safe & Zéro Heap Fragmentation)
 * ============================================================================
 * Description : 
 * - Quickshifter par mise à la masse (Kill-Switch PVL).
 * - Launch Control 2-Step (limiteur de régime à l'arrêt, sur capteur levier).
 * - Slip Control (Mini Traction Control) pour départ arrêté en ligne droite.
 * - Pit Limiter via interrupteur au guidon.
 * - Serveur Web et Server-Sent Events (SSE) sur Core 0.
 * - Boucle de contrôle moteur temps-réel sur Core 1.
 * ============================================================================
 */

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>

// ============================================================================
// 1. DÉFINITION DES BROCHES (Configuration ESP32 S3 / Universelle Safe)
// ============================================================================
const int PIN_CUT          = 4;  // Sortie vers Optocoupleur/Thyristor (HIGH = Coupe l'allumage)
const int PIN_RPM          = 5;  // Entrée signal inductif antiparasite
const int PIN_WHEEL_FRONT  = 14; // Capteur Hall Roue Avant
const int PIN_WHEEL_REAR   = 27; // Capteur Hall Roue Arrière
const int PIN_SHIFTER      = 26; // Capteur Sélecteur de vitesse (Hall/Piezo)
const int PIN_CLUTCH_HALL  = 25; // Capteur Hall Embrayage (Launch Control)
const int PIN_PIT_BTN      = 33; // Interrupteur Pit Limiter (ON/OFF)

// ============================================================================
// 2. VARIABLES GLOBALES & PARAMÈTRES WEB
// ============================================================================
Preferences preferences;

// Paramètres de configuration (Modifiés par le Web, lus par le Core 1)
volatile int cfg_shiftCutTimeMs  = 50;   
volatile int cfg_minShiftRpm     = 6000;  
volatile int cfg_launchRpm       = 8000;  
volatile int cfg_maxSlipPercent  = 15;    
volatile int cfg_wheelCircumMm   = 1800;  
volatile int cfg_pitRpm          = 4000;  

// Variables ISR (Modifiées par les interruptions matérielles)
volatile unsigned long lastRpmTime_isr = 0;
volatile int currentRpm_isr = 0;
volatile unsigned long frontPulses_isr = 0;
volatile unsigned long rearPulses_isr = 0;

// Variables de travail (Partagées inter-cores via Mutex)
volatile float speedFrontKmh = 0.0;
volatile float speedRearKmh = 0.0;
volatile float currentSlip = 0.0;
volatile int liveRpm = 0; 

// État de la coupure moteur
bool isCutting = false;
unsigned long cutEndTime = 0;
unsigned long lastShiftTime = 0;
unsigned long lastSpeedCalcTime = 0;
unsigned long lastPitCutTime = 0; 

const int PULSES_PER_REV = 3; // Nombre d'aimants/dents par roue

// Spinlock pour les sections critiques (Atomicité RTOS)
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================================
// 3. INTERRUPTIONS MATÉRIELLES (ISR)
// ============================================================================

// Interruption : Signal RPM Inductif
void IRAM_ATTR isr_rpm() {
  unsigned long now = micros();
  unsigned long delta = now - lastRpmTime_isr;
  // Debounce (ignore les impulsions de bruit parasites < 2ms)
  if (delta > 2000) { 
    currentRpm_isr = 60000000 / delta; 
    lastRpmTime_isr = now;
  }
}

// Interruption : Roue Avant
void IRAM_ATTR isr_wheel_front() { 
  portENTER_CRITICAL_ISR(&mux);
  frontPulses_isr++; 
  portEXIT_CRITICAL_ISR(&mux);
}

// Interruption : Roue Arrière
void IRAM_ATTR isr_wheel_rear() { 
  portENTER_CRITICAL_ISR(&mux);
  rearPulses_isr++; 
  portEXIT_CRITICAL_ISR(&mux);
}

// ============================================================================
// 4. FONCTIONS DE CONTRÔLE MOTEUR (Temps-Réel sur Core 1)
// ============================================================================

// Fonction : Fermeture du circuit à la masse (Coupure d'étincelle)
void triggerCut(int durationMs) {
  isCutting = true;
  cutEndTime = millis() + durationMs;
  digitalWrite(PIN_CUT, HIGH); 
}

// Fonction : Calculs physiques (Vitesses et Glissement) à 20Hz
void calculateDynamics() {
  if (millis() - lastSpeedCalcTime >= 50) { 
    
    // Lecture et RAZ atomiques des impulsions depuis les ISR
    portENTER_CRITICAL(&mux);
    unsigned long fp = frontPulses_isr; frontPulses_isr = 0;
    unsigned long rp = rearPulses_isr;  rearPulses_isr = 0;
    portEXIT_CRITICAL(&mux);

    float distPerPulse = (float)cfg_wheelCircumMm / (float)PULSES_PER_REV; 
    
    speedFrontKmh = ((fp * distPerPulse) / 50.0) * 3.6;
    speedRearKmh  = ((rp * distPerPulse) / 50.0) * 3.6;
    
    if (speedFrontKmh > 5.0) {
      currentSlip = ((speedRearKmh - speedFrontKmh) / speedFrontKmh) * 100.0;
    } else {
      currentSlip = 0.0;
    }
    lastSpeedCalcTime = millis();
  }
}

// Fonction : Gestion du Stand et du Départ Arrêté
void handleLaunchAndPit() {
  bool pitActive = digitalRead(PIN_PIT_BTN) == LOW;
  bool clutchPulled = digitalRead(PIN_CLUTCH_HALL) == LOW;

  // PRIORITÉ 1 : Le Pit Limiter (Avec Cooldown de 50ms)
  if (pitActive) {
    if (liveRpm > cfg_pitRpm && !isCutting && (millis() - lastPitCutTime) > 50) {
      triggerCut(20); 
      lastPitCutTime = millis();
    }
    return; // On ignore le reste si on est dans les stands
  }

  // PRIORITÉ 2 : Le Départ (Launch ou Slip Control)
  if (clutchPulled && speedFrontKmh < 5.0) {
    // 2-Step Limiter : Moto à l'arrêt, levier tiré
    if (liveRpm > cfg_launchRpm && !isCutting) {
      triggerCut(15); 
    }
  } else {
    // Slip Control : La moto avance, on gère la glisse
    if (currentSlip > cfg_maxSlipPercent && speedFrontKmh > 5.0 && !isCutting) {
      triggerCut(10); 
    }
  }
}

// Fonction : Gestion du Quickshifter à la volée
void handleQuickshifter() {
  bool shifterPressed = digitalRead(PIN_SHIFTER) == LOW; 
  // Cooldown de 250ms pour autoriser le passage rapide des rapports successifs
  if (shifterPressed && liveRpm >= cfg_minShiftRpm && (millis() - lastShiftTime) > 250) {
    triggerCut(cfg_shiftCutTimeMs);
    lastShiftTime = millis();
  }
}

// ============================================================================
// 5. TÂCHES FreeRTOS (Core 0 : Réseau et Capteurs Lents)
// ============================================================================
AsyncWebServer server(80);
AsyncEventSource events("/events"); // Canal Server-Sent Events

// Tâche SSE : Envoi de la télémétrie vers l'interface Web (Priorité 2)
void telemetryTask(void * pvParameters) {
  char jsonBuf[64]; // Allocation statique (zéro fragmentation)
  
  for(;;) {
    vTaskDelay(250 / portTICK_PERIOD_MS); 
    
    // N'envoie des données réseau que si un téléphone/PC affiche la page
    if (events.count() > 0) {
      // Snapshot cohérent instant T (évite le "Tearing")
      portENTER_CRITICAL(&mux);
      int rpm   = liveRpm;
      float spd = speedFrontKmh;
      float slp = currentSlip;
      portEXIT_CRITICAL(&mux);
      
      snprintf(jsonBuf, sizeof(jsonBuf), "{\"rpm\":%d,\"speed\":%.1f,\"slip\":%.1f}", rpm, spd, slp);
      events.send(jsonBuf, "telemetry", millis());
    }
  }
}

// Tâche IMU : Futur enregistrement des capteurs (Priorité 1)
void imuTask(void * pvParameters) {
  for(;;) {
    // Lecture Capteur d'inclinaison & SD
    vTaskDelay(10 / portTICK_PERIOD_MS); 
  }
}

// ============================================================================
// 6. SERVEUR WEB (Page HTML & Traitement POST)
// ============================================================================

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Racing 50cc ECU</title>
  <style>
    body { font-family: Arial; text-align: center; background: #222; color: #fff;}
    h1 { color: #f39c12; }
    .card { background: #333; padding: 20px; border-radius: 10px; max-width: 400px; margin: auto; margin-bottom: 20px;}
    .live-data { display: flex; justify-content: space-between; margin-top: 10px; border-top: 1px solid #555; padding-top: 10px;}
    .val { font-size: 24px; font-weight: bold; color: #00ffcc; }
    label { display: block; margin-top: 15px; font-weight: bold; text-align: left; font-size: 14px;}
    input { width: 100%; padding: 10px; margin-top: 5px; border-radius: 5px; border: none; box-sizing: border-box; background: #eee;}
    button { background: #f39c12; color: white; border: none; padding: 15px; margin-top: 20px; width: 100%; font-size: 16px; border-radius: 5px; cursor: pointer; font-weight: bold;}
    .status { font-size: 12px; color: #aaa; margin-top: 10px; }
  </style>
</head>
<body>
  <h1>ECU Racing 50cc</h1>
  
  <div class="card">
    <h3>Télémétrie Temps Réel</h3>
    <div class="live-data">
      <div><span>RPM</span><br><span class="val" id="live_rpm">0</span></div>
      <div><span>Vitesse</span><br><span class="val" id="live_spd">0</span><small> km/h</small></div>
      <div><span>Slip Ratio</span><br><span class="val" id="live_slip">0</span><small> %</small></div>
    </div>
    <div class="status" id="conn_status">Connexion: En attente...</div>
  </div>

  <div class="card">
    <h3>Paramètres de Course</h3>
    <label>Cut Shifter (10-150 ms):</label><input type="number" id="shiftCut" value="%SHIFT_CUT%">
    <label>RPM Min Shifter:</label><input type="number" id="minRpm" value="%MIN_RPM%">
    <label>Launch RPM (2-Step):</label><input type="number" id="launchRpm" value="%LAUNCH_RPM%">
    <label>Slip Control Max (0-50 %):</label><input type="number" id="slipMax" value="%SLIP_MAX%">
    <label>Circ. Roue (mm):</label><input type="number" id="wheelCirc" value="%WHEEL_CIRC%">
    <label>Pit Limiter (RPM):</label><input type="number" id="pitRpm" value="%PIT_RPM%">
    <button onclick="saveConfig()">Sauvegarder dans l'ECU</button>
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
      
      fetch('/save', { method: 'POST', body: data })
        .then(response => {
          if(response.ok) alert("Sauvegarde réussie !");
          else alert("Erreur lors de la sauvegarde.");
        })
        .catch(err => alert("Erreur de communication WiFi."));
    }

    if (!!window.EventSource) {
      var source = new EventSource('/events');
      
      source.addEventListener('open', function(e) {
        document.getElementById('conn_status').innerText = "Connexion: Connecté (Live)";
        document.getElementById('conn_status').style.color = "#00ffcc";
      }, false);

      source.addEventListener('telemetry', function(e) {
        try {
          const data = JSON.parse(e.data);
          document.getElementById('live_rpm').innerText = data.rpm;
          document.getElementById('live_spd').innerText = parseFloat(data.speed).toFixed(1);
          document.getElementById('live_slip').innerText = parseFloat(data.slip).toFixed(1);
        } catch(err) {}
      }, false);

      source.addEventListener('error', function(e) {
        if (e.target.readyState != EventSource.OPEN) {
          document.getElementById('conn_status').innerText = "Connexion: Perdue. Reconnexion...";
          document.getElementById('conn_status').style.color = "orange";
        }
      }, false);
    }
  </script>
</body>
</html>
)rawliteral";

String processor(const String& var){
  if(var == "SHIFT_CUT")  return String(cfg_shiftCutTimeMs);
  if(var == "MIN_RPM")    return String(cfg_minShiftRpm);
  if(var == "LAUNCH_RPM") return String(cfg_launchRpm);
  if(var == "SLIP_MAX")   return String(cfg_maxSlipPercent);
  if(var == "WHEEL_CIRC") return String(cfg_wheelCircumMm);
  if(var == "PIT_RPM")    return String(cfg_pitRpm);
  return String();
}

void setupWebServer() {
  WiFi.softAP("RacingECU", "12345678");

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request){
    // Validation sécurisée des paramètres Web
    if(request->hasParam("shiftCut", true))  cfg_shiftCutTimeMs = constrain(request->getParam("shiftCut", true)->value().toInt(), 10, 150);
    if(request->hasParam("minRpm", true))    cfg_minShiftRpm    = constrain(request->getParam("minRpm", true)->value().toInt(), 2000, 15000);
    if(request->hasParam("launchRpm", true)) cfg_launchRpm      = constrain(request->getParam("launchRpm", true)->value().toInt(), 2000, 15000);
    if(request->hasParam("slipMax", true))   cfg_maxSlipPercent = constrain(request->getParam("slipMax", true)->value().toInt(), 0, 50);
    if(request->hasParam("wheelCirc", true)) cfg_wheelCircumMm  = constrain(request->getParam("wheelCirc", true)->value().toInt(), 1000, 2500);
    if(request->hasParam("pitRpm", true))    cfg_pitRpm         = constrain(request->getParam("pitRpm", true)->value().toInt(), 2000, 10000);
    
    // Sauvegarde en mémoire morte NVRAM
    preferences.begin("ecu_config", false);
    preferences.putInt("shiftCut", cfg_shiftCutTimeMs);
    preferences.putInt("minRpm", cfg_minShiftRpm);
    preferences.putInt("launchRpm", cfg_launchRpm);
    preferences.putInt("slipMax", cfg_maxSlipPercent);
    preferences.putInt("wheelCirc", cfg_wheelCircumMm);
    preferences.putInt("pitRpm", cfg_pitRpm);
    preferences.end();
    
    request->send(200, "text/plain", "OK");
  });

  server.addHandler(&events);
  server.begin();
}

// ============================================================================
// 7. SETUP & MAIN LOOP (Coeur du Programme)
// ============================================================================

void setup() {
  Serial.begin(115200);
  
  // Configuration Pin Output (Cut PVL)
  pinMode(PIN_CUT, OUTPUT);
  digitalWrite(PIN_CUT, LOW);
  
  // Configuration Pins Input (avec Pull-Up interne)
  pinMode(PIN_RPM, INPUT_PULLUP);
  pinMode(PIN_WHEEL_FRONT, INPUT_PULLUP);
  pinMode(PIN_WHEEL_REAR, INPUT_PULLUP);
  pinMode(PIN_SHIFTER, INPUT_PULLUP);
  pinMode(PIN_CLUTCH_HALL, INPUT_PULLUP);
  pinMode(PIN_PIT_BTN, INPUT_PULLUP);

  // Attachement des interruptions matérielles
  attachInterrupt(digitalPinToInterrupt(PIN_RPM), isr_rpm, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_WHEEL_FRONT), isr_wheel_front, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_WHEEL_REAR), isr_wheel_rear, RISING);

  // Chargement de la configuration sauvegardée
  preferences.begin("ecu_config", true);
  cfg_shiftCutTimeMs = preferences.getInt("shiftCut", 50);
  cfg_minShiftRpm    = preferences.getInt("minRpm", 6000);
  cfg_launchRpm      = preferences.getInt("launchRpm", 8000);
  cfg_maxSlipPercent = preferences.getInt("slipMax", 15);
  cfg_wheelCircumMm  = preferences.getInt("wheelCirc", 1800);
  cfg_pitRpm         = preferences.getInt("pitRpm", 4000);
  preferences.end();

  // Démarrage Réseau
  setupWebServer();

  // Affectation des tâches de télémétrie sur le Core 0
  xTaskCreatePinnedToCore(telemetryTask, "SSE_Task", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(imuTask,       "IMU_Task", 4096, NULL, 1, NULL, 0);
}

// Boucle Principale Temps-Réel (Core 1)
void loop() {
  // 1. EXTRACTION SÉCURISÉE DU RPM (Fix Race Condition)
  portENTER_CRITICAL(&mux);
  liveRpm = currentRpm_isr;
  unsigned long lastRpm = lastRpmTime_isr;
  portEXIT_CRITICAL(&mux);

  // 2. TIMEOUT MOTEUR (Vérification si le moteur a calé)
  if (micros() - lastRpm > 500000) {
    liveRpm = 0;
    // Remise à zéro synchronisée de la source ISR
    portENTER_CRITICAL(&mux);
    currentRpm_isr = 0;
    portEXIT_CRITICAL(&mux);
  }

  // 3. FIN DE COUPURE (Libération de la masse)
  if (isCutting && millis() > cutEndTime) {
    digitalWrite(PIN_CUT, LOW);
    isCutting = false;
  }

  // 4. CALCULS DYNAMIQUES
  calculateDynamics();

  // 5. LOGIQUE MOTEUR ET PRIORITÉS DE COUPURE
  if (!isCutting) {
    handleLaunchAndPit(); 
  }
  
  if (!isCutting) {
    handleQuickshifter(); 
  }

  // Boucle libre, yield() maintient le système stable sans le ralentir
  yield(); 
}