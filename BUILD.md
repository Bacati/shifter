# Aptatio ECU 50cc 2T — Build, câblage & sécurité

`README.md` contient le dossier de conception. Ce fichier couvre la mise en œuvre
technique de la **V23.0** (refonte modulaire).

---

## 1. Arborescence

`shifter.ino` (setup/loop only) reste à la racine ; les modules sont rangés
par domaine dans `src/`. Compatible **Arduino IDE** (compile `src/**`
récursivement) **et** **PlatformIO** (`src_dir = .`, testé : build OK, Flash 63 %).

```
shifter.ino                    point d'entrée (setup / loop)
platformio.ini  .gitignore  README.md  BUILD.md
src/
  core/        Version.h  Config.h  Types.h  SharedState.{h,cpp}
  sensors/     Sensors.{h,cpp}  Imu.{h,cpp}
  actuator/    IgnitionCut.{h,cpp}
  control/     Arbiter.{h,cpp}
  persistence/ Storage.{h,cpp}  License.{h,cpp}  secrets.example.h  [secrets.h]
  net/         Network.{h,cpp}  WebServer.{h,cpp}  WebPage.h  Telemetry.{h,cpp}
```

| Module | Rôle |
|---|---|
| `core/Config.h` · `Version.h` · `Types.h` | Broches, constantes, enums, `EcuConfig` |
| `core/SharedState.*` | Frontière de données Core 0 ↔ Core 1 (atomics + spinlock) |
| `sensors/Sensors.*` | ISR, filtre RPM/EMI, vitesses roues **(mesure de période)**, glisse, anti-rebonds |
| `sensors/Imu.*` | MPU6050 (I²C) → angle de roulis → garde-fou anti-angle du slip control |
| `actuator/IgnitionCut.*` | Broche de coupure + application de la priorité stricte |
| `control/Arbiter.*` | Arbitre central des assistances + `leanAuthority()` (anti-angle) |
| `persistence/Storage.*` | NVS (mutex) + `sanitize()` (clamp unique) |
| `persistence/License.*` | DRM HMAC-SHA256 lié au HWID + anti-brute-force |
| `net/Network.*` | AP WPA2 par appareil + latch de reboot |
| `net/WebServer.*` · `WebPage.h` | Routes HTTP/SSE + auth Bearer + OTA |
| `net/Telemetry.*` | Tâches Core 0 : producteur SSE + superviseur fail-safe |

> Les `#include` inter-modules utilisent des chemins relatifs
> (`#include "../core/Config.h"`), donc l'arborescence se déplace/renomme
> sans casser la résolution, quel que soit le point d'entrée.

---

## 2. Prérequis

- **Cœur** : Arduino-ESP32 **2.0.x** (ESP-IDF 4.4). La signature
  `esp_task_wdt_init(uint32_t, bool)` utilisée ici est celle de l'IDF 4.
- **Bibliothèques** :
  - PlatformIO : `ESP32Async/AsyncTCP` + `ESP32Async/ESPAsyncWebServer` (voir `platformio.ini`).
  - Arduino IDE : installer *AsyncTCP* et *ESPAsyncWebServer* (forks maintenus
    « ESP32Async ») dans `~/Documents/Arduino/libraries/`.
- `mbedtls` : fourni par le cœur ESP32 (aucune install).

### Secret de licence (obligatoire)

```bash
cp src/persistence/secrets.example.h src/persistence/secrets.h
# éditer src/persistence/secrets.h :
#   LICENSE_HMAC_SECRET = chaîne aléatoire ≥ 32 caractères
```

`src/persistence/secrets.h` est gitignoré. Sans lui, la compilation **échoue
volontairement** (`#error`). Pour un build de test uniquement :
`-DALLOW_INSECURE_DEFAULT_SECRET`.

---

## 3. Compilation

PlatformIO :
```bash
pio run -e esp32dev
pio run -e esp32dev -t upload
pio device monitor
```

Arduino IDE : ouvrir `shifter.ino`, carte « ESP32 Dev Module », téléverser.

Arduino CLI :
```bash
arduino-cli compile --fqbn esp32:esp32:esp32 .
```

> **Quirk PlatformIO/Windows** : au **tout premier** build d'un nouvel
> environnement, SCons peut échouer avec `can't create ... : No such file or
> directory` (création des sous-dossiers `.pio/build/<env>/src/src/...`).
> **Relance simplement la commande** : le 2ᵉ passage réussit. Les builds
> incrémentaux suivants sont OK.

### Simulateur Wokwi (`-DWOKWI_SIM`)

```bash
pio run -e wokwi
```

Firmware `.pio/build/wokwi/firmware.bin`. Cet environnement :

- **désactive le WiFi / le serveur web / la télémétrie SSE** — l'émulation
  WiFi de Wokwi est très lente ; sans elle le simulateur tourne à ~100 % et la
  LED réagit instantanément. (Le test du serveur web se fait sur vraie carte.)
- force la **licence PRO** ;
- lance un **faux moteur** (`src/sim/SimEngine.cpp`) qui balaye régime /
  vitesses / glisse en boucle (5 phases de 4 s) → LAUNCH / SHIFT / SLIP /
  OVERREV visibles sur la LED CUT et l'analyseur logique, sans matériel ;
- clignote 6× la LED CUT au boot (diagnostic « le firmware tourne »).

`diagram.json` : ESP32 + LED sur D4 (CUT) + boutons D26/D25/D33
(shifter / embrayage / pit). `wokwi.toml` pointe sur ce build.

L'environnement `esp32dev` (vraie carte) garde le WiFi complet.

---

## 4. Câblage

| Signal | GPIO | Remarque |
|---|---|---|
| Coupure allumage (`CUT`) | 4 | **PULL-DOWN 10 kΩ physique obligatoire.** Sortie → optocoupleur → thyristor/MOSFET SiC. `CUT` haut = allumage à la masse. |
| Trigger RPM | 5 | 1 impulsion / tour vilebrequin. GPIO5 est *strapping* : garder l'entrée au repos à HIGH au boot (INPUT_PULLUP OK). |
| Roue avant | 14 | Capteur VR/Hall. GPIO14 émet un bref PWM au boot — sans effet sur une entrée. |
| Roue arrière | 27 | |
| Sélecteur (shifter) | 26 | **Filtre RC 100 nF matériel obligatoire** (anti-rebond). |
| Contacteur embrayage (Hall) | 25 | Actif à l'état bas (tiré = LOW). |
| Bouton pit | 33 | Actif à l'état bas. |
| IMU MPU6050 — SDA | 21 | I²C, adresse 0x68 (AD0 à la masse). Pull-ups 4,7 kΩ vers 3V3. |
| IMU MPU6050 — SCL | 22 | Monter le capteur solidaire du châssis. Ajuster le mapping d'axes dans `cfg::IMU_ACC_ROLL_*` / `IMU_GYRO_ROLL_AXIS` / `IMU_ROLL_SIGN`. |

Toutes les entrées TOR sont en `INPUT_PULLUP`. Un fil coupé → état « relâché »
(fail-safe). Les macros `GPIO.out_w1ts/w1tc` n'adressent que GPIO 0–31
(`static_assert` sur `pins::CUT`).

**IMU absent / muet** → le slip control est **désactivé** (fail-safe, `IMU_REQUIRED=1`).
Pour un banc sans MPU6050 : compiler avec `-DIMU_REQUIRED=0` (slip control sans
garde-fou anti-angle — **test uniquement**, jamais en production).

---

## 5. Réseau & accès

Au démarrage, le port série (115200) affiche :

```
  WiFi SSID : AptatioECU-XXXXXX
  WiFi PASS : <12 caractères générés au 1er boot, stockés en NVS>
  IP        : 192.168.4.1
  CONFIG PIN: <6 chiffres, régénéré à CHAQUE boot>
```

- **SSID** : suffixé par les 6 derniers octets hex de la MAC. WPA2, canal 6, 2 clients max.
- **Mot de passe AP** : par appareil, persistant. Rotation via **WiFi → Nouveau
  mot de passe AP** dans l'UI (auth requise, reboot).
- **PIN de configuration** : éphémère (RAM), affiché uniquement sur le série,
  changé à chaque boot. Il s'échange contre un **jeton Bearer** (TTL 15 min)
  via `/api/login`. Lockout 60 s après 5 échecs.

### Endpoints

| Méthode | Route | Auth | Notes |
|---|---|---|---|
| GET | `/` | — | page statique (aucune substitution serveur) |
| GET | `/api/state` | — | JSON config + tier + ssid + hwid + flags |
| GET | `/events` | — | flux SSE télémétrie |
| POST | `/api/login` | — | `pin=` → `{token,ttl}` ; rate-limité |
| POST | `/api/config` | **Bearer** | champs `launchRpm`/`maxSlipPercent` → 403 si tier insuffisant |
| POST | `/api/license` | rate-limité | `key=` ; lockout 5 min après 8 échecs ; reboot au succès |
| POST | `/api/wifi` | **Bearer** | rotation mot de passe AP ; reboot |
| POST | `/api/ota` | **Bearer** | `Update.h` ; **refusé si RPM ≠ 0** ; reboot au succès |

En-têtes : `X-Content-Type-Options: nosniff`, `Cache-Control: no-store`,
`Referrer-Policy: no-referrer`, CSP restrictive sur `/`. Pas de CORS.

---

## 6. Licences (« upgrade pour les différentes versions »)

```
cle = HEX_MAJ( HMAC-SHA256( LICENSE_HMAC_SECRET , "<MAC>:<tier>" ) )[0..15]
```

- 16 caractères hex (64 bits), comparaison **à temps constant**.
- `tier` : `0`=BASIC, `1`=TRACK, `2`=PRO.
- Le tier stocké en NVS est **revalidé** au boot contre la clé stockée : écrire
  `licenseTier=2` en NVS ne débloque rien.
- Fonctions par tier : BASIC = shifter + pit + overrev ; TRACK = + launch ;
  PRO = + predictive slip.
- Diagnostic usine : compiler avec `-DLICENSE_PRINT_KEYS_ON_BOOT` pour imprimer
  les 3 clés attendues de la carte sur le série.

### Génération d'une clé côté fabricant (exemple Python)

```python
import hmac, hashlib
SECRET = b"...le meme que secrets.h..."
mac    = "AA:BB:CC:DD:EE:FF"   # HWID affiché dans l'UI
tier   = 2                     # 0/1/2
print(hmac.new(SECRET, f"{mac}:{tier}".encode(), hashlib.sha256).hexdigest().upper()[:16])
```

### ⚠️ Limite de sécurité

Le secret est dans le binaire. Sur une puce **sans Flash Encryption + Secure
Boot v2**, il reste extractible par dump de la flash SPI, ce qui casse le DRM.
En production : activer les deux via `idf.py menuconfig` / `platformio` (eFuses).
Sur la grille, **l'AP WiFi reste la frontière de confiance** : ne pas partager
le mot de passe.

---

## 7. OTA firmware

1. Se connecter (PIN → jeton).
2. **Moteur arrêté** (RPM = 0), sinon `/api/ota` renvoie 403.
3. UI : *Mise à jour firmware* → choisir le `.bin` → *Envoyer* (barre de progression).
4. Succès → reboot automatique sur la nouvelle version (`/api/state` → `fw`).
5. Rollback : re-flash filaire (USB) de la version précédente.

CLI :
```bash
curl -H "Authorization: Bearer <token>" -F "firmware=@firmware.bin" http://192.168.4.1/api/ota
```

---

## 8. Changements de comportement vs V22.1

| # | Avant | Après |
|---|---|---|
| 1 | Filtre EMI : rejet > 30000 tr/min (`delta < 2000 µs`) | Rejet > `RPM_MAX_PLAUSIBLE` (20000, conforme spec) + rejet des hausses hors slew (`RPM_SLEW_MAX`) ; delta mesuré depuis la **dernière impulsion validée** |
| 2 | Overrev sur RPM filtré (EWMA, en retard) | Overrev sur **RPM instantané** (`state::getRpmInstant()`) — aucun retard |
| 3 | Fenêtre quickshifter 25 ms non implémentée (appui avalé sous `minShiftRpm`) | Latch d'appui conservé sur 25 ms : la coupure part si le régime monte dans la fenêtre ; one-shot + lockout 250 ms |
| 4 | ISR roues sans anti-rebond | Anti-rebond optionnel `cfg::WHEEL_MIN_PULSE_US` (0 par défaut = comportement VR/Hall inchangé) |
| 5 | Pit / embrayage lus sans anti-rebond | Anti-rebond `cfg::INPUT_DEBOUNCE_MS` (15 ms) |
| 6 | 1ère fenêtre vitesse calculée sur un `dt` de plusieurs secondes | 1ère fenêtre ignorée (seed) |
| 7 | `filteredRate` static local, jamais remis à zéro | État de dérivée de glisse remis à zéro au calage moteur |
| 8 | Dispatch arbitre par heuristique `currentMode != requestedMode` | Priorité portée par `ignition::request()` : un mode ≥ (ré)arme et prolonge, jamais ne raccourcit ; un mode < est ignoré pendant une coupure prioritaire |
| 9 | Race ISR ↔ `loop()` (spinlock pris seulement côté `loop`) | ISR **et** lecteurs prennent le même spinlock (`portENTER_CRITICAL_ISR`) ; RPM/EMI/EWMA sortis de l'ISR |
| 10 | Reboot via `xTaskCreate` depuis un handler AsyncTCP | Latch `net::requestReboot()` relu dans `loop()` |
| 11 | `imuTask` vide | `supervisorTask` : surveille le heartbeat de `loop()`, force `ignition::forceClear()` + reboot si figé > 1 s |
| 12 | Vitesse roue = comptage d'impulsions / fenêtre 50 ms (quantifié, bruité à basse vitesse) | **Mesure de période** : `v` sur `(count-1)` intervalles exacts ; rejet des vitesses impossibles + limitation de pente. Signal de glisse bien plus propre → filtre de dérivée allégé (`SLIP_RATE_FILTER_A` 0.2 → 0.35), horizon d'avance réduit (50 → 30 ms) |
| 13 | Slip control : aucune notion d'angle | **Garde-fou anti-angle (IMU MPU6050)** : `leanAuthority()` — autorité pleine < 12°, dégressive jusqu'à 28°, nulle au-delà ; nulle aussi si `|accel| > 1,3 g` ; **nulle si l'IMU est absent/muet** (`IMU_REQUIRED=1`). Hystérésis de ré-armement à 8°. N'affecte QUE le slip control (launch = à l'arrêt, moto droite ; overrev/pit/shift = inchangés). |

Inchangé : priorités de l'arbitre, valeurs de carto par défaut, brochage des
E/S existantes, principe « CUT = allumage à la masse », fail-safe pull-down au boot.

> L'estimation d'angle gère le piège classique moto : en virage stabilisé
> l'accéléromètre pointe le long de l'axe de la moto (il « voit à plat »). Le
> filtre complémentaire ne recale donc le gyro sur l'accéléro que lorsque
> `|accel|` est proche de 1 g (moto peu sollicitée) ; sinon il fait confiance à
> l'intégration gyro (biais estimé au boot + suivi lent).

> Note : sur Arduino-ESP32 2.0.x, le task-watchdog de la tâche IDLE du Core 1
> n'est pas armé par défaut, donc la `loop()` serrée (sans `vTaskDelay`) reste
> valide ; le superviseur + le WDT de `loop()` couvrent le gel applicatif.

---

## 9. Vérification

```bash
# Compilation
pio run -e esp32dev            # ou : arduino-cli compile --fqbn esp32:esp32:esp32 .

# Contrôles statiques
grep -rn "12345678" src shifter.ino                     # -> vide
grep -rn "COMPANY_SECRET_SALT\|processor(" src          # -> vide
grep -rn "s_rpmLast\|s_rpmEdgeSeq" src                  # -> src/sensors/Sensors.cpp uniquement
```

Matrice de test sur cible : voir la section « Verification » du plan
(`snoopy-gliding-sparrow.md`) — générateur de signal RPM, tap sélecteur,
bouton pit, embrayage + basse vitesse, banc à rouleaux, auth WiFi, licence,
OTA moteur arrêté vs tournant, gel de `loop()`.
