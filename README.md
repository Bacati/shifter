# Aptatio ECU — 50cc 2 Temps (allumage PVL)

Calculateur de gestion moteur sur ESP32 pour moto de course 50cc 2T à allumage
autonome (type PVL) : **quickshifter, launch control, pit limiter, limiteur
d'overrev, contrôle de motricité (slip control) et télémétrie WiFi**.

Seule arme d'action : la **coupure d'allumage** (pas de Ride-by-Wire sur un
carburateur). Sur un PVL, « couper » = mettre le primaire à la masse via un
étage de puissance haute tension isolé de l'ESP32.

---

> ## ⚠️ STATUT : PROTOTYPE
>
> Firmware **v23.0** : compile proprement (Arduino-ESP32 2.0.x), logique validée
> en **simulation Wokwi uniquement**. **Jamais exécuté sur du matériel réel.**
>
> **Ne pas monter sur une moto** avant d'avoir déroulé toute la campagne d'essais
> au banc (§4) puis le test final progressif (§5). Un bug ici coupe l'allumage
> d'une moto en mouvement.

---

## 1. Ce que fait le logiciel

### 1.1 Les 5 assistances — arbitre à priorité stricte

À chaque tour de boucle, un **arbitre central** décide qui a le contrôle de
l'étincelle. Priorité décroissante :

| Prio | Assistance | Rôle | Licence |
|---|---|---|---|
| 1 | **Overrev limiter** | Coupe au-delà de `OVERREV_RPM` (défaut 16 000). Filet de survie moteur. Écrase tout. | toutes |
| 2 | **Pit limiter** | Bride le régime dans la voie des stands (bouton au guidon). | toutes |
| 3 | **Quickshifter** | Coupure nette (~50 ms) sur appui sélecteur : libère les crabots sans débrayer. Fenêtre d'analyse 25 ms, verrou anti double-coupure 250 ms. | toutes |
| 4 | **Launch control** | 2-step limiter au départ arrêté (embrayage tiré, moto ~immobile). Charge l'échappement sans emballer le moteur. | TRACK + |
| 5 | **Slip control** | Motricité **en ligne droite** : compare roue avant (libre) / arrière (motrice), coupe si la glisse dépasse `maxSlipPercent`. **Pas de TC sur l'angle** (voir 1.4). | PRO |

Une coupure programmée **va toujours à son terme** : un mode plus prioritaire
peut la prolonger, jamais la raccourcir ; un mode moins prioritaire est ignoré
tant qu'une coupure prioritaire est active.

### 1.2 Formules / licences

Un seul binaire, bridage **à l'exécution** par une clé liée au HWID (adresse MAC).

| Tier | Débloque |
|---|---|
| **BASIC** (par défaut, aucune clé) | Overrev + Pit + Quickshifter |
| **TRACK** | + Launch control |
| **PRO** | + Slip control |

`clé = HEX_MAJ( HMAC-SHA256( SECRET , "<MAC>:<tier>" ) )[0..15]`. Le `SECRET` est
dans `src/persistence/secrets.h` (gitignoré). Génération / provisioning : voir
[`tools/README.md`](tools/README.md).

### 1.3 Signal & robustesse

- **Vitesse roue par mesure de période** (intervalles inter-impulsions), pas par
  comptage sur fenêtre → signal de glisse propre même à vitesse modérée ; rejet
  des vitesses impossibles et limitation de pente.
- **Filtre EMI RPM** : rejette toute impulsion impliquant > `RPM_MAX_PLAUSIBLE`
  (20 000 tr/min) ou une hausse hors limite physique.
- **Overrev sur RPM instantané** (pas la valeur filtrée) → limiteur sans retard.
- **Dual-core** : Core 1 = sanctuaire calcul moteur (`loop()`), Core 0 = WiFi,
  télémétrie, IMU, superviseur.
- **Double watchdog** : WDT matériel sur `loop()` (3 s) **+** superviseur (Core 0)
  qui, si `loop()` se fige > 1 s, force la coupure OFF (allumage rendu) et
  redémarre.
- Synchro ISR ↔ tâches par spinlock ; scalaires partagés en `std::atomic`.

### 1.4 Garde-fou anti-angle (IMU MPU6050)

Une micro-coupure sur l'angle = retour d'adhérence brutal = **high-side**. Un
MPU6050 (I²C) estime l'angle de roulis (filtre complémentaire gyro + accéléro,
gérant le piège moto : en virage stabilisé l'accéléro « voit à plat »).

Le **slip control** voit son autorité :
- pleine sous 12°,
- dégressive de 12° à 28°,
- **nulle au-delà de 28°**,
- **nulle si `|accel| > 1,3 g`** (virage / freinage fort),
- **nulle si l'IMU est absent ou muet** (fail-safe, `IMU_REQUIRED=1`).

N'affecte **que** le slip control (launch = à l'arrêt moto droite ; overrev / pit
/ shifter = inchangés).

### 1.5 Interface web & OTA

L'ESP32 génère son propre AP WiFi **WPA2 avec mot de passe unique par appareil**
(généré au 1er boot, imprimé sur le port série). Page web embarquée :
- **Télémétrie live** (SSE) : RPM, vitesses, glisse, roulis, état IMU, mode arbitre, licence.
- **Cartographie** : temps de coupure, régimes limites, circonférence pneu,
  tolérance de glisse — clampés, sauvegardés en NVRAM.
- **Licence** : saisie de clé d'upgrade.
- **OTA** : mise à jour firmware (refusée moteur tournant).

Endpoints mutateurs (`/api/config`, `/api/wifi`, `/api/ota`) protégés par un
**jeton Bearer** échangé contre un **PIN** (imprimé sur série, verrouillage après
échecs). Détail dans [`BUILD.md`](BUILD.md).

---

## 2. Architecture

`shifter.ino` = `setup()` / `loop()` uniquement. Modules par domaine dans `src/`
(`core`, `sensors`, `actuator`, `control`, `persistence`, `net`, `sim`, `factory`).
Environnements PlatformIO : `esp32dev` (prod), `wokwi` (simu, WiFi coupé),
`factory` (provisioning). **Détail complet : [`BUILD.md`](BUILD.md).**

---

## 3. Ce qui manque / n'est pas fait

### 3.1 Bloquant avant tout usage sur moto

- **Aucun essai sur matériel réel.** Compilation + Wokwi ≠ « ça marche ».
- **`WiFi.macAddress()` est appelé avant `WiFi.begin()`** : vérifier sur la carte
  qu'il renvoie la vraie MAC eFuse et non `00:00:…` — sinon les licences ne sont
  pas liées à la carte.
- **Constantes de contrôle = valeurs par défaut** : toutes les durées de coupure,
  la tolérance de glisse, le régime launch, les seuils d'angle et le **mapping des
  axes de l'IMU** (`cfg::IMU_ACC_ROLL_*`, `IMU_ROLL_SIGN`) sont à régler sur la
  vraie moto.
- **OTA sans signature ni rollback** : une mauvaise image = carte à re-flasher au
  fil. Ajouter `esp_ota_mark_app_valid_cancel_rollback()` + idéalement une
  vérification de signature.
- **Coupe-circuit physique** indépendant de l'ECU : obligatoire (règlement +
  sécurité), à câbler et tester **avant** tout.

### 3.2 Durcissement sécurité restant

- **Flash Encryption + Secure Boot v2** non activés (eFuses) : sans eux, le
  `SECRET` s'extrait d'un dump flash → DRM contournable. Secure Boot v2 avec
  Arduino/PlatformIO est difficile (bootloader précompilé) → probable migration
  ESP-IDF. Voir [`tools/README.md`](tools/README.md) §4.
- **Pas de HTTPS** : le trafic HTTP est en clair pour qui a le mot de passe WiFi
  (l'air est chiffré WPA2). Acceptable paddock, pas « produit ».
- **Ancien secret dans l'historique git** (`COMPANY_SECRET_SALT`, commit V22.1) :
  ne pas réutiliser cette chaîne ; purge éventuelle via `git filter-repo`.

### 3.3 Risques au niveau du code (connus, non éliminés)

- `state::setLean` / `setSpeeds` / `setSlip` écrivent plusieurs atomiques
  séparément → un lecteur peut voir un jeu « à moitié à jour ». Toléré pour un
  garde-fou à hystérésis ; ajouter un seqlock si besoin de garantie stricte.
- La tâche IMU n'est pas surveillée par un watchdog : bus I²C bloqué → tâche figée
  (sûr : slip désactivé) mais morte jusqu'au reboot.
- Le superviseur fait `esp_restart()` depuis une tâche : collision improbable mais
  possible avec une écriture OTA en cours.
- **Aucun test unitaire**, aucune protection anti-régression.

---

## 4. Campagne d'essais AU BANC (avant la moto)

Ne pas sauter d'étape. À chaque phase : moteur coupé sauf mention contraire.

### 4.0 Matériel de banc

- ESP32 DevKit flashé en `esp32dev` (avec `src/persistence/secrets.h` renseigné),
  build `-DLICENSE_PRINT_KEYS_ON_BOOT` pour lire les clés.
- Générateur de signal (ou 2ᵉ MCU) pour injecter RPM et impulsions roues.
- 3 boutons poussoirs (shifter D26, embrayage D25, pit D33) vers GND.
- **Résistance de pull-down 10 kΩ sur D4 (CUT)** + LED/résistance ou oscilloscope sur D4.
- MPU6050 sur I²C (D21/D22) + pull-ups 4,7 kΩ.
- Multimètre / oscilloscope.

### 4.1 Boot & sécurité de base

1. **Pull-down CUT** : ESP32 non alimenté / en reset → D4 doit lire **LOW**
   (allumage rendu). Firmware au ralenti → D4 LOW. D4 ne passe HIGH que pendant
   une coupure réelle.
2. **Log série** : noter `[BOOT] HWID: …`. **Vérifier que ce n'est pas
   `00:00:00:00:00:00`.**
3. `[IMU] MPU6050 OK` (ou `absent` → alors le slip control sera désactivé, c'est
   le comportement attendu).
4. **WiFi** : rejoindre `AptatioECU-XXXXXX` avec le mot de passe imprimé. Ouvrir
   `http://192.168.4.1` → la page s'affiche.
5. **Auth** : `POST /api/config` refusé sans jeton. Login avec le PIN. Modifier
   une valeur → *Flasher NVRAM* → reboot → valeur conservée (NVS).
6. **Licence** : comparer les 3 clés imprimées au boot avec
   `python tools/make_key.py <MAC> --all` → identiques. Débloquer TRACK → reboot →
   `[LIC] tier 1`. Mauvaise clé ×8 → verrouillage.

### 4.2 Capteurs (signaux injectés)

7. **RPM** : carré sur D5, balayer la fréquence → la télémétrie RPM suit. À la
   fréquence = `OVERREV_RPM` → D4 pulse (oscillo). Vérifier en plus :
   - **Burst EMI dense** (parasites µs) superposé à un RPM valide → le RPM ne
     décroche pas, pas de coupure parasite.
   - **Bruit EMI seul, sans signal RPM valide** → après `RPM_STALL_US` le RPM
     retombe bien à **0** (pas de « moteur fantôme »).
   - **Impulsion manquée** (couper 1 impulsion sur N du générateur) → le RPM
     affiché ne se **fige pas** à la moitié : il replonge un instant puis
     retrouve la vraie valeur en 2–3 impulsions.
   - Accélération franche (rampe de fréquence rapide) → l'estimation suit sans
     être écrêtée (pente admise 120 000 tr/min/s).
8. **Vitesses roues** : carrés sur D14 / D27 à fréquences connues → vitesses
   télémétrées = `circonférence × f / pulsesPerRev`. Roue arrière plus rapide →
   `slip %` apparaît.
9. **Boutons** : tap SHIFTER → **une seule** impulsion CUT de `shiftCutMs`
   (mesurer au chrono/oscillo), jamais de double. PIT maintenu → strobe.
10. **IMU** : incliner le MPU6050 → `lean` télémétré suit. Au-delà de 28° (avec
    conditions de glisse injectées) → les coupures slip s'arrêtent. Débrancher
    l'IMU → `imu:false` → slip désactivé.

### 4.3 Assistances complètes (licence PRO, signaux injectés)

11. **Overrev** : la coupure engage pile au seuil, tient tant qu'on est au-dessus,
    relâche proprement.
12. **Quickshifter** : RPM > `minShiftRpm`, tap SHIFTER → coupure nette, verrou
    250 ms respecté. RPM < `minShiftRpm` → pas de coupure.
13. **Pit** : PIT maintenu + RPM > `pitRpm` → bridage ; en dessous → rien.
14. **Launch** : embrayage tiré + vitesse avant 0 + RPM ≈ `launchRpm` → la coupure
    module ; relâcher (> 5 km/h) → launch se désarme.
15. **Slip** : arrière plus rapide au-delà de la tolérance → coupures ; incliner
    l'IMU → autorité dégressive → coupures nulles > 28° ; secousse `|accel|`
    > 1,3 g → blocage dur.
16. **Priorité** : injecter deux conditions simultanées (overrev + shifter) →
    l'overrev gagne, la coupure n'est jamais raccourcie.
17. **Superviseur** : figer artificiellement `loop()` (build simu, commande
    `hang`) → D4 repasse LOW + redémarrage. Débrancher un capteur en cours → pas
    de crash.

### 4.4 Électronique de puissance + EMI (CRITIQUE)

18. Câbler l'étage d'isolation réel : D4 → optocoupleur → thyristor/MOSFET → un
    PVL sur établi avec bougie (**sans moteur**). Vérifier :
    - CUT HIGH met réellement le primaire à la masse (l'étincelle s'arrête).
    - CUT LOW → l'étincelle revient.
    - Les pics primaire ~400 V **n'atteignent pas** l'ESP32 (oscillo sur le 3,3 V
      et les GPIO pendant que le PVL crache).
    - Avec le PVL en fonctionnement : l'ESP32 garde un RPM propre, ne reset pas,
      ne rate pas d'allumage. **C'est ici que le filtre EMI se valide pour de
      vrai.**
19. **Thermique** : faire tourner le MOSFET/thyristor au duty attendu quelques
    minutes → vérifier le dissipateur.

### 4.5 Moteur au banc

20. Moteur tournant, moto sur béquille (roue arrière libre), RPM réel depuis le
    PVL. Vérifier :
    - Ralenti : aucune coupure.
    - Montée en régime : l'overrev accroche (**mettre `OVERREV_RPM` prudent au
      début**).
    - Sélecteur (roue arrière tournante) : coupure quickshifter, la boîte engage.
    - Launch : embrayage tiré, gaz en grand → le 2-step tient le régime.
    - Le **slip control** ne se teste pas vraiment sur béquille (roue avant à 0) →
      banc à rouleaux ou validation piste prudente.
21. **Régler ici toutes les constantes** (durées de coupure à l'oreille/au
    ressenti, marge d'overrev, régime launch, `minShiftRpm`, seuils IMU après
    calibration du mapping d'axes).

---

## 5. Test final SUR LA MOTO (progressif)

1. **Coupe-circuit physique** indépendant, câblé et testé **en premier**.
2. Zone fermée / privée, basse vitesse d'abord.
3. **Ligne droite uniquement** au début. **Slip control désactivé** pour les
   premiers runs (licence BASIC ou `slipMax` très haut) → valider overrev, pit,
   shifter, launch d'abord.
4. Puis activer le slip control, accélérations franches en ligne droite,
   progressivement.
5. **Ne pas « tester » le garde-fou anti-angle en se couchant fort** dès les
   premiers tours : faire confiance à la validation banc (§4.2 pt 10). C'est un
   limiteur de sécurité, pas une fonction à sonder à vitesse.
6. **Data-log** via la télémétrie à chaque run, relire entre les runs.
7. Plan « l'ECU fait n'importe quoi » : coupe-circuit ; un reset rend l'étincelle
   (pull-down).

---

## 6. En cas de problème

- Aucune étincelle / moteur qui broute → vérifier le pull-down D4, l'étage de
  puissance, le log série (`[SUPERVISOR]`, reboots).
- Pas de série sous Wokwi → voir `BUILD.md` §5 ; en simu, la LED CUT qui clignote
  seule 6× au boot = le firmware tourne.
- Licence refusée → MAC exacte (majuscules, `:`), secret identique à
  `secrets.h`, `tools/make_key.py`.

---

## 7. Voir aussi

- [`BUILD.md`](BUILD.md) — build, câblage, environnements, modèle de sécurité,
  changements vs V22.1, matrice de test simu.
- [`tools/README.md`](tools/README.md) — génération de clés, provisioning usine,
  Flash Encryption.

---

## Annexe — Concept (dossier technique d'origine, condensé)

**Pourquoi ce design.** Appliquer l'électronique d'une Superbike sur un 50cc 2T à
carburateur est un défi physique et électrique. Pas de Ride-by-Wire → la seule
arme est la coupure d'allumage. PVL autonome (sans batterie), tensions primaires
extrêmes → l'ESP32 (3,3 V) est isolé par optocoupleur pilotant un thyristor /
MOSFET SiC ; pour couper, on met le système **à la masse**. Bruit
électromagnétique colossal → filtre logiciel qui rejette toute impulsion
d'allumage physiquement impossible.

**Abandon volontaire du TC sur l'angle.** Sur un 2T à carbu, on ne peut pas
réduire la puissance en douceur. Des micro-coupures pendant que le pilote est sur
l'angle provoqueraient un retour d'adhérence violent (high-side). Le slip control
se concentre donc sur la **motricité en ligne droite** ; l'IMU sert de garde-fou
pour l'y contraindre.

**Fiabilité aérospatiale.** OS temps-réel (FreeRTOS), dual-core isolé, atomicité
et spinlocks sur les données partagées, accès direct aux registres GPIO pour la
coupure, double watchdog rendant l'allumage au pilote au moindre gel.
