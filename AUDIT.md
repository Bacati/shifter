# AUDIT firmware v23.x + banc de test + préparation CAN

Passes du 2026-10-18 sur tout `src/`, `shifter.ino`, `platformio.ini`, la doc.
**270 tests automatiques** exécutent le vrai code sur PC (voir §6). **Rien n'a tourné sur du matériel.**

---

## 1. Ce qui a été trouvé et corrigé

Chaque ligne « prouvé » = un test qui échoue sur l'ancien code (ou sur une mutation qui le recrée) et passe maintenant.

### Sécurité de pilotage (les plus graves)

| # | Problème | Correctif | Prouvé par |
|---|---|---|---|
| 1 | **Le garde-fou anti-angle devient aveugle en virage.** L'accéléro « voit à plat » et ramenait l'angle estimé vers 0° (0,0° après 3 s pour 10–25° réels) → slip control à pleine autorité couché = risque high-side. | Recalage accéléro interdit si lacet > 3 °/s. | `imu_virage_stabilise_*`, chaos 20 min (mutant : slip à 36,7° avec « −3,8° » estimé) |
| 2 | **Angle faux après redémarrage / ré-init de l'IMU en virage** : le gyro intègre la remise à plat depuis 0 → l'estimation valait −angle, et était déclarée fiable dès le 1er échantillon « moto droite ». | `ok` seulement après convergence soutenue (score), recalage accéléro filtré. | `imu_reboot_dans_un_virage…`, chaos |
| 3 | **Biais du gyro mesuré en roulant** (zéro-rate MPU6050 jusqu'à ±20 °/s) : un redémarrage en mouvement fabriquait un biais faux → dérive de plusieurs dizaines de degrés (erreur mesurée 67°). | Biais mesuré **seulement roues arrêtées** et signal calme ; sinon IMU non fiable jusqu'au prochain arrêt. | `imu_trajet_long_*` (67° → 5,8–7°), `full_reboot_en_roulant…` |
| 4 | IMU endormi (brownout) ou figé : lectures nulles jugées valides ; jamais de ré-init (« morte jusqu'au reboot »). | Détection (|acc| < 0,5 g, 10 trames identiques), ré-init auto toutes les 500 ms, tâche lancée même IMU absente au boot. | `imu_capteur_endormi…`, `imu_trame_figee…`, `imu_bus_bloque…`, `imu_absent_au_boot…` |
| 5 | **Quickshifter déclenché par un parasite EMI** de quelques µs (ISR sur front) : 97 coupures de 50 ms en 57 s avec 2 parasites/s. | Confirmation : la broche doit être basse ≥ 2 ms ; un rebond de contact reste accepté (latch gardé jusqu'à la fin de la fenêtre 25 ms). | `full_parasites_shifter…`, `shift_parasite…`, `full_shifter_contact_rebondissant…` |
| 6 | **Fausse glisse → coupures de motricité** : (a) une impulsion parasite roue arrière → 63 coupures/116 s ; (b) capteur roue avant débranché : la vitesse avant estimée décroît → fausse glisse. | Glisse = minimum des 4 dernières fenêtres de 40 ms ; roue avant déclarée « vivante » seulement si impulsion récente. | `full_parasites_roues…` (63 → 0), `full_capteur_roue_avant_debranche…` |
| 7 | **Launch armé roue avant seule < 5 km/h** (capteur avant mort = launch actif en roulant). | Les deux roues < 5 km/h. | `launch_capteur_avant_mort…` |
| 8 | **Launch et pit limiter ne limitaient pas** : modulation ≤ 40 % de coupure < gain moteur à vide (modèle : launch 11 300–11 800 tr/min pour une cible de 8000). | Au-delà de cible + 600 tr/min (launch) / pitRpm + 400 (pit) : coupure continue. | `full_launch_boucle_fermee…` (8038–9004), `full_pit_limiter…` |
| 9 | **Priorité du pit limiter** contraire au README (il était le plus faible). | Ordre aligné : Overrev > **Pit** > Shift > Launch > Slip (décision utilisateur). | `cut_ordre_de_priorite_complet`, `pit_prend_le_dessus…` |
| 9b | **Pit limiter en km/h** (demande utilisateur) : le limiteur ne dépendait que du régime, donc la vitesse réelle variait avec le rapport. | Nouveau réglage `pitKmh` (défaut 60, 10–100, page web + NVS v2 avec migration), vitesse = roue la plus rapide, calculée avec la circonférence/impulsions **déjà configurées** (pas de nouveau paramètre roue). Repli sur `pitRpm` si aucune roue ne donne de vitesse (capteurs HS, moto à l'arrêt qui emballe). | `pit_kmh_*` (6), `web_config_pitKmh…`, `config_migration_blob_v1…`, `full_pit_limiter_kmh_*` |

### Robustesse

| # | Problème | Correctif | Prouvé par |
|---|---|---|---|
| 10 | `async_tcp` (prio 10) pouvait tourner sur le Core 1 et préempter `loop()`. | `-DCONFIG_ASYNC_TCP_RUNNING_CORE=0` (présent dans la ligne de compilation). | `pio run -v` |
| 11 | Écriture NVS (config/licence/mot de passe) moteur en marche : le cache flash est coupé sur les 2 cœurs, `loop()` figé, coupure non relâchée. | `409` si moteur en marche. | `web_config_refusee_moteur…`, `web_licence_refusee…`, `web_wifi_rotation…` |
| 12 | `/api/config` : champ vide / texte → clampé au minimum **en silence** et enregistré. | Parsing strict + bornes, `400`, rien enregistré (même si un autre champ était valide). | `web_config_valeur_invalide…`, `parse_int_strict` |
| 13 | **Config NVS non atomique** : 6 coupures de courant sur 8 laissaient une carto MIXTE. | Un seul blob versionné + CRC32 (écriture NVS atomique) ; migration depuis v23.0 ; CRC invalide → anciennes clés / défauts. | `config_coupure_courant…` (6/8 → 0/8), `config_blob_corrompu…`, `config_migration…` |
| 14 | Sonde eau : 85,0 °C d'usine accepté, 1 échec CRC = « absent », double attente (cycle 2,3 s), fraîcheur jamais utilisée, re-scan du bus à chaque lecture. | 85,0 rejeté sauf continuité, 3 échecs tolérés, conversion non bloquante, adresse mémorisée, `ok` inclut la fraîcheur. | `test_watertemp` (10/11 → 11/11, mutation sur l'ancien code) |
| 15 | RPM : « moteur fantôme » avec du bruit (rien ne distinguait un parasite d'une impulsion). | Verrouillage : 5 périodes cohérentes (±30 %) et ≤ 16 000 tr/min avant de publier ; déverrouillage après 6 périodes incohérentes. | `rpm_bruit_aleatoire_clairseme…`, `rpm_emi_seul…` |
| 16 | Vitesse roue : une roue lente (< 5,2 km/h avec 3 imp/tour ; < 15 km/h avec 1 imp/tour) lue **0**. | Délai d'arrêt = max(400 ms, durée pour 2 km/h). | `roues_basse_vitesse_5kmh` |
| 17 | Aucune borne absolue sur une coupure ; `maxSlipPercent` pouvait valoir 0–2. | Plafond 200 ms ; `SLIP_MIN = 5`. | `cut_plafond_absolu_200ms`, `config_bornes_coherentes…` |
| 18 | `src_dir = .` aurait compilé `test/` dans le firmware. | `build_src_filter` exclut `test/` et `tools/`. | `pio run` |
| 19 | Rien n'empêchait deux fonctions de partager une broche (carte commune). | `static_assert` d'unicité + broches CAN réservées (GPIO18/19). | test négatif/positif |
| 20 | **Licence perdue sur coupure de courant pendant l'activation** : clé et tier étaient 2 items NVS écrits l'un après l'autre. Coupure entre les deux (TRACK→PRO) = clé PRO + tier TRACK → la revalidation échoue → **client rétrogradé en BASIC** (même plus TRACK). | Un seul item NVS `lic` = `"<tier>:<clé>"` (écriture atomique). Anciens items lus seulement si `lic` absent (cartes déjà déployées). | `licsec_coupure_pendant_upgrade…` (échoue sur l'ancien code : niveau relu 0), `licsec_activation_ecrit_un_seul_item_nvs`, mutation vérifiée |
| 21 | **Re-saisie d'une ancienne clé inférieure = rétrogradation** (clé TRACK valide saisie sur une carte PRO → écrasait le PRO). | Une clé valide de niveau inférieur est acceptée sans rien changer (retour arrière = `WIPE` usine). | `licsec_ancienne_cle_inferieure_ne_retrograde_pas`, mutation vérifiée |

## 2. Valeurs mesurées à valider sur le vrai moteur (réglages, pas des bugs)

| Mesure (modèle de banc) | Valeur | À faire |
|---|---|---|
| Réaction slip : perte d'adhérence → 1re coupure | **~233 ms** (4 fenêtres × 40 ms + arbitre) | C'est le prix de l'immunité aux parasites roue. `SLIP_CONFIRM_WINDOWS` (3 = ~190 ms mais ~7 coupures parasites/116 s à 1 parasite/s). |
| Pit limiter km/h | +1,7 km/h avec ~40 km/h/s d'accélération ; +10,7 km/h avec un modèle extrême (160 km/h/s à vide) | Le dépassement vient de la latence de mesure roue (~40–80 ms) × l'accélération ; ajuster `PIT_HARD_BAND_KMH` (3) selon la moto. |
| Launch tenu | cible +0 / +1000 tr/min | Ajuster `LAUNCH_HARD_BAND_RPM`. |
| Glisse parasite à 2 % de jitter capteur | 10 % (+4 % prédictif < tolérance 15 %) | Marge mince : mesurer le jitter réel des capteurs, relever la tolérance si besoin. |
| RPM sous EMI dense aléatoire | ±25 % de bruit de mesure, jamais d'overrev | Filtre matériel (Schmitt + RC) indispensable. |
| Bruit **continu** > 300 impulsions/s après l'arrêt moteur | indiscernable d'un moteur à ~19 000 tr/min → overrev | Aucun filtre logiciel possible : blindage + Schmitt trigger. |
| Parasite périodique fixe à +3,1 ms de chaque vraie impulsion | RPM lu 0 (jamais d'overrev) | Idem. |

## 3. Points ouverts — décisions à prendre

1. ~~Pit limiter en régime ou en km/h~~ → **fait (km/h)**, voir 9b. À valider : la circonférence de roue et le nombre d'impulsions/tour doivent être justes dans la page web (le limiteur en dépend).
2. **Version** : `Version.h` = `23.0`, README `v23.0`, commits `v23.1`. Ce travail change le comportement de sécurité → numéro à décider (24.0 ?).
3. **Identité de la carte = `WiFi.macAddress()`** : fonctionne, mais couple la licence à l'état du WiFi. Préférer `esp_read_mac` (carte commune : les nœuds sans WiFi auront aussi un ID).
4. **`/api/license` sans authentification** + verrou global (n'importe qui sur l'AP peut bloquer la saisie 5 min). Acceptable paddock.
5. **Télémétrie incomplète** : pas de vitesse arrière, RPM instantané, état de coupure, cause du dernier reset, compteur de reboots superviseur.
6. Déjà connus (README §3) : OTA sans signature/rollback, pas de HTTPS, Secure Boot / Flash Encryption absents, ancien secret dans l'historique git.

## 4. Ce que le banc PC ne peut PAS prouver

- Courses ISR ↔ tâches (les sections critiques sont des no-op sur PC).
- Durée réelle de `loop()` sur l'ESP32 (le banc suppose 100 µs).
- Gel de `loop()` pendant une écriture flash (NVS/OTA) et gain de gigue après déplacement d'`async_tcp`.
- EMI réel, étage de puissance, optocoupleur, thyristor.
- Axes / signe réels de l'IMU, bruit réel des capteurs, tous les seuils de réglage.
- Le transport HTTP réel (AsyncTCP) : seule la logique des gestionnaires est testée.
- Wokwi : non exécutable ici (pas de jeton). Le scénario `test/host/test_fw_sim.cpp` rejoue le même firmware (mêmes options que l'env `wokwi`) ; `test/wokwi/scenario.test.yaml` est fourni **non vérifié**.

## 5. CAN — carte mère commune (réponses reçues : sonde de **culasse**, l'ECU **a une batterie**)

### 5.1 Conséquence importante du choix « culasse »

Un DS18B20 s'arrête à **125 °C** ; une culasse de 2T dépasse couramment 150–250 °C. Il faut une autre technologie :

| Option | Plage | Remarque |
|---|---|---|
| Thermocouple K + MAX31855/MAX6675 (SPI numérique) | −200…+1350 °C | Signal de quelques µV en entrée : **à ne pas câbler en analogique sur un long fil près du PVL**. |
| Rondelle de bougie NTC/PT1000 + ADC | ~−40…250 °C | Analogique, sensible au bruit comme une thermistance (cf. choix DS18B20). |

→ **Recommandation : nœud capteur déporté près de la culasse (MAX31855 en SPI sur place) qui émet en CAN.** Le signal µV ne parcourt que quelques cm, le bus différentiel (avec CRC matériel) traverse la zone bruyante — même logique que le DS18B20 vs thermistance. Sur la carte commune : même PCB, rôle sonde = firmware différent. Si le BE préfère la sonde sur l'ECU : MAX31855 sur la carte ECU, thermocouple blindé torsadé, capot métallique ; à valider à l'oscilloscope.

L'ECU ayant une batterie, la contrainte « alimentation » disparaît (protections load-dump/inversion à prévoir sur le 12 V).

### 5.2 Principes (inchangés)

- Le CAN n'est jamais dans le chemin critique : tâche Core 0 → `SharedState` (même modèle que `setWaterTemp()`, avec fraîcheur déjà intégrée).
- Aucun nœud distant ne peut commander une coupure ; donnée absente = **invalide**, jamais « dernière valeur ».
- La température ne doit **agir** (limiteur/avertissement) qu'après définition du sens du défaut (capteur perdu = brider ou ignorer ?).

### 5.3 Matériel (à valider par le BE)

| Sujet | Proposition |
|---|---|
| Contrôleur | TWAI intégré (pas de MCP2515). **TX = GPIO18, RX = GPIO19** réservés (hors strapping 0/2/5/12/15, hors PSRAM 16/17). |
| Transceiver | 3,3 V automobile : TJA1051T/3, TCAN1042, MCP2562FD. |
| Protection | TVS CAN (PESD1CAN/2CAN), self de mode commun, ESD connecteur. |
| Terminaison | 120 Ω aux 2 extrémités seulement : pad/jumper, non monté par défaut. |
| Rôle du nœud | Strap 3 bits (GPIO34/35/36, entrées seules) + `-DNODE_ROLE`. |
| Débit | 500 kbit/s, ID 11 bits, charge < 40 % (250 kbit/s si harnais long/bruité). |
| Câblage | Paire torsadée blindée, 4 fils (CANH, CANL, +V, GND), connecteur étanche, masse en un point. |

### 5.4 Plan de trames (proposition, IDs bas = priorité haute)

| ID | Nom | Émetteur | Période | Contenu |
|---|---|---|---|---|
| 0x100 | ECU_ENGINE | ECU | 20 ms | rpm u16, mode u8, flags u8, compteur u4 + CRC8 |
| 0x101 | ECU_WHEELS | ECU | 40 ms | avant u16 (0,1 km/h), arrière u16, glisse i16 (0,1 %) |
| 0x102 | ECU_LEAN | ECU | 20 ms | roulis i16 (0,1°), \|accel\| u16 (0,01 g), flags (fiable, biais valide) |
| 0x200 | TEMP_HEAD | sonde | 100 ms | T culasse i16 (0,1 °C), statut u8 (OK / absent / hors plage / défaut thermocouple), compteur + CRC8 |
| 0x201 | TEMP_AUX | sonde | 500 ms | eau / huile / échappement / air (4 × i16) + masque de validité |
| 0x7E0+id | NODE_HEARTBEAT | tous | 1 s | rôle, version FW, uptime, cause reset, défauts |
| 0x7F0/1 | DIAG_REQ / RSP | outil | à la demande | lecture/écriture carto, version, journal |

Un DBC unique versionné (`docs/ecu.dbc`) fera foi pour ECU, dash et data-logger.

### 5.5 Logiciel (`src/can/`, à écrire après validation du BE)

Tâche Core 0 (`driver/twai.h`, IDF 4.4, alertes BUS_OFF → reprise auto, file TX bornée) ; producteurs lisent `state::*` ; consommateurs écrivent `SharedState` avec horodatage + champ `source` ; banc : `TWAI_MODE_NO_ACK`, premier branchement en `LISTEN_ONLY` ; le banc PC de `test/host` recevra un faux bus (injection de trames, nœud muet, bus-off, trames corrompues/CRC).

### 5.6 Questions restantes pour le bureau d'étude

1. Sonde de culasse sur l'ECU ou nœud déporté ? (recommandation : nœud déporté, §5.1)
2. Technologie : thermocouple K (+ MAX31855) ou rondelle de bougie ? Plage visée ?
3. Liste des nœuds (ECU, sonde, dash, logger, GPS…) et qui est maître de la configuration.
4. Débit, longueur de harnais, connecteur.
5. La température doit-elle agir un jour ? Sens du défaut ?
6. Le DS18B20 « eau » reste-t-il (sur l'ECU) en plus de la culasse ?

## 6. Banc de test (`test/host`)

`python test/host/run_tests.py` (voir `test/host/README.md`) : compile le **vrai** `src/` contre un faux Arduino/FreeRTOS (horloge virtuelle, ordonnanceur coopératif des tâches, MPU6050 / DS18B20 / NVS / serveur HTTP simulés, **tas instrumenté**) avec Zig (`pip install ziglang`), un processus par test.

| Fichier | Tests | Couvre |
|---|---|---|
| `test_sensors` | 25 | RPM (précision, jitter, EMI, calage, verrouillage, débordement `micros`), roues, glisse, anti-rebond |
| `test_arbiter` | 52 | coupure, priorités, overrev/pit (km/h)/shift/launch/slip, hystérésis, **matrice de licences**, fuzz 120 s |
| `test_imu` | 21 | virages 10–35°, chicane, bascule rapide, redémarrage en virage, biais, panne/ré-init, 10 min de trajet |
| `test_watertemp` | 11 | 85 °C d'usine, échecs, débranchement, fraîcheur |
| `test_supervisor` | 8 | gel de `loop()`, reboot, JSON SSE |
| `test_persist` | 26 | bornes, NVS, coupure de courant, migration v1→v2, licence (clés = `tools/make_key.py`), réseau |
| `test_web` | 28 | auth/PIN/jeton/TTL, config, licence, wifi, OTA, calculateur BASIC |
| `test_fw_sim` | 11 | `shifter.ino` en mode Wokwi |
| `test_fw_full` | 31 | firmware de prod en boucle fermée, pannes, parasites, endurance, **chaos 20 min × 16 graines : 0 violation** |
| `test_stress` | 24 | **moto complète** (boîte 6 rapports, embrayage, couple, patinage, wheelie, virages) pilotée par un pilote virtuel — voir §6.1 |
| `test_stress_web` | 1 | 40 000 requêtes HTTP variées : 0 octet de fuite |
| `test_license_security` | 33 | **batterie abonnements / sécurité** — voir §6.4 |

### 6.1 Stress (`test_stress.cpp`)

Chaque test vérifie, à **chaque début de coupure**, des invariants avec la vérité physique du modèle (pas l'estimation de l'ECU) : slip jamais couché à plus de 35° réels ni sans IMU fiable ni hors licence PRO ; launch seulement embrayage tiré, roues < 5 km/h et licence TRACK+ ; shift jamais sous le régime mini ; overrev seulement au-dessus du seuil ; pit seulement bouton appuyé. Et, sur toute la durée : **chaque appui valide au sélecteur produit une coupure, aucun appui sous le régime mini n'en produit**, pas de redémarrage, tâches vivantes, régime moteur max < seuil + 2000, tas stable.

| Scénario | Volume | Résultat |
|---|---|---|
| Courses BASIC / TRACK / PRO, épisodes étranges mêlés (baisse d'adhérence, wheelie, double appui, passage raté, appui bref, coups d'embrayage, lâchers brutaux, calage, pit en accélération, jusqu'à 30° d'angle) | 3 × 150 cycles (~1650 passages) | 0 violation, **0 appui valide manqué, 0 coupure indue** |
| 1000+ passages de vitesse au sélecteur (PRO) | 215 cycles, **1080 passages** | idem |
| Rétrogradages avec embrayage + blip, et sans embrayage (sélecteur) | 260 cycles, **1310 rétrogradages** (462 sans embrayage) | idem |
| Départs BASIC / TRACK / PRO (adhérence 0,4–1,1, relâché 40–900 ms) | 3 × 300 départs | BASIC : moteur libre (15 900 tr/min) ; TRACK/PRO : régime tenu à ≤ cible + 1000 dans **100 %** des départs |
| Patinage (baisse d'adhérence à chaque cycle) | 2 × 250 cycles | **PRO réduit le patinage à 44 %** de celui du BASIC |
| Patinage en virage profond (jusqu'à 46°) | 220 cycles | 0 coupure de motricité couché > 35° ; mutation « sans garde-fou de lacet » détectée |
| Sélecteur maintenu 10 s / rafale de 2000 appuis / 1500 appuis à l'arrêt en neutre | — | 1 coupure par appui maintenu ; verrou 250 ms respecté ; coupure au-dessus de 6000 tr/min seulement |
| Capteur RPM muet 50–400 ms plein gaz | 100 cycles | limiteur tient (max 17 200 tr/min) |
| Calage + redémarrage | 300 cycles, 146 calages | rpm → 0, relance propre |
| Vitesse oscillant autour de 5 km/h (seuils launch/slip) | 600 s | 0 violation |
| Entrées simultanées embrayage + pit + sélecteur | 80 cycles | 0 violation |
| Débordement `micros()` (35 min) et `millis()` (50 min) | 90 min simulées | 0 violation |
| 400 configurations extrêmes appliquées à chaud | 406 cycles | 0 violation |
| Changements de licence en roulant (clés valides et invalides) | 128 tentatives | 0 violation ; pas d'écriture NVS superflue |
| **Fuite mémoire** | **3 h de course simulée** (910 cycles, 3249 passages, 21 763 coupures) + 40 000 requêtes web | **+0 octet, +0 allocation** |

Contrôle de la puissance des tests (mutations) : retirer le garde-fou de lacet → le test « patinage en virage profond » et le chaos échouent ; retirer la confirmation du sélecteur → 97 coupures parasites détectées ; remettre l'ancienne `WaterTemp.cpp` → 2 tests échouent.

### 6.2 Corrigé grâce au stress

| Constat | Correctif |
|---|---|
| Launch : le régime dépassait la cible de ~1000 tr/min en moyenne (50 % des départs hors tolérance) | `LAUNCH_HARD_BAND_RPM` 600 → **300** : 100 % des 300 départs sous cible + 1000 |
| (artefacts de modèle corrigés dans le banc, pas dans le firmware : embrayage à glissement, dérive du roulis calculée à vitesse figée) | — |

## 6.3 Limites connues révélées par les tests

- Un capteur RPM muet **plus de 500 ms** plein gaz ne laisse plus aucun limiteur actif (régime remis à 0 par le calage) : panne matérielle à traiter par la redondance / le coupe-circuit physique.
- Le slip control réagit en ~233 ms (confirmation sur 4 fenêtres) : prix de l'immunité aux parasites roue.
- Un client authentifié peut faire écrire la NVS en boucle (`/api/wifi`, `/api/config`) : usure flash possible, pas de limitation de débit.

## 6.4 Batterie abonnements / sécurité (`test_license_security.cpp` + `check_security.py`)

| Famille | Ce qui est prouvé |
|---|---|
| Persistance | coupure de courant à **chaque** écriture NVS pour BASIC→TRACK, BASIC→PRO, TRACK→PRO : jamais un niveau inférieur à l'ancien ; 1 seul item NVS par activation ; 1000 redémarrages PRO/TRACK : niveau gardé, **0 écriture NVS** au boot ; coupures pendant config/WiFi ne touchent pas la licence ; MAC fausse au boot (WiFi pas prêt) → BASIC pour ce boot **sans rien détruire**, PRO revient au boot suivant |
| Inactivité | PRO inchangé après 1 j, 30 j, 49,7 j (débordement `millis`), 400 j, 10 ans ; horloge à 0 / 2³¹ / 2³²−5 / 2⁴⁰ : aucun effet (**aucune expiration dans le code**) ; parcours client BASIC→TRACK→(4 mois)→PRO→(400 j)→50 redémarrages |
| Pas de triche | NVS falsifiée (15 variantes : tier ≠ clé, hors plage, tronquée, 4 Ko…) ; NVS d'une carte PRO copiée sur 300 autres cartes → BASIC ; clé de chaque carte × tier refusée sur 39 autres cartes ; 0 collision de clés ; chiffres hex uniformes, avalanche 50 % ; toutes les mutations d'une bonne clé (≈ 290), troncatures, séparateurs, NUL, UTF-8, 70 Ko : refusées ; **300 000 clés aléatoires : 0 succès**, 0 fuite mémoire |
| Anti brute-force | plafond mesuré 2 304 essais/jour (verrou 5 min après 8 échecs), 30 jours d'acharnement respectés ; ≈ 10¹² ans pour 50 % de chances en ligne ; verrou correct au débordement de `millis` ; bonne clé refusée pendant le verrou ; échecs n'écrivent rien |
| Web | parcours BASIC→TRACK→PRO avec droits exacts à chaque étape (launch = TRACK, slip = PRO) y compris 6 mois plus tard ; un refus 403 ne modifie ni config ni NVS ; champs injectés (`tier`, `licenseTier`…) sans effet ; aucune réponse HTTP ni la liaison série ne contient le secret, les clés, le PIN ; aucun mode simulation en production |
| Config (`check_security.py`) | secret jamais commité, non suivi par git, absent de l'historique, non recopié ; l'env `esp32dev` n'a aucun drapeau de test (`SIM_TIER`, `FACTORY_TOOLS`, `LICENSE_PRINT_KEYS…`) ; **binaire de production** sans console usine / `WIPE` / mode sim |

Contrôle de la puissance (mutations) : remettre l'écriture en 2 items → 4 tests échouent ; retirer la protection anti-baisse → 1 test échoue.

### Limites de sécurité NON couvertes (inchangées, à connaître)

- **Pas d'abonnement à durée** : la licence est perpétuelle, liée à la carte (README outils §1 « choix produit »). Rien n'expire, rien ne se renouvelle ; si le modèle économique doit être un vrai abonnement (échéance, renouvellement), il n'existe pas.
- **« Usage unique » = liée à UNE carte** (MAC) : la clé n'est pas consommée ; la re-saisir sur la même carte est possible (SAV). Elle ne marche sur aucune autre carte.
- **Secret HMAC dans le binaire** : tant que Flash Encryption + Secure Boot ne sont pas activés, un dump flash (ou un firmware modifié) contourne tout. C'est LE risque réel ; le brute-force en ligne est infaisable.
- Secret actuel : 28 caractères, ~89 bits estimés → correct mais en dessous de la recommandation (32 car. aléatoires). Le changer **invalide toutes les clés déjà vendues**.
- Compteur de verrou en RAM (remis à zéro par un redémarrage = accès physique) ; `/api/license` sans authentification (déni de service possible : bloquer la saisie 5 min).
- Échec d'écriture NVS (mémoire pleine/usée) non détecté : l'activation répond OK puis la carte retombe en BASIC au redémarrage.
- Clé saisie en HTTP clair sur le point d'accès (WPA2 seulement) : une clé interceptée ne sert que pour sa propre carte.

## 7. Fichiers modifiés

`platformio.ini`, `src/core/{Config.h,Types.h,SharedState.cpp}`, `src/sensors/{Imu.cpp,Sensors.cpp,Sensors.h,WaterTemp.cpp}`, `src/control/Arbiter.cpp`, `src/actuator/IgnitionCut.cpp`, `src/persistence/Storage.cpp`, `src/net/WebServer.cpp`, `src/net/ParamParse.h` (nouveau), `README.md`, `AUDIT.md`, `SUIVI.md`, `test/` (nouveau, dont `test_license_security.cpp` et `check_security.py`).
Les trois environnements (`esp32dev`, `wokwi`, `factory`) compilent ; flash 65,9 %.
