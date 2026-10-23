# CONTEXT.md - shifter (TTP ECU 50cc 2T), résumé pour démarrer vite

Mis à jour le 2026-10-23 (HEAD e3d4998). À RÉÉCRIRE, pas à compléter, quand l'état change.
Détails: `SUIVI.md` (état + prochaine étape), `AUDIT.md` (risques, réglages), `README.md`, `BUILD.md`, `tools/README.md`.

## Statut
Prototype ESP32. Compile (3 envs PlatformIO). Jamais testé sur matériel. 273 tests, PC seulement (`test/host/`).
Version: `src/core/Version.h` dit 23.0, le dernier commit s'appelle « v24 » (à trancher).

## Licences (pas d'abonnement)
3 tiers à vie, liés à la MAC de la carte, rien n'expire ni ne se renouvelle.
- BASIC (sans clé): Overrev, Pit limiter, Quickshifter.
- TRACK: + Launch control.
- PRO: + Slip control + télémétrie SSE.
- Clé = HMAC-SHA256(secret, "MAC:tier") tronquée à 16 hex. Génération `tools/make_key.py`, secret dans `src/persistence/secrets.h` (gitignoré).
- Activation: page web « Licence » (`POST /api/license`, verrou 5 min après 8 échecs) ou provisioning usine (`tools/provision.py`, `tools/license_gui.py`).
- Stockage NVS item `lic`; pas de baisse de tier; 403 sur `launchRpm` sans TRACK et `maxSlipPercent` sans PRO.
- Aucun prix, aucun paiement/backend dans le repo.

## Capteurs (GPIO)
RPM 5 | roue AV 14 | roue AR 27 | sélecteur shifter 26 | embrayage Hall 25 | bouton pit 33 | IMU MPU6050 I2C 21/22 | DS18B20 eau 1-Wire 32 (informatif) | sortie CUT 4.
CAN / sonde de culasse (GPIO 18/19 réservés): PAS fait, cahier des charges seulement (`AUDIT.md` §5.6).

## Fonctions
- Arbitre à priorité: Overrev > Pit limiter > Quickshifter > Launch (TRACK) > Slip (PRO).
- Garde-fou d'angle IMU (plein <12°, nul >28°, nul si >1,3 g ou IMU absent).
- Double watchdog (3 s + superviseur 400 ms).
- Web embarqué: AP WPA2, cartographie en NVS avec CRC, licence, rotation mot de passe WiFi, OTA (sans signature ni rollback), PIN + jeton Bearer, pas de HTTPS.
- Télémétrie live SSE `/events` (PRO): rpm, speed, slip, mode, tier, lean, imu, waterTemp, waterOk. Pas d'enregistrement (« data-log » annoncé mais non fait).
- Simulateur Wokwi: compile, jamais exécuté.

## Reste à faire
1. Relire `git diff`, fixer le numéro de version, vérifier circonférence de roue et impulsions/tour.
2. Carte réelle déjà en TRACK/PRO (v23.x): flasher, vérifier `[LIC] tier n` (migration), tester un tier supérieur.
3. Banc réel: valider `AUDIT.md` §2, répondre au bureau d'études, puis écrire `src/can/`.
Décisions: licence à vie vs vrai abonnement; secret HMAC 32 car. (28 aujourd'hui) AVANT la première vente; Flash Encryption + Secure Boot v2; `esp_read_mac` au lieu de `WiFi.macAddress()`.
Bloquants moto: aucun essai matériel, constantes à régler, OTA non signé, coupe-circuit physique indépendant à câbler d'abord.
Risques: slip ~233 ms; RPM muet >500 ms = limiteur inactif; EMI >300 imp/s; `/api/license` sans auth (DoS 5 min); usure NVS.
Doc incohérente: version (23.0 vs v24), nb de tests (237/270/273), `SUIVI.md` dit « rien commité » (faux), flash 63 % vs 65,9 %.
