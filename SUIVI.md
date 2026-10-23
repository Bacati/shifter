# SUIVI — état au 2026-10-23

**État** : HEAD `e3d4998` (20/10, « rename + test + correction telemetry »), précédé de `93314b2` (19/10, « v24 : sécurisation, garde-fou, tests, abonnement »).
Arbre propre avant cette passe. Les 3 environnements PlatformIO compilent (`esp32dev`, `wokwi`, `factory`) ; suite PC 273/273 au dernier run connu
(20/10, non relancée le 23/10). Détail : `AUDIT.md` (§1 #20-21, §6.4). Résumé court : `CONTEXT.md`.
Commandes : `python test/host/run_tests.py` (tout) · `python test/host/check_security.py [--elf]` (config sécurité, après `pio run -e esp32dev`).

**Décisions prises le 2026-10-23 (utilisateur) — à implémenter**
1. Licence **à vie** (liée à la MAC), pas d'abonnement à durée.
2. Secret HMAC passé à **32 caractères** (aucune clé vendue jusqu'ici, donc sans impact).
3. **BASIC = quickshifter seul** ; **pit limiter à partir de TRACK**.
4. **Data log sur carte SD** (aujourd'hui seul le flux live SSE PRO existe).
5. **Revoir les versions** : `Version.h` = 23.0 alors que les commits s'appellent v24 ; README/BUILD/AUDIT à aligner.

**Jamais fait** : aucun essai matériel (ni affichage réel de la page verrouillée sur navigateur), Wokwi non exécuté (pas de jeton), aucun code CAN
(spec + GPIO18/19 réservées), aucun test sur carte réelle de l'écriture `lic` (migration d'une carte déjà licenciée en v23.x → à essayer une fois).

**Reste à décider**
- Flash Encryption + Secure Boot (seule vraie protection contre le dump flash) — `tools/README.md` §4.
- Identité de carte via `esp_read_mac` plutôt que `WiFi.macAddress()` (AUDIT §3.3).

**Prochaine étape exacte**
1. Implémenter les décisions ci-dessus (tiers, secret 32 car., versions, data log SD), relancer les tests PC et les 3 env pio.
2. Relire `git diff` ; vérifier circonférence de roue et impulsions/tour (le pit limiter km/h en dépend).
3. Sur une carte réelle déjà en TRACK/PRO (firmware v23.x) : flasher, vérifier `[LIC] tier n` au boot (migration), puis activer un tier supérieur.
4. Banc réel : valider les réglages de `AUDIT.md` §2 ; répondre aux questions du BE (§5.6), puis écrire `src/can/`.
