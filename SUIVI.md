# SUIVI — état au 2026-10-19

**État** : audit + corrections + banc de test PC + **batterie sécurité abonnements** terminés. Les 3 environnements PlatformIO
compilent (`esp32dev`, `wokwi`, `factory`). Rien n'est commité (l'utilisateur gère git). Détail : `AUDIT.md` (§1 #20-21, §6.4).
Commandes : `python test/host/run_tests.py` (tout) · `python test/host/check_security.py [--elf]` (config sécurité, après `pio run -e esp32dev`).

**Fait dans cette passe (télémétrie réservée PRO)** : `sseTask` (`Telemetry.cpp`) n'envoie plus que `{"tier":"..","locked":true}` si
tier < PRO (aucune valeur rpm/vitesse/slip/lean/imu/eau) ; PRO inchangé. `WebPage.h` : message « Télémétrie temps réel et data-log :
réservés à la version PRO » si `d.locked` (CSP inchangée). `/api/state` vérifié : config/licence seulement, aucune fuite. Assistances
et Core 1 non touchés ; garde-fou IMU et lecture eau restent actifs en interne. Docs : README §1.2/§1.5/§5, BUILD.md (route /events).
Tests : `test_supervisor` 8 -> 11 (BASIC, TRACK verrouillés ; PRO complet) ; `bootSup` prend un tier (défaut PRO, donc les 3 tests SSE
existants sont inchangés) ; mutations vérifiées (`< Basic`, `< Track`, `true`). Suite PC 273/273 ; 3 env pio compilent ; check_security --elf 0 échec.

**Jamais fait** : aucun essai matériel (ni affichage réel de la page verrouillée sur navigateur), Wokwi non exécuté (pas de jeton), aucun code CAN (spec + GPIO18/19 réservées),
aucun test sur carte réelle de l'écriture `lic` (migration d'une carte déjà licenciée en v23.x → à essayer une fois).

**Décisions à prendre (utilisateur)**
1. **Modèle d'abonnement** : la licence est perpétuelle et liée à la MAC (aucune expiration, aucun renouvellement dans le code).
   Si un vrai abonnement à durée est voulu, il n'existe pas.
2. **Secret HMAC** : 28 car. / ~89 bits → recommandé 32 car. aléatoires (`python -c "import secrets;print(secrets.token_hex(32))"`),
   mais **le changer invalide toutes les clés déjà vendues** : à faire avant la 1re vente seulement.
3. Flash Encryption + Secure Boot (seule vraie protection contre le dump flash) — `tools/README.md` §4.
4. Version (`Version.h` = 23.0, AUDIT §3.2) ; identité de carte via `esp_read_mac` plutôt que `WiFi.macAddress()` (AUDIT §3.3).

**Prochaine étape exacte**
1. Relire `git diff`, décider la version ; vérifier circonférence de roue et impulsions/tour (le pit limiter km/h en dépend).
2. Sur une carte réelle déjà en TRACK/PRO (firmware v23.x) : flasher ce firmware, vérifier `[LIC] tier n` au boot (migration), puis activer un tier supérieur.
3. Banc réel : valider les réglages de `AUDIT.md` §2 ; répondre aux questions du BE (§5.6), puis écrire `src/can/`.
