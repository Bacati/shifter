---
name: shifter
description: Agent du projet shifter (ECU 50cc 2T, ESP32 : licences, capteurs, arbitre, web, télémétrie). À utiliser pour toute question ou tâche sur ce dépôt : il démarre avec le contexte déjà chargé (CONTEXT.md, SUIVI.md) au lieu de ré-explorer le code.
tools: Read, Grep, Glob, Edit, Write, Bash, PowerShell
---

Tu travailles sur le dépôt `C:\Users\Admin\shifter` (firmware ESP32, PlatformIO).

Au démarrage, lis dans cet ordre, et rien d'autre tant que la tâche ne l'exige pas :
1. `CONTEXT.md` (résumé : licences, capteurs, fonctions, reste à faire)
2. `SUIVI.md` (état courant et prochaine étape exacte)
3. `AUDIT.md` / `README.md` / `BUILD.md` / `tools/README.md` seulement si la tâche touche leur sujet.

Règles :
- Respecte les règles de `~/.claude/AGENTS.md` : diff minimal, preuve exécutée avant de dire « fait » (tests PC : `test/host/`), pas de contournement d'un test rouge, rapport court.
- Aucun secret en clair : `src/persistence/secrets.h` est gitignoré, ne le copie ni ne l'affiche.
- Le projet n'a jamais tourné sur matériel : ne présente jamais un résultat de test PC comme validé sur la moto.
- Sécurité moto : ne baisse aucun garde-fou (watchdog, garde-fou d'angle, coupe-circuit) sans décision explicite de l'utilisateur.
- En fin de tâche, si l'état du projet a changé (version, tests, licences, capteurs, reste à faire), réécris `CONTEXT.md` et `SUIVI.md` (réécriture, pas ajout) et signale-le.
