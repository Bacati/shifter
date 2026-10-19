# Banc de test PC

Compile le **vrai** code de `src/` (et `shifter.ino`) contre de faux Arduino/FreeRTOS, puis exécute chaque
test dans son propre processus (état isolé). Horloge virtuelle : 20 minutes de conduite s'exécutent en
quelques secondes.

```bash
pip install ziglang            # compilateur C++ (ou avoir g++ / clang++ ; variable CXX pour en forcer un)
python test/host/run_tests.py            # tout
python test/host/run_tests.py imu        # un fichier (test_*imu*.cpp)
python test/host/run_tests.py -v rpm     # filtre par nom de test + sortie détaillée
HOST_SEED=7 test/host/build/test_fw_full.exe --verbose full_chaos_20min_invariants_a_chaque_coupure
HOST_STAMP=1 ...                         # horodate les lignes série (@12.345)
```

## Structure

| Élément | Rôle |
|---|---|
| `host.h/.cpp` | horloge, ordonnanceur coopératif des tâches FreeRTOS (threads à passage de témoin), événements, broches + ISR, MPU6050 / DS18B20 / NVS simulés, mini framework (`TEST`, `CHECK`, `INFO`) |
| `mock/` | faux `Arduino.h`, `Wire`, `OneWire`, `DallasTemperature`, `Preferences`, `WiFi`, `mbedtls` (HMAC-SHA256 réel), `ESPAsyncWebServer`, `Update`, FreeRTOS |
| `common.h`, `common_ride.h` | boucle ECU, générateurs d'impulsions, profil de mouvement moto (IMU physiquement cohérent) |
| `test_*.cpp` | `// SRC:` liste les sources réelles à lier, `// DEFS:` les options de compilation |

## Limites (à ne pas oublier)

- Un seul thread logique s'exécute à la fois : **les courses ISR ↔ tâches ne sont pas testées**.
- La durée d'un tour de `loop()` est supposée (100 µs).
- Le transport HTTP, l'EMI, l'électronique de puissance, le flash réel : sur matériel uniquement.

## Ajouter un test

```cpp
TEST(mon_test) { /* host::runFor(SEC(1)); CHECK(...); */ }
```
Un test qui valide un correctif doit échouer sur l'ancien code : vérifier par mutation (remettre l'ancienne ligne, relancer, restaurer).
