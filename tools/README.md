# Provisioning & chiffrement — Aptatio ECU

Chaîne d'outils pour **programmer une carte à la commande** (tier BASIC / TRACK / PRO)
et, en production, **chiffrer la flash** pour protéger le secret de licence.

```
tools/
  make_key.py     génère une clé de licence (MAC + tier + secret)
  provision.py    lit la MAC d'une carte, calcule et injecte la licence (série)
  provisioning_log.csv   journal auto (gitignoré)
```

Le **secret** vient de `src/persistence/secrets.h` (ou de la variable
d'environnement `APTATIO_SECRET`). Les scripts refusent de tourner si le
placeholder n'a pas été remplacé.

Prérequis : `pip install pyserial`.

---

## 1. Modèle de licence (rappel)

```
clé = HEX_MAJ( HMAC-SHA256( SECRET , "<MAC>:<tier>" ) )[0..15]
tier : 0 = BASIC   1 = TRACK   2 = PRO
```

- **Déterministe** : même (MAC, tier, secret) → toujours la même clé.
- **Liée à la carte** : une clé calculée pour la MAC A est refusée sur la MAC B.
- **Pas d'usage unique** (choix produit) : la clé reste valable à vie sur *sa* carte.
- **Ré-activation** (carte réinitialisée / SAV) : régénérer la même clé et la
  ressaisir — rien à stocker côté serveur, la clé est déterministe.
- BASIC ne nécessite **aucune** clé : une carte neuve démarre en BASIC.

---

## 2. Deux façons de livrer un tier

### A. Provisioning usine (carte programmée à la commande)

```bash
# 1. Flasher le firmware "factory" (= prod + console série MAC?/KEY/WIPE)
pio run -e factory -t upload

# 2. Provisionner : lit la MAC, calcule la clé, l'injecte, journalise
python tools/provision.py --port COM5 --tier TRACK --order 1234

# 3. Flasher le firmware de PROD par-dessus, SANS effacer la NVS
pio run -e esp32dev -t upload
```

`pio run -t upload` réécrit bootloader + table de partitions + app, **pas** la
partition NVS (offset 0x9000) → la licence injectée à l'étape 2 survit.
Au boot, `license::begin()` revalide la clé stockée → la carte est en TRACK.

> Vérifier `tools/provisioning_log.csv` : `timestamp, mac, tier, key, order`.

#### Sans la console série (variante NVS directe)

Si tu préfères ne jamais flasher le build `factory` :

```bash
# lire la MAC : flasher esp32dev, ouvrir le moniteur -> ligne "[BOOT] HWID: AA:BB:.."
python tools/make_key.py AA:BB:CC:DD:EE:FF TRACK        # -> la clé
# générer une image NVS contenant licenseKey + licenseTier (ancien format, toujours lu tant que
# l'item "lic" = "<tier>:<clé>" écrit par le firmware n'existe pas), puis :
#   esptool.py --port COMx write_flash 0x9000 nvs.bin
```

(La console série de `provision.py` est plus robuste : la carte valide la clé
elle-même, pas besoin de connaître l'offset NVS.)

### B. Upgrade terrain (client déjà équipé)

Le client va sur le site, achète PRO, reçoit sa clé :

```bash
python tools/make_key.py <MAC-du-client> PRO
```

Il la saisit dans la page web (`Licence` → *Activer*) → `POST /api/license`
→ reboot en PRO. La MAC, il la lit sur la page (`HWID: ...`).

---

## 3. Générer les clés côté site de commande

`make_key.py` est la référence. Portage direct dans ton backend :

**Python**
```python
import hmac, hashlib
KEY_LEN = 16
TIERS = {"BASIC": 0, "TRACK": 1, "PRO": 2}
def make_key(secret: bytes, mac: str, tier: str) -> str:
    p = f"{mac}:{TIERS[tier]}".encode()               # mac = "AA:BB:CC:DD:EE:FF" MAJ
    return hmac.new(secret, p, hashlib.sha256).hexdigest().upper()[:KEY_LEN]
```

**Node**
```js
const crypto = require("crypto");
const KEY_LEN = 16, TIERS = { BASIC: 0, TRACK: 1, PRO: 2 };
const makeKey = (secret, mac, tier) =>
  crypto.createHmac("sha256", secret)
        .update(`${mac}:${TIERS[tier]}`)
        .digest("hex").toUpperCase().slice(0, KEY_LEN);
```

Le site n'a **pas besoin de base de données** pour les clés (déterministes).
Une table `MAC ↔ commande ↔ tier vendu` reste utile pour le SAV et la compta.

Flux stock : tu saisis les MAC de ton lot de cartes en BDD ; à la commande, le
bon de commande porte la MAC de la carte attribuée → tu prends cette carte, tu
lances `provision.py --tier <vendu>`, tu flashes la prod, tu expédies.

---

## 4. Chiffrement flash (protection du secret) — production

Sans ça, le `SECRET` est extractible d'un dump flash → n'importe qui génère
toutes les clés. À faire **après** validation complète du firmware (opérations
sur eFuses = **irréversibles**). Teste d'abord sur **une carte sacrifiable**.

### 4.1 Flash Encryption — mode développement (recommandé pour un petit atelier)

Permet de continuer à re-flasher (filaire chiffré + OTA). Chiffre la flash
(dont la NVS et donc le secret).

```bash
# clé de chiffrement — UNE fois, gardée hors-ligne et secrète
espsecure.py generate_flash_encryption_key flash_encryption_key.bin

# par carte : graver la clé puis activer le chiffrement
espefuse.py --port COMx burn_key flash_encryption flash_encryption_key.bin
espefuse.py --port COMx burn_efuse FLASH_CRYPT_CONFIG 0xF
espefuse.py --port COMx burn_efuse FLASH_CRYPT_CNT 0x1

# (re)flasher en chiffré
esptool.py --port COMx write_flash --encrypt \
  0x1000  .pio/build/esp32dev/bootloader.bin \
  0x8000  .pio/build/esp32dev/partitions.bin \
  0x10000 .pio/build/esp32dev/firmware.bin
```

Durcissements eFuse conseillés ensuite : `DISABLE_DL_DECRYPT`,
`DISABLE_DL_CACHE`, `JTAG_DISABLE` (voir doc Espressif « Flash Encryption »).

### 4.2 Secure Boot v2 — pour bloquer un firmware modifié

Empêche de booter une app non signée par ta clé (donc de shunter le check
licence, et d'installer une OTA non signée).

⚠️ **Difficile avec le combo Arduino + PlatformIO** : le core Arduino fournit un
bootloader précompilé ; Secure Boot v2 exige de reconstruire et **signer** le
bootloader. En pratique, ça demande soit un bootloader custom signé, soit de
passer le projet sous **ESP-IDF** (`idf.py menuconfig` → *Secure Boot v2*, la
signature/chiffrement sont alors gérés par `idf.py flash`).

Décision à prendre selon l'écart de prix BASIC↔PRO à protéger :
- petit écart / petit volume → Flash Encryption (4.1) seule, suffisant pour
  arrêter la copie casuelle ;
- enjeu fort → migration ESP-IDF pour Flash Encryption **Release** + Secure Boot v2.

---

## 5. Récap opérateur (carte à la commande)

| Étape | Commande |
|---|---|
| Flasher build usine | `pio run -e factory -t upload` |
| Provisionner | `python tools/provision.py --port COMx --tier <T> --order <N>` |
| Flasher la prod (garde NVS) | `pio run -e esp32dev -t upload` |
| (prod) chiffrer | section 4.1 |
| Vérifier | moniteur série → `[LIC] tier <n>` au boot |
