#!/usr/bin/env python3
"""Controle statique de la securite des licences (ce que les tests C++ ne voient pas).

    python test/host/check_security.py            # sortie 1 si un controle FAIL (les WARN n'echouent pas)
    python test/host/check_security.py --elf      # + inspecte .pio/build/esp32dev/firmware.elf (apres `pio run -e esp32dev`)

Le secret n'est JAMAIS affiche.
"""
import math
import re
import subprocess
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
results = []   # (niveau, texte)


def check(level, ok, text):
    results.append(("OK" if ok else level, text))


def git(*a):
    r = subprocess.run(["git", *a], cwd=ROOT, capture_output=True, text=True, encoding="utf-8", errors="replace")
    return r.returncode, r.stdout


# ---- 1. secret ----------------------------------------------------------------------------------
sec_file = ROOT / "src" / "persistence" / "secrets.h"
secret = None
if sec_file.is_file():
    m = re.search(r'#define\s+LICENSE_HMAC_SECRET\s+"([^"]*)"', sec_file.read_text(encoding="utf-8", errors="replace"))
    secret = m.group(1) if m else None
check("FAIL", secret is not None, "secrets.h present et definit LICENSE_HMAC_SECRET")
if secret is not None:
    check("FAIL", "change" not in secret.lower() and "insecure" not in secret.lower() and "example" not in secret.lower(),
          "le secret n'est pas un placeholder")
    check("FAIL", len(secret) >= 16, f"secret >= 16 caracteres (actuel : {len(secret)})")
    c = Counter(secret)
    ent = -sum(n / len(secret) * math.log2(n / len(secret)) for n in c.values()) * len(secret)
    check("WARN", len(secret) >= 32 and ent >= 128,
          f"secret robuste : >= 32 caracteres ET >= 128 bits d'entropie estimee (actuel : {len(secret)} car., ~{ent:.0f} bits, "
          f"{len(c)} symboles distincts). Generer : python -c \"import secrets;print(secrets.token_hex(32))\" "
          "-- ATTENTION : changer le secret invalide TOUTES les cles deja vendues")
    rc, out = git("ls-files", "--", "src/persistence/secrets.h")
    check("FAIL", rc == 0 and out.strip() == "", "secrets.h n'est PAS suivi par git")
    rc, out = git("check-ignore", "src/persistence/secrets.h")
    check("FAIL", rc == 0, "secrets.h est dans .gitignore")
    rc, out = git("log", "--all", "--oneline", "-S" + secret)
    check("FAIL", rc == 0 and out.strip() == "", "le secret actuel n'apparait dans AUCUN commit (historique complet)")
    rc, out = git("log", "--all", "--oneline", "--", "src/persistence/secrets.h")
    check("FAIL", rc == 0 and out.strip() == "", "secrets.h n'a jamais ete commite")
    rc, out = git("grep", "-n", "-F", secret, "--", ".", ":(exclude)src/persistence/secrets.h")
    check("FAIL", out.strip() == "", "le secret n'est recopie dans aucun fichier suivi")

# ---- 2. drapeaux de build du firmware de production ---------------------------------------------
ini = (ROOT / "platformio.ini").read_text(encoding="utf-8")
sections = {}
cur = None
for line in ini.splitlines():
    s = re.match(r"\[(.+)\]", line.strip())
    if s:
        cur = s.group(1); sections[cur] = []
    elif cur and not line.lstrip().startswith(";"):
        sections[cur].append(line)
prod = "\n".join(sections.get("env:esp32dev", []))
for flag in ("LICENSE_PRINT_KEYS_ON_BOOT", "ALLOW_INSECURE_DEFAULT_SECRET", "WOKWI_SIM", "SIM_TIER", "FACTORY_TOOLS", "SIM_FREEZE"):
    check("FAIL", flag not in prod, f"env esp32dev (production) ne definit pas -D{flag}")
check("FAIL", "[env:esp32dev]" in ini and "extends" not in prod, "env esp32dev n'herite d'aucun env de test")
for env in ("env:wokwi", "env:factory"):
    body = "\n".join(sections.get(env, []))
    check("FAIL", "extends" in body and "esp32dev" in body, f"{env} etend esp32dev (ne modifie pas la prod)")

# ---- 3. coherence outil / firmware -----------------------------------------------------------------
mk = (ROOT / "tools" / "make_key.py").read_text(encoding="utf-8")
lic = (ROOT / "src" / "persistence" / "License.h").read_text(encoding="utf-8")
k1 = re.search(r"^KEY_LEN\s*=\s*(\d+)", mk, re.M)
k2 = re.search(r"#define\s+LICENSE_KEY_HEX_LEN\s+(\d+)", lic)
check("FAIL", bool(k1 and k2 and k1.group(1) == k2.group(1)), "longueur de cle identique outil Python / firmware")
check("WARN", bool(k2 and int(k2.group(1)) >= 16), "cle >= 64 bits (16 hex)")
rc, out = git("check-ignore", "tools/provisioning_log.csv")
check("FAIL", rc == 0, "tools/provisioning_log.csv (MAC + cles vendues) est ignore par git")

# ---- 4. sources : aucune trace de licence en clair, aucune sortie de cle -------------------------
src_text = ""
for p in list((ROOT / "src").rglob("*.cpp")) + list((ROOT / "src").rglob("*.h")) + [ROOT / "shifter.ino"]:
    if p.name in ("secrets.h", "secrets.example.h"):
        continue
    src_text += p.read_text(encoding="utf-8", errors="replace")
prints = [l for l in src_text.splitlines() if re.search(r"(Serial|print).*(licenseKey|expectedKey|LICENSE_HMAC_SECRET)", l)]
unguarded = [l for l in prints if "LICENSE_PRINT_KEYS_ON_BOOT" not in l]
# les seules sorties de cle autorisees sont sous #ifdef LICENSE_PRINT_KEYS_ON_BOOT / outils usine (voir lecture manuelle)
check("WARN", len(unguarded) <= 2, f"sorties serie mentionnant une cle/secret : {len(unguarded)} ligne(s) a relire (usine/sim seulement)")

# ---- 5. firmware compile ---------------------------------------------------------------------------
if "--elf" in sys.argv:
    elf = ROOT / ".pio" / "build" / "esp32dev" / "firmware.elf"
    if not elf.is_file():
        check("FAIL", False, "firmware.elf introuvable : lancer `pio run -e esp32dev` d'abord")
    else:
        data = elf.read_bytes()
        for needle in (b"insecure-default-change-me", b"FACTORY MODE", b"[SIM]", b"WIPE", b"LICENSE_PRINT_KEYS"):
            check("FAIL", needle not in data, f"firmware de production sans la chaine {needle.decode()!r}")
        if secret:
            check("WARN", secret.encode() in data,
                  "(info) le secret HMAC est dans le binaire : sans Flash Encryption un dump flash le revele")

width = max(len(t) for _, t in results)
fails = warns = 0
for lvl, t in results:
    print(f"[{lvl:4s}] {t}")
    fails += lvl == "FAIL"
    warns += lvl == "WARN"
print(f"\n{len(results) - fails - warns} OK, {warns} avertissement(s), {fails} ECHEC(S)")
sys.exit(1 if fails else 0)
