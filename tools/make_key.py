#!/usr/bin/env python3
"""Generateur de cles de licence TTP ECU.

    cle = HEX_MAJ( HMAC-SHA256( SECRET, "<MAC>:<tier>" ) )[0 .. LICENSE_KEY_HEX_LEN-1]

Doit produire EXACTEMENT la meme chaine que src/persistence/License.cpp.

Le SECRET est lu, dans l'ordre :
  1) variable d'environnement  APTATIO_SECRET
  2) le #define LICENSE_HMAC_SECRET de src/persistence/secrets.h

Exemples :
    python tools/make_key.py AA:BB:CC:DD:EE:FF PRO
    python tools/make_key.py AA:BB:CC:DD:EE:FF 2
    python tools/make_key.py --all AA:BB:CC:DD:EE:FF
"""
import argparse
import hmac
import hashlib
import os
import re
import sys
from pathlib import Path

KEY_LEN = 16  # doit valoir LICENSE_KEY_HEX_LEN (src/persistence/License.h)
TIERS = {"BASIC": 0, "TRACK": 1, "PRO": 2}
TIER_NAME = {v: k for k, v in TIERS.items()}

SECRETS_H = Path(__file__).resolve().parent.parent / "src" / "persistence" / "secrets.h"


def load_secret() -> bytes:
    env = os.environ.get("APTATIO_SECRET")
    if env:
        return env.encode()
    if not SECRETS_H.is_file():
        sys.exit(f"secret introuvable : ni $APTATIO_SECRET ni {SECRETS_H}")
    text = SECRETS_H.read_text(encoding="utf-8", errors="replace")
    m = re.search(r'#define\s+LICENSE_HMAC_SECRET\s+"([^"]*)"', text)
    if not m:
        sys.exit(f"LICENSE_HMAC_SECRET absent de {SECRETS_H}")
    val = m.group(1)
    if "CHANGE" in val.upper():
        sys.exit("secrets.h contient encore le placeholder : mets un vrai secret.")
    return val.encode()


def normalise_mac(mac: str) -> str:
    """Format attendu par WiFi.macAddress() : 6 octets hex MAJ separes par ':'."""
    raw = re.sub(r"[^0-9A-Fa-f]", "", mac)
    if len(raw) != 12:
        sys.exit(f"MAC invalide : {mac!r} (attendu 6 octets hex)")
    return ":".join(raw[i:i + 2] for i in range(0, 12, 2)).upper()


def make_key(secret: bytes, mac: str, tier: int) -> str:
    payload = f"{mac}:{tier}".encode()
    digest = hmac.new(secret, payload, hashlib.sha256).hexdigest().upper()
    return digest[:KEY_LEN]


def parse_tier(s: str) -> int:
    u = s.strip().upper()
    if u in TIERS:
        return TIERS[u]
    if u.isdigit() and int(u) in TIER_NAME:
        return int(u)
    sys.exit(f"tier invalide : {s!r} (BASIC/TRACK/PRO ou 0/1/2)")


def main() -> None:
    ap = argparse.ArgumentParser(description="Genere une cle de licence TTP ECU.")
    ap.add_argument("mac", help="adresse MAC de la carte (HWID affiche par la carte)")
    ap.add_argument("tier", nargs="?", default="PRO", help="BASIC | TRACK | PRO (defaut PRO)")
    ap.add_argument("--all", action="store_true", help="affiche les 3 cles pour cette carte")
    args = ap.parse_args()

    secret = load_secret()
    mac = normalise_mac(args.mac)

    if args.all:
        for t in (0, 1, 2):
            print(f"{TIER_NAME[t]:5s}  {make_key(secret, mac, t)}")
    else:
        t = parse_tier(args.tier)
        print(make_key(secret, mac, t))


if __name__ == "__main__":
    main()
