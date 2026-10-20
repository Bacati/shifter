#!/usr/bin/env python3
"""Provisioning automatise d'une carte TTP ECU.

Prerequis :
  - la carte est flashee avec l'environnement 'factory' :
        pio run -e factory -t upload
  - pip install pyserial

Deroulement :
  1) ouvre le port serie de la carte
  2) 'MAC?'            -> recupere l'adresse MAC reelle de la carte
  3) calcule la cle    (tools/make_key.py, meme secret que le firmware)
  4) 'KEY <cle>'       -> la carte valide et stocke la licence en NVS
  5) 'INFO'            -> verifie le tier applique
  6) journalise dans tools/provisioning_log.csv

Ensuite : flasher le firmware de prod SANS effacer la NVS :
        pio run -e esp32dev -t upload

Exemples :
    python tools/provision.py --port COM5 --tier TRACK --order 1234
    python tools/provision.py --port /dev/ttyUSB0 --tier PRO
"""
import argparse
import csv
import datetime as dt
import sys
import time
from pathlib import Path

try:
    import serial  # pyserial
except ImportError:
    sys.exit("pyserial manquant :  pip install pyserial")

from make_key import load_secret, make_key, normalise_mac, parse_tier, TIER_NAME

LOG = Path(__file__).resolve().parent / "provisioning_log.csv"


def read_lines(ser, timeout_s=4.0):
    """Lit les lignes disponibles pendant timeout_s."""
    end = time.time() + timeout_s
    buf = b""
    out = []
    while time.time() < end:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                out.append(line.decode(errors="replace").strip())
    return out


def send(ser, cmd):
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    ser.flush()


def find_prefixed(lines, prefix):
    for ln in lines:
        if ln.upper().startswith(prefix.upper()):
            return ln
    return None


def provision(port, tier, order="", baud=115200, expect_mac="", log=print):
    """Provisionne la carte sur `port`. Leve RuntimeError en cas d'echec.
    Retourne (mac, cle)."""
    secret = load_secret()
    with serial.Serial(port, baud, timeout=0.2) as ser:
        time.sleep(1.5)                      # la carte reset a l'ouverture du port
        macline = None
        seen = []
        for attempt in range(8):            # la carte peut encore booter (WiFi ~2 s)
            send(ser, "MAC?")
            lines = read_lines(ser, timeout_s=1.5)
            seen += lines
            macline = find_prefixed(lines, "MAC ")
            if macline:
                break
        if not macline:
            raise RuntimeError("pas de reponse 'MAC ...' de la carte. Recu :\n  "
                               + "\n  ".join(seen[-20:])
                               + "\n(la carte est-elle flashee avec  pio run -e factory  ?)")
        mac = normalise_mac(macline.split(None, 1)[1])

        if expect_mac and normalise_mac(expect_mac) != mac:
            raise RuntimeError(f"MAC lue {mac} != MAC attendue {normalise_mac(expect_mac)}")

        key = make_key(secret, mac, tier)
        log(f"MAC   : {mac}")
        log(f"tier  : {TIER_NAME[tier]}")
        log(f"cle   : {key}")

        send(ser, f"KEY {key}")
        lines = read_lines(ser)
        ok = find_prefixed(lines, "OK tier=")
        if not ok:
            raise RuntimeError("echec : la carte n'a pas accepte la cle. Recu :\n  "
                               + "\n  ".join(lines))
        applied = int(ok.split("tier=")[1].split()[0])
        if applied != tier:
            raise RuntimeError(f"tier applique {applied} != demande {tier}")
        log(f"OK    : carte provisionnee en {TIER_NAME[applied]}")

    new = not LOG.exists()
    with LOG.open("a", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["timestamp", "mac", "tier", "key", "order"])
        w.writerow([dt.datetime.now().isoformat(timespec="seconds"),
                    mac, TIER_NAME[tier], key, order])
    log(f"journalise -> {LOG}")
    return mac, key


def main() -> None:
    ap = argparse.ArgumentParser(description="Provisionne une carte TTP ECU.")
    ap.add_argument("--port", required=True, help="port serie (COM5, /dev/ttyUSB0, ...)")
    ap.add_argument("--tier", required=True, help="BASIC | TRACK | PRO")
    ap.add_argument("--order", default="", help="reference de commande (journal)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mac", default="", help="force la MAC au lieu de la lire (verification)")
    args = ap.parse_args()
    try:
        provision(args.port, parse_tier(args.tier), args.order, args.baud, args.mac)
    except RuntimeError as ex:
        sys.exit(str(ex))


if __name__ == "__main__":
    main()
