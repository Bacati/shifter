#!/usr/bin/env python3
"""Fenetre de programmation d'une carte TTP ECU.

    python tools/license_gui.py

Branche la carte en USB, choisis le port et la version (BASIC / TRACK / PRO),
clique "Programmer la carte". Enchaine automatiquement :
  1) pio run -e factory -t upload    (firmware + console de provisioning)
  2) provision.py                    (lit la MAC, calcule et injecte la licence)
  3) pio run -e esp32dev -t upload   (firmware de prod, la NVS est conservee)

La section "Cle seule" calcule une cle a la main (upgrade terrain), sans carte.
Prerequis : pip install pyserial platformio
"""
import subprocess
import sys
import threading
import tkinter as tk
from pathlib import Path
from tkinter import ttk

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import make_key as mk  # noqa: E402
import provision as pv  # noqa: E402

try:
    from serial.tools import list_ports
except ImportError:  # pyserial deja exige par provision
    list_ports = None


class App(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("TTP ECU - Programmation carte")
        self.resizable(False, False)
        pad = {"padx": 10, "pady": 4}

        # ---- programmation de la carte -------------------------------------
        box = ttk.LabelFrame(self, text="Programmer une carte (USB)")
        box.grid(row=0, column=0, sticky="ew", padx=10, pady=8)

        ttk.Label(box, text="Port :").grid(row=0, column=0, sticky="w", **pad)
        self.port = tk.StringVar()
        self.port_cb = ttk.Combobox(box, textvariable=self.port, width=12)
        self.port_cb.grid(row=0, column=1, sticky="w", **pad)
        ttk.Button(box, text="Actualiser", command=self.refresh_ports).grid(
            row=0, column=2, sticky="w", **pad)

        ttk.Label(box, text="Version :").grid(row=1, column=0, sticky="w", **pad)
        self.tier = tk.StringVar(value="TRACK")
        bar = ttk.Frame(box)
        bar.grid(row=1, column=1, columnspan=2, sticky="w", **pad)
        for name in ("BASIC", "TRACK", "PRO"):
            ttk.Radiobutton(bar, text=name, value=name, variable=self.tier,
                            command=self._auto_key).pack(side="left", padx=6)

        ttk.Label(box, text="N° commande :").grid(row=2, column=0, sticky="w", **pad)
        self.order = tk.StringVar()
        ttk.Entry(box, textvariable=self.order, width=16).grid(
            row=2, column=1, columnspan=2, sticky="w", **pad)

        self.go = ttk.Button(box, text="Programmer la carte", command=self.start)
        self.go.grid(row=3, column=0, columnspan=3, sticky="ew", **pad)

        self.logbox = tk.Text(box, width=64, height=12, state="disabled",
                              font=("Consolas", 9), wrap="word")
        self.logbox.grid(row=4, column=0, columnspan=3, padx=10, pady=(4, 8))

        # ---- cle seule -----------------------------------------------------
        man = ttk.LabelFrame(self, text="Cle seule (upgrade terrain, sans carte)")
        man.grid(row=1, column=0, sticky="ew", padx=10, pady=(0, 8))
        ttk.Label(man, text="MAC / HWID :").grid(row=0, column=0, sticky="w", **pad)
        self.mac = tk.StringVar()
        e = ttk.Entry(man, textvariable=self.mac, width=22, font=("Consolas", 11))
        e.grid(row=0, column=1, **pad)
        e.bind("<Return>", lambda _e: self.make_key())
        ttk.Button(man, text="Generer", command=self.make_key).grid(row=0, column=2, **pad)
        self.key = tk.StringVar()
        ttk.Entry(man, textvariable=self.key, width=22, state="readonly",
                  font=("Consolas", 12, "bold")).grid(row=1, column=1, **pad)
        ttk.Button(man, text="Copier", command=self.copy).grid(row=1, column=2, **pad)

        self.refresh_ports()

    # ---- ports ---------------------------------------------------------
    def refresh_ports(self) -> None:
        ports = [p.device for p in list_ports.comports()] if list_ports else []
        self.port_cb["values"] = ports
        if ports and self.port.get() not in ports:
            self.port.set(ports[-1])

    # ---- journal -------------------------------------------------------
    def log(self, text: str) -> None:
        self.after(0, self._log, text)

    def _log(self, text: str) -> None:
        self.logbox.configure(state="normal")
        self.logbox.insert("end", text.rstrip() + "\n")
        self.logbox.see("end")
        self.logbox.configure(state="disabled")

    # ---- programmation -------------------------------------------------
    def start(self) -> None:
        port = self.port.get().strip()
        if not port:
            self._log("Choisis un port serie.")
            return
        self.go.configure(state="disabled")
        self.logbox.configure(state="normal")
        self.logbox.delete("1.0", "end")
        self.logbox.configure(state="disabled")
        args = (port, self.tier.get(), self.order.get().strip())
        threading.Thread(target=self.worker, args=args, daemon=True).start()

    def pio(self, env: str, port: str) -> None:
        cmd = [sys.executable, "-m", "platformio", "run", "-e", env,
               "-t", "upload", "--upload-port", port]
        flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
        p = subprocess.Popen(cmd, cwd=ROOT, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, text=True,
                             errors="replace", creationflags=flags)
        tail = []
        for line in p.stdout:
            tail.append(line)
            if any(k in line for k in ("Writing at", "Hash of data", "Leaving", "Error", "SUCCESS", "FAILED")):
                self.log(line)
        if p.wait() != 0:
            raise RuntimeError(f"pio -e {env} en echec :\n" + "".join(tail[-15:]))

    def worker(self, port: str, tier_name: str, order: str) -> None:
        try:
            tier = mk.TIERS[tier_name]
            self.log(f"[1/3] Flash firmware 'factory' sur {port} ...")
            self.pio("factory", port)
            self.log(f"[2/3] Provisioning {tier_name} ...")
            pv.provision(port, tier, order, log=self.log)
            self.log("[3/3] Flash firmware de production (NVS conservee) ...")
            self.pio("esp32dev", port)
            self.log(f"\nTERMINE : carte programmee en {tier_name}.")
        except (RuntimeError, SystemExit, OSError) as ex:
            self.log(f"\nECHEC : {ex}")
        finally:
            self.after(0, lambda: self.go.configure(state="normal"))

    # ---- cle seule -----------------------------------------------------
    def _auto_key(self) -> None:
        if self.mac.get().strip():
            self.make_key()

    def make_key(self) -> None:
        try:
            mac = mk.normalise_mac(self.mac.get())
            key = mk.make_key(mk.load_secret(), mac, mk.TIERS[self.tier.get()])
        except SystemExit as ex:
            self.key.set("")
            self._log(str(ex.code))
            return
        self.mac.set(mac)
        self.key.set(key)

    def copy(self) -> None:
        if self.key.get():
            self.clipboard_clear()
            self.clipboard_append(self.key.get())


if __name__ == "__main__":
    App().mainloop()
