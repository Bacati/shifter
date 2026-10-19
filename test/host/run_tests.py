#!/usr/bin/env python3
"""Banc de test PC du firmware : compile le VRAI code de src/ contre de faux Arduino/FreeRTOS
(test/host/mock) puis execute chaque test dans son propre processus.

    python test/host/run_tests.py            # tout compiler + lancer
    python test/host/run_tests.py sensors    # seulement les fichiers test_*sensors*.cpp
    python test/host/run_tests.py -v rpm     # filtre par nom de test, sortie detaillee

Compilateur : $CXX, sinon `python -m ziglang c++` (pip install ziglang), sinon g++/clang++.
Chaque test_*.cpp declare ses sources avec des lignes   // SRC: core/SharedState.cpp ...
et ses options avec                                     // DEFS: -DWOKWI_SIM
"""
import concurrent.futures as cf
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
SRC = ROOT / "src"
BUILD = HERE / "build"


def find_cxx():
    if os.environ.get("CXX"):
        return os.environ["CXX"].split()
    probe = subprocess.run([sys.executable, "-m", "ziglang", "version"], capture_output=True, text=True)
    if probe.returncode == 0:
        return [sys.executable, "-m", "ziglang", "c++"]
    for c in ("g++", "clang++"):
        if shutil.which(c):
            return [c]
    sys.exit("aucun compilateur C++ : pip install ziglang")


def parse_header(path):
    srcs, defs = [], []
    for line in path.read_text(encoding="utf-8").splitlines():
        m = re.match(r"\s*//\s*SRC:\s*(.*)", line)
        if m:
            srcs += m.group(1).split()
        m = re.match(r"\s*//\s*DEFS:\s*(.*)", line)
        if m:
            defs += m.group(1).split()
    return srcs, defs


def compile_one(cxx, test):
    srcs, defs = parse_header(test)
    out = BUILD / (test.stem + ".exe")
    cmd = cxx + ["-std=gnu++11", "-O1", "-g", "-w", "-fno-strict-aliasing",
                 f"-I{HERE / 'mock'}", f"-I{HERE}", f"-I{SRC}", f"-I{ROOT}"] + defs + \
        [str(HERE / "host.cpp"), str(test)] + [str(SRC / s) for s in srcs] + ["-o", str(out)]
    r = subprocess.run(cmd, capture_output=True, text=True)
    return test, out, r


def main():
    args = sys.argv[1:]
    verbose = "-v" in args
    args = [a for a in args if a != "-v"]
    file_filter = None
    name_filter = None
    for a in args:
        if file_filter is None and any(HERE.glob(f"test_*{a}*.cpp")):
            file_filter = a
        else:
            name_filter = a
    BUILD.mkdir(exist_ok=True)
    (BUILD / ".gitignore").write_text("*\n")
    tests = sorted(HERE.glob("test_*.cpp"))
    if file_filter:
        tests = [t for t in tests if file_filter in t.name]
    cxx = find_cxx()

    built = []
    with cf.ThreadPoolExecutor(max_workers=4) as ex:
        for test, out, r in ex.map(lambda t: compile_one(cxx, t), tests):
            if r.returncode != 0:
                print(f"[BUILD FAIL] {test.name}\n{r.stderr[-3000:]}")
                built.append((test, None))
            else:
                built.append((test, out))

    total = failed = 0
    failed_names = []
    for test, exe in built:
        if exe is None:
            failed += 1
            failed_names.append(test.name + " (compilation)")
            continue
        names = subprocess.run([str(exe), "--list"], capture_output=True, text=True).stdout.split()
        for n in names:
            if name_filter and name_filter not in n:
                continue
            total += 1
            r = subprocess.run([str(exe)] + (["--verbose"] if verbose else []) + [n],
                               capture_output=True, text=True, timeout=300, cwd=ROOT,
                               env={**os.environ, "HOST_TEST_PYTHON": sys.executable})
            ok = r.returncode == 0
            if not ok:
                failed += 1
                failed_names.append(f"{test.stem}::{n}")
            if verbose or not ok:
                print(r.stdout.rstrip())
                if r.stderr.strip():
                    print(r.stderr.rstrip())
            else:
                print(f"[PASS] {test.stem}::{n}")
    print(f"\n{total - failed}/{total} tests OK" + (f", ECHECS : {failed_names}" if failed else ""))
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
