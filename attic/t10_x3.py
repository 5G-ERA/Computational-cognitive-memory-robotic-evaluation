#!/usr/bin/env python3
"""T10 x3 para la campana v2: mundo bloqueado ya cargado en el puente (comprobado por
el llamador). Cada pierna: guion T10 --destino B (reset a A -> arco de descubrimiento).
Apunta cada run al manifiesto v2."""
import os, re, subprocess, sys, time

RAIZ = os.path.dirname(os.path.abspath(__file__))
MAN = os.path.join(RAIZ, "tasks", "manifiestos", "campana_dcc_v2.txt")

for i in range(3):
    t0 = time.time()
    try:
        p = subprocess.run([sys.executable, "guion.py", "T10", "--destino", "B"],
                           env=dict(os.environ, G1_LASER_SNAP="0.5"), cwd=RAIZ,
                           capture_output=True, text=True, timeout=420)
        out = p.stdout + p.stderr
    except subprocess.TimeoutExpired:
        out = "TIMEOUT"
    m = re.search(r"^run: (\S+\.json)$", out, re.MULTILINE)
    if m and m.group(1).endswith("_omega_ref.json"):
        m = None
    f = os.path.relpath(m.group(1), RAIZ) if m else ""
    with open(MAN, "a") as fh:
        fh.write("T10|B|%s\n" % f)
    print("[T10 rep %d] %s (%.0fs)" % (i + 1, f or "SIN RUN", time.time() - t0),
          flush=True)
    time.sleep(2)

with open(MAN, "a") as fh:
    fh.write("T10 X3 COMPLETO (mundo bloqueado)\n")
print("T10 COMPLETO", flush=True)
