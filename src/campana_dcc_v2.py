#!/usr/bin/env python3
"""Campana DCC v2 (25-ago): T1-T9 + T11, 3 reps, con la cadena del testigo validada.

Por que una v2: la campana del 24-ago es ANTERIOR a la cadena validada del cristal
(sin G1_COVREF vivo -> cov_missing=None en todas las muestras; rect infra-resolucion;
instrumento base instantanea). El repuntuado offline de esa campana es valido, pero sus
runs no llevan la evidencia de lidar que el resolutor necesita en vivo. Esta campana
corre con: guion que exporta G1_COVREF a TODAS las configs, panel validado sobre pared
fiable, cov_missing v2 (base acumulada) por defecto, y puerta de encaramiento en el
certificado. T10 va aparte (mundo bloqueado, reinicio del puente); T12 es propiedad
del registro y no necesita puente.

Encadenado: destino B con /reset (sale de A); destino A con --sin-reset (sale de B,
encadenada) -- el /reset por defecto de guion teleporta a A y vaciaria las piernas A.
"""
import json, os, re, subprocess, sys, time

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAN = os.path.join(RAIZ, "tasks", "manifiestos", "campana_dcc_v2.txt")
ORDEN = [("T1", "B"), ("T2", "A"), ("T3", "B"), ("T4", "A"), ("T5", "B"),
         ("T6", "A"), ("T7", "B"), ("T8", "A"), ("T9", "B"), ("T11", "A")]
REPS = int(os.environ.get("REPS", "3"))

hechas = 0
if os.path.exists(MAN):
    hechas = sum(1 for ln in open(MAN) if "|" in ln)
    print("continuando: %d piernas ya en el manifiesto" % hechas, flush=True)

i_global = 0
for ronda in range(REPS):
    for cfg, dst in ORDEN:
        i_global += 1
        if i_global <= hechas:
            continue
        args = [sys.executable, os.path.join(RAIZ, "src", "guion.py"), cfg, "--destino", dst]
        if dst == "A":
            args.append("--sin-reset")
        env = dict(os.environ, G1_LASER_SNAP="0.5")
        t0 = time.time()
        try:
            p = subprocess.run(args, env=env, cwd=RAIZ, capture_output=True,
                               text=True, timeout=420)
            out = p.stdout + p.stderr
        except subprocess.TimeoutExpired:
            out = "TIMEOUT"
        m = re.search(r"^run: (\S+\.json)$", out, re.MULTILINE)
        if m and m.group(1).endswith("_omega_ref.json"):
            m = None
        f = os.path.relpath(m.group(1), RAIZ) if m else ""
        with open(MAN, "a") as fh:
            fh.write("%s|%s|%s\n" % (cfg, dst, f))
        print("[r%d %s->%s] %s (%.0fs)" % (ronda + 1, cfg, dst, f or "SIN RUN",
                                           time.time() - t0), flush=True)
        time.sleep(2)

with open(MAN, "a") as fh:
    fh.write("CAMPANA DCC V2 T1-T9+T11 COMPLETA (%d reps; T10 y T12 aparte)\n" % REPS)
print("CAMPANA V2 COMPLETA", flush=True)
