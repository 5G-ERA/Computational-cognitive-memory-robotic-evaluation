#!/usr/bin/env python3
"""Bitacora de sesion: registra el ultimo run en un fichero persistente y versionado.

Cada run deja dos clases de dato y las dos hacen falta:
  - lo MEDIDO, que se saca del dataset (llegada, tiempo, colisiones, cruce, desvio lateral en
    el vano, bateria, estados META, cobertura);
  - lo OBSERVADO por el operador, que la instrumentacion NO ve. El caso conocido: el 14-ago una
    llegada se registro con ncol=0 y el brazo izquierdo toco el marco. Por eso la nota va como
    argumento obligatorio y no como comentario opcional.

Uso, justo despues de cada run:
    python3 tools/bitacora.py "cruce limpio, sin roce"
    python3 tools/bitacora.py --parada "lo pare yo, se pegaba a la jamba"

Escribe en tasks/SESSION_LOG_<fecha>.md, en append. Nunca reescribe lo anterior.
"""
import json
import glob
import math
import os
import sys
import time
import collections

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DX, DY = -3.90, 1.25
AX = math.radians(135.0)
UX, UY = math.cos(AX), math.sin(AX)


def ultimo_run():
    fs = [f for f in glob.glob(os.path.join(RAIZ, "dataset", "2026*_ours_[AB].json"))
          if "_col" not in f and "_end" not in f]
    if not fs:
        sys.exit("no hay ningun run en dataset/")
    return max(fs, key=os.path.getmtime)


def lat(p):
    return -(p["x"] - DX) * UY + (p["y"] - DY) * UX


def srob(p):
    return (p["x"] - DX) * UX + (p["y"] - DY) * UY


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    parada = "--parada" in sys.argv
    if not args:
        sys.exit(__doc__)
    nota = " ".join(args)

    f = ultimo_run()
    d = json.load(open(f))
    ss = d.get("samples") or []
    if len(ss) < 5:
        sys.exit("el ultimo run tiene %d muestras: no se registra" % len(ss))
    env = d.get("env_g1") or {}
    ev = d.get("events") or []
    g = d.get("goal") or {}
    base = os.path.basename(f)[:-5]
    destino = "A" if base.endswith("_A") else "B"

    dfin = math.hypot(ss[-1]["x"] - g.get("x", 0), ss[-1]["y"] - g.get("y", 0))
    llego = dfin < 0.45
    ncol = sum(1 for e in ev if e.get("kind") == "collision")
    cruces = sum(1 for e in ev if e.get("kind") == "door_crossed")
    van = [p for p in ss if abs(srob(p)) <= 0.25]
    la = (sum(lat(p) for p in van) / len(van)) if van else None
    bats = [p.get("bat") for p in ss if isinstance(p.get("bat"), (int, float))]
    est = collections.Counter(p.get("meta_state") for p in ss if p.get("meta_state"))
    cov = sorted(p["cov_def"] for p in ss if isinstance(p.get("cov_def"), (int, float)))
    fases = collections.Counter()
    for p in ss:
        fases[str(p.get("phase", "")).replace("AGR-", "")] += 1

    flags = " ".join("%s=%s" % (k.replace("G1_", ""), env[k]) for k in
                     ("G1_METASM", "G1_DOOR_CTR2", "G1_DOOR_CTR_HOLD", "G1_DOOR_YAW2",
                      "G1_DOOR_EXIT_CTR", "G1_DOOR_CTR_TOL", "G1_DOOR_VIS")
                     if env.get(k) is not None)

    fecha = time.strftime("%Y-%m-%d")
    log = os.path.join(RAIZ, "tasks", "SESSION_LOG_%s.md" % fecha)
    nuevo = not os.path.exists(log)
    with open(log, "a") as w:
        if nuevo:
            w.write("# Session log — %s\n\n"
                    "Appended by `tools/bitacora.py` after each run. Measured facts come from the\n"
                    "dataset; the **operator note** is the part the instrumentation cannot see —\n"
                    "arm contact, spill, a run stopped by hand. `ncol = 0` does not mean clean.\n\n"
                    "---\n\n" % fecha)
        w.write("## %s → %s  ·  `%s`\n\n" % (time.strftime("%H:%M"), destino, base))
        w.write("| | |\n|---|---|\n")
        w.write("| Outcome | **%s**%s |\n" % (
            "arrived" if llego else "did not arrive",
            " — **stopped by operator**" if parada else ""))
        w.write("| Arm / flags | `%s` · %s |\n" % (env.get("G1_ARM", "-"), flags or "—"))
        w.write("| Duration | %.0f s |\n" % ss[-1]["t"])
        w.write("| Final distance to goal | %.2f m |\n" % dfin)
        w.write("| Collisions (detected) | %d |\n" % ncol)
        w.write("| Door crossings | %d |\n" % cruces)
        if la is not None:
            w.write("| Lateral offset at the gap | %+.3f m |\n" % la)
        if bats:
            w.write("| Battery | %d%% → %d%% |\n" % (max(bats), min(bats)))
        if est:
            w.write("| META states | %s |\n" % ", ".join(
                "%s %.0f%%" % (k, 100.0 * v / len(ss)) for k, v in est.most_common()))
        if cov:
            w.write("| Coverage deficit | median %.3f, p90 %.3f, max %.3f |\n"
                    % (cov[len(cov) // 2], cov[int(0.9 * len(cov))], cov[-1]))
        w.write("| Phases | %s |\n" % ", ".join(
            "%s×%d" % (k, v) for k, v in fases.most_common(6) if k))
        w.write("\n**Operator:** %s\n\n---\n\n" % nota)

    print("registrado en %s" % os.path.relpath(log, RAIZ))
    print("  %s → %s  %s  %.0fs  col=%d  cruces=%d  lat=%s" % (
        base[9:15], destino, "LLEGO" if llego else "fallo", ss[-1]["t"], ncol, cruces,
        ("%+.3f" % la) if la is not None else "-"))


if __name__ == "__main__":
    main()
