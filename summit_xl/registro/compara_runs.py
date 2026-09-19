#!/usr/bin/env python3
"""compara_runs.py — junta las runs del G1 y del Summit en una tabla y las resume lado a lado.

  python3 compara_runs.py --g1 RUTA/dataset/runs_stats.csv --summit RUTA/runs_stats.csv [--salida comparacion.csv]

Las columnas comunes casan por NOMBRE (son las 71 del G1). Lo que un robot no tiene queda en blanco.
Ojo con dos columnas que se llaman igual y no miden igual, y por eso salen marcadas con * en el resumen:
  collisions   G1: la pose no avanza o pico de par en las piernas · Summit: las ruedas no giran lo mandado
  progression  normalizada a 0.30 m/s, la marcha del G1; el Summit cruza la puerta a 0.05
"""
import argparse, csv, statistics
from collections import defaultdict

COLS_RESUMEN = ["time_s", "path_m", "efficiency", "collisions", "stuck_episodes", "reloc_jumps", "spd_mean", "spd_max",
         "c0_min", "clearance_mean", "progression_mean", "reliability_mean", "loc_conf_mean", "laser_noise_mean",
         "scan_hz", "stale_pct"]
DISTINTA = {"collisions", "progression_mean"}


def lee(ruta, robot):
    filas = list(csv.DictReader(open(ruta, newline="")))
    for f in filas:
        f["robot"] = f.get("robot") or robot
        f.setdefault("arm", "")
    return filas


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--g1"); ap.add_argument("--summit")
    ap.add_argument("--salida", default="comparacion_runs.csv"); ap.add_argument("--por", default="robot,result")
    a = ap.parse_args()
    filas = (lee(a.g1, "g1") if a.g1 else []) + (lee(a.summit, "summit_xl") if a.summit else [])
    if not filas:
        ap.error("ni --g1 ni --summit")
    cols = ["robot"] + [c for c in dict.fromkeys(k for f in filas for k in f) if c != "robot"]
    with open(a.salida, "w", newline="") as fo:
        w = csv.DictWriter(fo, fieldnames=cols); w.writeheader(); w.writerows(filas)
    print("%d runs -> %s\n" % (len(filas), a.salida))
    por = a.por.split(","); grupos = defaultdict(list)
    for f in filas:
        grupos[tuple(f.get(k, "") for k in por)].append(f)
    print("%-28s %4s  " % (" / ".join(por), "n") + " ".join("%11s" % (c[:10] + ("*" if c in DISTINTA else "")) for c in COLS_RESUMEN))
    for g in sorted(grupos):
        fs = grupos[g]; out = []
        for c in COLS_RESUMEN:
            v = [x for x in (num(f.get(c)) for f in fs) if x is not None]
            out.append("%11s" % ("%.3g" % statistics.mean(v) if v else "·"))
        print("%-28s %4d  " % (" / ".join(g)[:28], len(fs)) + " ".join(out))
    print("\n* se llaman igual y NO miden igual: ver la cabecera de este script y el campo 'defs' de cada JSON.")


if __name__ == "__main__":
    main()
