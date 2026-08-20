#!/usr/bin/env python3
"""Compara los estadisticos de barrido de unos runs (p.ej. gemelo con/sin marcha) contra los
objetivos REALES medidos en marcha (8 runs del 20-ago, muestras con spd>0.1).

Uso:  python3 analysis/compara_marcha.py dataset/2026XXXX_*.json
"""
import json
import sys

REAL = {  # (mediana, p90) medidos en el robot real ANDANDO, 20-ago-2026
    "laser_noise": (0.173, 0.586),
    "c0_std": (0.087, 0.364),
    "scan_churn": (0.400, 0.494),
    "filt_rej": (0.054, 0.093),
    "loc_conf": (0.964, 1.000),
}


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    vals = {k: [] for k in REAL}
    runs = 0
    resultados = []
    for f in sys.argv[1:]:
        try:
            d = json.load(open(f))
        except Exception:
            continue
        runs += 1
        resultados.append((f.split("/")[-1][9:15],
                           (d.get("summary") or {}).get("time_s"),
                           (d.get("summary") or {}).get("collisions"),
                           d.get("result")))
        for m in d.get("samples") or []:
            if not (isinstance(m.get("spd"), (int, float)) and m["spd"] > 0.1):
                continue
            for k in vals:
                v = m.get(k)
                if isinstance(v, (int, float)):
                    vals[k].append(v)
    print("runs: %d" % runs)
    for r in resultados:
        print("  %s  %ss  col=%s  %s" % r)
    print("\n%-12s %10s %10s   %s" % ("campo", "mediana", "p90", "objetivo real (med, p90)"))
    for k in REAL:
        v = sorted(vals[k])
        if not v:
            print("%-12s %10s %10s   (%.3f, %.3f)" % (k, "-", "-", *REAL[k]))
            continue
        med = v[len(v) // 2]
        p90 = v[int(0.9 * (len(v) - 1))]
        print("%-12s %10.3f %10.3f   (%.3f, %.3f)" % (k, med, p90, *REAL[k]))


if __name__ == "__main__":
    main()
