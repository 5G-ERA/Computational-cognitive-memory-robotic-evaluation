"""Validacion NO CIRCULAR del contrato de calidad visual.

Las etiquetas vienen del cuaderno del operador (tasks/SESSION_LOG_2026-08-21.md), no de
ninguna estadistica de imagen, asi que se puede preguntar honestamente si la media de luma
sigue a la iluminacion declarada. Los fotogramas de 1587 bytes son el placeholder que escribe
el arnes cuando el canal WebRTC no entrega: se descartan, no son imagenes.
"""
import glob, os, statistics
import numpy as np
from PIL import Image

RAIZ = "/home/ros/Documents/G1_UNITREE_ROBOT_META_REASONING/dataset"
DECLARADO = [
    ("20260821_144604_ours_B", "OSCURO", "todas las luces apagadas, oficina y laboratorio"),
    ("20260821_145228_ours_A", "OSCURO", "luces apagadas en todas las salas"),
    ("20260821_155536_ours_B", "MIXTO",  "encendidas en el laboratorio, apagadas en la oficina"),
    ("20260821_160208_ours_A", "ILUMIN", "todas las luces encendidas"),
]

def stats(p):
    a = np.asarray(Image.open(p).convert("L"), dtype=np.float64)
    m = a.mean(); p1, p99 = np.percentile(a, 1), np.percentile(a, 99)
    lap = 4*a[1:-1,1:-1] - a[:-2,1:-1] - a[2:,1:-1] - a[1:-1,:-2] - a[1:-1,2:]
    return {"media": m, "rms_norm": a.std()/max(1., m),
            "michelson": (p99-p1)/max(1., p99+p1), "grano": float(np.abs(lap).mean()),
            "brillo_p99": float(p99)}

campos = ["media", "rms_norm", "michelson", "grano", "brillo_p99"]
grupos = {}
print("%-26s %-8s %5s " % ("run", "estado", "n") + "".join("%11s" % c for c in campos))
for run, estado, desc in DECLARADO:
    fs = [f for f in sorted(glob.glob(os.path.join(RAIZ, run + "_t*.jpg")))
          if os.path.getsize(f) > 2500]
    if not fs:
        print("%-26s %-8s   sin fotogramas reales (todo placeholder)" % (run, estado)); continue
    ss = [stats(f) for f in fs]
    grupos.setdefault(estado, []).extend(ss)
    print("%-26s %-8s %5d " % (run, estado, len(ss))
          + "".join("%11.2f" % statistics.mean(s[c] for s in ss) for c in campos))

print("\n--- separacion OSCURO vs ILUMIN, etiquetas del operador ---")
print("%-12s %19s %19s %8s  %s" % ("estadistico", "OSCURO [min-max]", "ILUMIN [min-max]", "solape", "umbral util?"))
if "OSCURO" in grupos and "ILUMIN" in grupos:
    for c in campos:
        o = [s[c] for s in grupos["OSCURO"]]; l = [s[c] for s in grupos["ILUMIN"]]
        o_lo, o_hi, l_lo, l_hi = min(o), max(o), min(l), max(l)
        sol = (sum(1 for v in o if l_lo <= v <= l_hi) + sum(1 for v in l if o_lo <= v <= o_hi))/(len(o)+len(l))
        sep = o_hi < l_lo or l_hi < o_lo
        hueco = (l_lo-o_hi) if o_hi < l_lo else ((o_lo-l_hi) if l_hi < o_lo else 0.0)
        print("%-12s %8.2f[%5.1f-%5.1f] %8.2f[%5.1f-%5.1f] %7.0f%%  %s"
              % (c, statistics.median(o), o_lo, o_hi, statistics.median(l), l_lo, l_hi, 100*sol,
                 ("SI, hueco %.1f" % hueco) if sep else "no"))
    if "MIXTO" in grupos:
        mm = [s["media"] for s in grupos["MIXTO"]]
        print("\nMIXTO (una sala encendida): media %.1f [%.1f-%.1f]  -- donde cae respecto al umbral 100"
              % (statistics.median(mm), min(mm), max(mm)))
