import os
"""La prueba que decide el jueves: el gate NO usa la luma cruda, usa la EMA
    illum_ema = 0.8*illum_ema + 0.2*luma          (g1_goto.py:2004)
y dispara con  illum_ema > 100  (g1_goto.py:2442).

Asi que se reproduce esa EMA exactamente sobre la secuencia de fotogramas reales de cada
run, con la etiqueta de luz del cuaderno del operador, y se pregunta si el umbral 100
clasifica bien. Es la unica pregunta que importa: no "sigue la media a la luz en general",
sino "acierta ESTE gate con ESTE umbral sobre ESTOS fotogramas".
"""
import glob, os, statistics
import numpy as np
from PIL import Image

RAIZ = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "dataset")
UMBRAL = 100.0
DECLARADO = [
    ("20260821_144604_ours_B", "OSCURO"),
    ("20260821_145228_ours_A", "OSCURO"),
    ("20260821_155536_ours_B", "MIXTO"),
    ("20260821_160208_ours_A", "ILUMIN"),
]
def luma(p):
    return float(np.asarray(Image.open(p).convert("L"), dtype=np.float64).mean())

print("EMA del gate (alpha 0.2 sobre el nuevo fotograma), umbral %.0f\n" % UMBRAL)
print("%-26s %-8s %4s %9s %9s %9s   %s" % ("run","estado","n","ema_ini","ema_med","ema_fin","% muestras con gate ACTIVO"))
resumen = {}
for run, estado in DECLARADO:
    fs = [f for f in sorted(glob.glob(os.path.join(RAIZ, run + "_t*.jpg"))) if os.path.getsize(f) > 2500]
    if not fs: continue
    ema = None; serie = []
    for f in fs:
        l = luma(f)
        ema = l if ema is None else 0.8*ema + 0.2*l
        serie.append(ema)
    act = 100.0*sum(1 for v in serie if v > UMBRAL)/len(serie)
    resumen.setdefault(estado, []).extend(serie)
    print("%-26s %-8s %4d %9.1f %9.1f %9.1f   %5.0f%%" % (run, estado, len(serie), serie[0],
          statistics.median(serie), serie[-1], act))

print("\n--- lo que el gate DEBERIA hacer ---")
print("  OSCURO -> gate INACTIVO (la vision gobierna; en oscuro es fiable y el eje del mapa centra bien)")
print("  ILUMIN -> gate ACTIVO   (la vision se suprime: es cuando iba sesgada +9 grados)")
print("\n--- lo que hace ---")
for e in ("OSCURO","MIXTO","ILUMIN"):
    if e not in resumen: continue
    v = resumen[e]
    act = 100.0*sum(1 for x in v if x > UMBRAL)/len(v)
    print("  %-7s ema %6.1f [%5.1f-%5.1f]   gate activo el %3.0f%% del tiempo" % (e, statistics.median(v), min(v), max(v), act))
o = resumen.get("OSCURO", []); l = resumen.get("ILUMIN", [])
if o and l:
    fp = 100.0*sum(1 for x in o if x > UMBRAL)/len(o)      # suprime cuando no debia
    fn = 100.0*sum(1 for x in l if x <= UMBRAL)/len(l)     # no suprime cuando debia
    print("\n  falso positivo (suprime la vision en OSCURO): %.0f%%" % fp)
    print("  falso negativo (la deja gobernar en ILUMIN):    %.0f%%" % fn)
    # mejor umbral posible con este estadistico
    mejor = None
    for u in [x/2.0 for x in range(120, 280)]:
        err = (sum(1 for x in o if x > u) + sum(1 for x in l if x <= u))/(len(o)+len(l))
        if mejor is None or err < mejor[0]: mejor = (err, u)
    print("  mejor umbral alcanzable con la EMA de luma: %.1f, con %.0f%% de error irreducible" % (mejor[1], 100*mejor[0]))
