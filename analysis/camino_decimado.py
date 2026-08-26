import os
"""¿El gemelo maniobra menos, o el camino REAL esta inflado por temblor de pose?

path_m se calcula como suma de |delta pose| entre muestras consecutivas, y esa suma la
infla cualquier ruido de pose: con ~200 muestras, 2 cm de temblor por muestra anaden 4 m
de camino que nadie recorrio. La diferencia que hay que explicar es 12.83 - 8.89 = 3.94 m.

Prueba: recalcular el camino DECIMANDO (1 de cada k muestras). El desplazamiento real es
invariante a la decimacion; el ruido se cancela y el camino se desploma. Si el camino real
cae mucho al decimar y el del gemelo no, entonces el gemelo no maniobra menos -- la medida
del real esta inflada, y mi hipotesis del "mundo demasiado limpio" es falsa.
"""
import glob, json, math, os, statistics

RAIZ = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "dataset")
def carga(f):
    try: return json.load(open(f))
    except Exception: return None

def camino(ss, k):
    pts = ss[::k]
    return sum(math.hypot(b["x"]-a["x"], b["y"]-a["y"]) for a, b in zip(pts, pts[1:]))

def grupo(fs, et):
    filas = []
    for f in fs:
        d = carga(f)
        if not d: continue
        ss = d.get("samples") or []
        if len(ss) < 60: continue
        base = camino(ss, 1)
        if base < 2: continue
        filas.append([camino(ss, k) for k in (1, 2, 3, 5, 8, 12)])
    if not filas:
        print("%-24s sin datos" % et); return None
    med = [statistics.median(x[i] for x in filas) for i in range(6)]
    print("%-24s n=%3d   " % (et, len(filas)) + "  ".join("k=%-2d %5.2f" % (k, m)
          for k, m in zip((1,2,3,5,8,12), med)))
    return med

print("camino mediano (m) recalculado decimando 1 de cada k muestras\n")
reales = [f for f in sorted(glob.glob(os.path.join(RAIZ, "2026*_ours_[AB].json")))
          if carga(f) and carga(f).get("sim_id") is None
          and not os.path.basename(f).startswith("2026082")]
gem = [f for f in sorted(glob.glob(os.path.join(RAIZ, "20260823_19*_ours_[AB].json")))
       + sorted(glob.glob(os.path.join(RAIZ, "20260824_09*_ours_[AB].json")))]
mr = grupo(reales, "REAL")
mg = grupo(gem, "GEMELO")
if mr and mg:
    print()
    print("caida del camino de k=1 a k=12:   real %.0f%%   gemelo %.0f%%"
          % (100.0*(mr[0]-mr[5])/mr[0], 100.0*(mg[0]-mg[5])/mg[0]))
    print("razon gemelo/real:   con k=1  %.2f      con k=12  %.2f" % (mg[0]/mr[0], mg[5]/mr[5]))
    print()
    print("si la razon sube al decimar, la brecha era MEDIDA (ruido de pose), no comportamiento.")
