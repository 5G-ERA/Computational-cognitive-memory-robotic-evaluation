import os
"""v3: la pregunta correcta. No es "que estadistico varia entre tandas" (eso lo hace
cualquiera que siga el contenido de la escena), sino "que estadistico SEPARA el estado
iluminado del oscuro sin solaparse", que es lo unico que un umbral puede usar.

Las 8 tandas se parten en dos grupos por la bimodalidad de la media (80-88 frente a
115-118, hueco de 27). Ese es el mejor sustituto disponible del estado declarado, porque
el barrido L1-L5 no quedo etiquetado. Se declara como tal.
"""
import glob, os, statistics
import numpy as np
from PIL import Image

DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "calib_luz", "2026-08-21")
def seg(n):
    t = os.path.basename(n).split("_")[0]
    return int(t[:2])*3600 + int(t[2:4])*60 + int(t[4:6])
def stats(path):
    a = np.asarray(Image.open(path).convert("L"), dtype=np.float64)
    m = a.mean(); p1, p99 = np.percentile(a, 1), np.percentile(a, 99)
    lap = 4*a[1:-1,1:-1] - a[:-2,1:-1] - a[2:,1:-1] - a[1:-1,:-2] - a[1:-1,2:]
    return {"media": m, "rms_norm": a.std()/max(1.,m), "michelson": (p99-p1)/max(1.,p99+p1),
            "grano": float(np.abs(lap).mean()), "sombra_p1": float(p1), "brillo_p99": float(p99)}
def bloques(fs, hueco=45):
    fs = sorted(fs, key=seg)
    out, cur = [], [fs[0]]
    for f in fs[1:]:
        if seg(f) - seg(cur[-1]) <= hueco:
            cur.append(f)
        else:
            out.append(cur)
            cur = [f]          # lista NUEVA: cur.clear() mutaba la ya guardada
    out.append(cur)
    return out

fs = glob.glob(os.path.join(DIR, "*[0-9].jpg"))
bs = bloques(fs)
tandas = []
for b in bs:
    ss = [stats(f) for f in b]
    tandas.append((os.path.basename(b[0])[:6], ss, statistics.mean(s["media"] for s in ss)))

OSC = [t for t in tandas if t[2] < 100]
LIT = [t for t in tandas if t[2] >= 100]
print("grupo OSCURO: %d tandas, %d fotos, media %.1f-%.1f"
      % (len(OSC), sum(len(t[1]) for t in OSC), min(t[2] for t in OSC), max(t[2] for t in OSC)))
print("grupo ILUMIN: %d tandas, %d fotos, media %.1f-%.1f"
      % (len(LIT), sum(len(t[1]) for t in LIT), min(t[2] for t in LIT), max(t[2] for t in LIT)))
print("\n(sustituto declarado: el estado real L1-L5 no se etiqueto; la particion viene de la\n"
      " bimodalidad de la media. Si Adrian recupera las horas, se reetiqueta y se repite.)\n")

campos = ["media", "rms_norm", "michelson", "grano", "sombra_p1", "brillo_p99"]
print("%-12s %17s %17s %8s  %s" % ("estadistico", "OSCURO med[min-max]", "ILUMIN med[min-max]", "solape", "umbral util?"))
for c in campos:
    o = [s[c] for t in OSC for s in t[1]]
    l = [s[c] for t in LIT for s in t[1]]
    o_lo, o_hi = min(o), max(o); l_lo, l_hi = min(l), max(l)
    # solape de rangos por foto: cuantas fotos caen dentro del rango del otro grupo
    sol = (sum(1 for v in o if l_lo <= v <= l_hi) + sum(1 for v in l if o_lo <= v <= o_hi)) / (len(o)+len(l))
    sep = (o_hi < l_lo) or (l_hi < o_lo)
    hueco = (l_lo - o_hi) if o_hi < l_lo else ((o_lo - l_hi) if l_hi < o_lo else 0.0)
    print("%-12s %7.2f[%5.1f-%5.1f] %7.2f[%5.1f-%5.1f] %7.0f%%  %s"
          % (c, statistics.median(o), o_lo, o_hi, statistics.median(l), l_lo, l_hi, 100*sol,
             ("SI, hueco limpio de %.1f" % hueco) if sep else "no (los rangos se cruzan)"))
