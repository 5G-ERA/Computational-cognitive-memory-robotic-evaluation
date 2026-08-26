"""¿A que distancia del cristal declarado se degrada de verdad la cobertura?

La zona de exigibilidad (2.0 m) la puse a ojo, y de ahi sale el residuo de T1/T2. Aqui se
mide: distancia de cada muestra al rectangulo de cristal declarado frente a cov_n. La zona
correcta es donde cov_n cae de verdad, no donde yo supuse.
"""
import json, math, os, statistics, sys
sys.path.insert(0, "/home/ros/Documents/G1_UNITREE_ROBOT_META_REASONING")
from dcc_omega import carga_referencia
RAIZ = "/home/ros/Documents/G1_UNITREE_ROBOT_META_REASONING"
MAN = os.path.join(RAIZ, "tasks/manifiestos/campana_dcc.txt")

def dist_rect(x, y, r):
    x0, y0, x1, y1 = r
    cx = min(max(x, min(x0, x1)), max(x0, x1))
    cy = min(max(y, min(y0, y1)), max(y0, y1))
    return math.hypot(x - cx, y - cy)

lineas = [l.strip().split("|") for l in open(MAN) if "|" in l and "COMPLETA" not in l]
t12 = [l for l in lineas if l[0] in ("T1", "T2") and l[2]][-6:]
print("runs T1/T2 de luz baja:", len(t12))

por_bin = {}
for cfg, _, f in t12:
    fp = os.path.join(RAIZ, f); refp = fp.replace(".json", "_omega_ref.json")
    if not os.path.exists(refp): continue
    d = json.load(open(fp))
    segs = carga_referencia(refp, d)
    for m in d.get("samples") or []:
        t = m.get("t")
        seg = next((s for s in segs if s["desde"] <= t < s["hasta"]), None)
        if not seg: continue
        cr = str((seg.get("estado") or {}).get("cristal") or "")
        if not cr.strip(): continue
        try:
            r = tuple(float(v) for v in cr.split(","))
        except ValueError:
            continue
        cn = m.get("cov_def")
        if not isinstance(cn, (int, float)): continue
        dd = dist_rect(m.get("x", 0), m.get("y", 0), r)
        b = min(int(dd / 0.5) * 0.5, 4.0)
        por_bin.setdefault(b, []).append(cn)

print("\ndistancia al cristal declarado  ->  cov_def (predichos por el mapa y AUSENTES)")
print("%-12s %6s %8s %8s %8s   %s" % ("bin (m)", "n", "mediana", "p25", "p75", "% sobre 0.5"))
for b in sorted(por_bin):
    v = por_bin[b]
    if len(v) < 15: continue
    v_s = sorted(v)
    print("%-12s %6d %8.0f %8.0f %8.0f   %5.0f%%"
          % ("%.1f-%.1f" % (b, b + 0.5), len(v), statistics.median(v),
             v_s[len(v)//4], v_s[3*len(v)//4], 100.0*sum(1 for x in v if x > 0.5)/len(v)))
print("\ncov_def alto = el mapa predice retorno y el barrido no lo da (la firma del cristal)")
