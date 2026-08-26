"""Efecto de la estabilizacion: cuanto tableteo quita y CUANTO RETRASA la transicion real.

El coste hay que medirlo, no suponerlo: si la histeresis retrasa mucho la transicion
escenificada, el retardo de conmutacion que mide el banco queda dominado por mi filtro.
"""
import glob, json, os, statistics, sys
sys.path.insert(0, "/home/ros/Documents/G1_UNITREE_ROBOT_META_REASONING")
from dcc_roles import EstabilizadorRol

RAIZ = "/home/ros/Documents/G1_UNITREE_ROBOT_META_REASONING/dataset"

def transiciones(seq, ts):
    out = []
    for i in range(1, len(seq)):
        if seq[i] != seq[i-1]:
            out.append((ts[i], seq[i-1], seq[i]))
    return out

def corre(f, conf, dwell, et):
    ss = json.load(open(f)).get("samples") or []
    if len(ss) < 30: return None
    est = EstabilizadorRol(conf=conf, dwell=dwell)
    ts = []; crudos = []; estab = []
    for m in ss:
        r, _, _, c = est.paso(m)
        ts.append(m["t"]); crudos.append(c); estab.append(r)
    tc = transiciones(crudos, ts); te = transiciones(estab, ts)
    # episodios cortos (<1 s) que sobreviven
    def cortos(seq, ts):
        n = 0; t0 = ts[0]
        for i in range(1, len(seq)):
            if seq[i] != seq[i-1]:
                if ts[i] - t0 < 1.0: n += 1
                t0 = ts[i]
        return n
    return {"et": et, "n_crudo": len(tc), "n_estab": len(te),
            "cortos_crudo": cortos(crudos, ts), "cortos_estab": cortos(estab, ts),
            "tc": tc, "te": te}

F = os.path.join(RAIZ, "20260824_113950_ours_B.json")
T_DECLARADO = 20.8      # instante en que el guion declaro el cambio de luz

print("T3 escenificada · el guion declaro la transicion a t=%.1f s\n" % T_DECLARADO)
print("%-22s %10s %10s %12s %12s" % ("config", "transic.", "cortas<1s", "detecta el", "retardo"))
for conf, dwell in ((1, 0.0), (2, 1.0), (3, 1.5)):
    r = corre(F, conf, dwell, "conf=%d dwell=%.1f" % (conf, dwell))
    if not r: continue
    # ¿cuando deja de gobernar illumination tras el cambio declarado?
    sal = [t for t, a, b in r["te"] if a == "illumination" and t >= T_DECLARADO - 2]
    det = sal[0] if sal else None
    print("%-22s %10d %10d %12s %12s"
          % (r["et"], r["n_estab"], r["cortos_estab"],
             ("%.1f s" % det) if det else "-",
             ("%+.1f s" % (det - T_DECLARADO)) if det else "-"))
base = corre(F, 1, 0.0, "sin estabilizar")
print("\nsin estabilizar: %d transiciones, %d de ellas mas cortas de 1 s"
      % (base["n_crudo"], base["cortos_crudo"]))
