"""Los secundarios del §9.3 sobre las cinco configuraciones escenificadas."""
import json, statistics, sys
sys.path.insert(0, "/home/ros/Documents/G1_UNITREE_ROBOT_META_REASONING")
from dcc_conditions import evalua_todas, usa_pose_para, CONDICIONES
from dcc_omega import carga_referencia, delta_muestra
from dcc_secundarios import puntua_secundarios

RAIZ = "/home/ros/Documents/G1_UNITREE_ROBOT_META_REASONING/dataset"
CASOS = [
    ("T5", "20260824_124430_ours_B.json"),
    ("T6", "20260824_124545_ours_A.json"),
    ("T3", "20260824_121500_ours_A.json"),
    ("T7", "20260824_120632_ours_A.json"),
    ("T8", "20260824_141540_ours_B.json"),
]

agg = {c: {"retardo": [], "perdidas": 0, "fronteras": 0, "persistencia": [],
           "ret_ok": 0, "ret_n": 0, "innec": []} for c in CONDICIONES}
for cfg, f in CASOS:
    run = json.load(open(RAIZ + "/" + f))
    if len(run.get("samples") or []) < 30:
        continue
    segs = carga_referencia("/tmp/g1_omega_ref_%s.json" % cfg, run)
    r = puntua_secundarios(run, segs, evalua_todas, usa_pose_para(run), delta_muestra, CONDICIONES)
    n_f = r["C4"]["n_fronteras"] if "C4" in r else 0
    print("%s: %d fronteras de referencia (guion + geometricas)" % (cfg, n_f))
    for c in CONDICIONES:
        a = agg[c]; x = r.get(c)
        if not x: continue
        a["retardo"] += x["retardo"]; a["perdidas"] += x["perdidas"]
        a["fronteras"] += x["n_fronteras"]; a["persistencia"] += x["persistencia"]
        a["ret_ok"] += x["retorno_ok"]; a["ret_n"] += x["retorno_n"]
        a["innec"].append(x["innecesarias_por_min"])

print("\n=== agregado (5 configuraciones) ===")
print("%-4s %10s %14s %12s %14s %12s %16s" % ("", "fronteras", "adoptadas", "PERDIDAS",
      "retardo med", "retorno", "innec/min"), end="")
print("%14s" % "persist med")
for c in CONDICIONES:
    a = agg[c]
    n_ad = len(a["retardo"])
    print("%-4s %10d %11d/%d %12d %13s %12s %15.1f"
          % (c, a["fronteras"], n_ad, a["fronteras"], a["perdidas"],
             ("%.1f s" % statistics.median(a["retardo"])) if a["retardo"] else "-",
             ("%d/%d" % (a["ret_ok"], a["ret_n"])) if a["ret_n"] else "-",
             statistics.median(a["innec"]) if a["innec"] else 0), end="")
    print("%13s" % (("%.1f s" % statistics.median(a["persistencia"])) if a["persistencia"] else "-"))
print("\nconstantes instrumentales declaradas, restables del retardo: EMA del contrato ~0.9 s;")
print("estabilizador en vuelo +0.5 s (aqui se puntua el resolutor puro, sin estabilizador).")
