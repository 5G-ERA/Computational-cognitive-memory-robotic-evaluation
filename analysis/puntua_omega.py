import os
"""A_meta y A_Omega, lado a lado, sobre las runs escenificadas con certificado."""
import json, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src"))
from dcc_conditions import evalua_todas, usa_pose_para, CONDICIONES
from dcc_omega import carga_referencia, puntua_run

RAIZ = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "dataset")
CASOS = [
    ("T5", "20260824_124430_ours_B.json"),
    ("T6", "20260824_124545_ours_A.json"),
    ("T3", "20260824_121500_ours_A.json"),
    ("T7", "20260824_120632_ours_A.json"),
    ("T8", "20260824_141540_ours_B.json"),
]

tot = {c: {"meta": 0, "omega": 0, "n": 0} for c in CONDICIONES}
for cfg, f in CASOS:
    run = json.load(open(RAIZ + "/" + f))
    if len(run.get("samples") or []) < 30:
        print("%s: run vacia, descartada" % cfg); continue
    segs = carga_referencia("/tmp/g1_omega_ref_%s.json" % cfg, run)
    print("\n%s · %s · %d muestras · segmentos: %s"
          % (cfg, f, len(run["samples"]),
             " -> ".join("%s@%.0fs" % (s["delta"], s["desde"]) for s in segs)))
    r = puntua_run(run, segs, evalua_todas, usa_pose_para(run))
    print("   %-4s %8s %9s" % ("", "A_meta", "A_Omega"))
    for c in CONDICIONES:
        a = r.get(c, {"meta": 0, "omega": 0, "n": 0})
        for k in ("meta", "omega", "n"):
            tot[c][k] += a[k]
        if a["n"]:
            print("   %-4s %7.0f%% %8.0f%%" % (c, 100.0*a["meta"]/a["n"], 100.0*a["omega"]/a["n"]))

print("\n=== agregado (n=%d fronteras por condicion) ===" % tot["C1"]["n"])
print("   %-4s %8s %9s   %s" % ("", "A_meta", "A_Omega", "meta-Omega (aciertos sin fundamento)"))
for c in CONDICIONES:
    a = tot[c]
    if not a["n"]: continue
    m, o = 100.0*a["meta"]/a["n"], 100.0*a["omega"]/a["n"]
    print("   %-4s %7.0f%% %8.0f%%   %.0f pp" % (c, m, o, m - o))
print("\ncontrastes A_Omega:  C4-C3 %+.0f pp   C4-C2 %+.0f pp   C4-C1 %+.0f pp"
      % tuple(100.0*(tot[x]["omega"]/tot[x]["n"] - tot[y]["omega"]/tot[y]["n"])
              for x, y in (("C4","C3"), ("C4","C2"), ("C4","C1"))))
