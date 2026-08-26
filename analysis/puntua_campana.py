"""La campana de desarrollo entera: A_meta, A_Omega y secundarios por configuracion y
condicion, con el certificado que viaja junto a cada run."""
import json, os, statistics, sys, collections
sys.path.insert(0, "/home/ros/Documents/G1_UNITREE_ROBOT_META_REASONING")
from dcc_conditions import evalua_todas, usa_pose_para, CONDICIONES
from dcc_omega import carga_referencia, delta_muestra, puntua_run
from dcc_secundarios import puntua_secundarios

RAIZ = "/home/ros/Documents/G1_UNITREE_ROBOT_META_REASONING"

runs = []
viejasT12 = 0
lineas = [l for l in open("/tmp/campana_dcc.txt") if "|" in l and "COMPLETA" not in l]
for ln in lineas:
    cfg, dst, f = ln.strip().split("|")
    if not f: continue
    # las T1/T2 de la primera tanda llevaban luz alta (guion erroneo): se descartan y las
    # sustituyen las re-corridas con luz baja anadidas al final del fichero
    if cfg in ("T1", "T2") and viejasT12 < 6:
        viejasT12 += 1; continue
    fp = os.path.join(RAIZ, f)
    ref = fp.replace(".json", "_omega_ref.json")
    if not os.path.exists(ref): continue
    d = json.load(open(fp))
    if len(d.get("samples") or []) < 30: continue
    runs.append((cfg, d, ref))
print("runs puntuables: %d" % len(runs))

# --- primarios por config x condicion ---
prim = collections.defaultdict(lambda: {c: [0, 0, 0] for c in CONDICIONES})  # meta, omega, n
sec = collections.defaultdict(lambda: {c: {"ad": 0, "fr": 0, "ret": [0, 0], "del": [], "per": [], "inn": []}
                                       for c in CONDICIONES})
for cfg, d, ref in runs:
    segs = carga_referencia(ref, d)
    up = usa_pose_para(d)
    r = puntua_run(d, segs, evalua_todas, up)
    for c in CONDICIONES:
        a = r.get(c)
        if a:
            prim[cfg][c][0] += a["meta"]; prim[cfg][c][1] += a["omega"]; prim[cfg][c][2] += a["n"]
    s2 = puntua_secundarios(d, segs, evalua_todas, up, delta_muestra, CONDICIONES)
    for c in CONDICIONES:
        x = s2.get(c)
        if not x: continue
        e = sec[cfg][c]
        e["ad"] += len(x["retardo"]); e["fr"] += x["n_fronteras"]
        e["ret"][0] += x["retorno_ok"]; e["ret"][1] += x["retorno_n"]
        e["del"] += x["retardo"]; e["per"] += x["persistencia"]
        e["inn"].append(x["innecesarias_por_min"])

orden = ["T1","T2","T3","T4","T5","T6","T7","T8","T9","T11","T10"]
print("\n=== A_meta por configuracion (n de reps entre parentesis) ===")
print("%-5s %6s %6s %6s %6s" % ("", "C1", "C2", "C3", "C4"))
reps = collections.Counter(cfg for cfg, _, _ in runs)
for cfg in orden:
    if cfg not in prim: continue
    fila = []
    for c in CONDICIONES:
        m, o, n = prim[cfg][c]
        fila.append("%.0f%%" % (100.0*m/n) if n else "-")
    print("%-5s %6s %6s %6s %6s   (%d)" % (cfg, *fila, reps[cfg]))

tot = {c: [0, 0, 0] for c in CONDICIONES}
for cfg in prim:
    for c in CONDICIONES:
        for i in range(3): tot[c][i] += prim[cfg][c][i]
print("\n=== agregado primario (%d fronteras/condicion) ===" % tot["C4"][2])
for c in CONDICIONES:
    m, o, n = tot[c]
    print("  %s  A_meta %.0f%%  A_Omega %.0f%%" % (c, 100.0*m/n, 100.0*o/n))

print("\n=== secundarios agregados ===")
print("%-4s %10s %10s %12s %10s %14s %12s" % ("", "fronteras", "adoptadas", "retardo med", "retorno", "innec/min med", "persist med"))
for c in CONDICIONES:
    fr = sum(sec[cfg][c]["fr"] for cfg in sec)
    ad = sum(sec[cfg][c]["ad"] for cfg in sec)
    ret = [sum(sec[cfg][c]["ret"][i] for cfg in sec) for i in (0, 1)]
    dl = [x for cfg in sec for x in sec[cfg][c]["del"]]
    pe = [x for cfg in sec for x in sec[cfg][c]["per"]]
    innl = [x for cfg in sec for x in sec[cfg][c]["inn"]]
    print("%-4s %10d %10d %12s %10s %14.1f %12s"
          % (c, fr, ad, ("%.1f s" % statistics.median(dl)) if dl else "-",
             "%d/%d" % (ret[0], ret[1]) if ret[1] else "-",
             statistics.median(innl) if innl else 0,
             ("%.1f s" % statistics.median(pe)) if pe else "-"))
