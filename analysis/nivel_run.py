"""A_meta A NIVEL DE RUN — la unidad que la pre-registracion declara (§4: "la unidad
independiente es la configuracion de transicion o la run, no cada frontera registrada").

El agregado anterior ("C4 54% sobre 8855 fronteras") violaba esa regla: valido como
descriptivo, invalido como base de contraste. Aqui: A_meta por run y condicion, agregado
como mediana [IQR] entre runs, y los contrastes como DIFERENCIAS PAREADAS dentro de cada
run (cada run rinde las cuatro condiciones sobre las mismas muestras, asi que el
emparejamiento es exacto).
"""
import json, os, statistics, sys, collections
sys.path.insert(0, "/home/ros/Documents/G1_UNITREE_ROBOT_META_REASONING")
from dcc_conditions import evalua_todas, usa_pose_para, CONDICIONES
from dcc_omega import carga_referencia, puntua_run

RAIZ = "/home/ros/Documents/G1_UNITREE_ROBOT_META_REASONING"
MAN = os.path.join(RAIZ, "tasks/manifiestos/campana_dcc.txt")

runs = []
viejasT12 = 0
for ln in open(MAN):
    if "|" not in ln or "COMPLETA" in ln: continue
    cfg, dst, f = ln.strip().split("|")
    if not f: continue
    if cfg in ("T1", "T2") and viejasT12 < 6:
        viejasT12 += 1; continue
    fp = os.path.join(RAIZ, f); ref = fp.replace(".json", "_omega_ref.json")
    if not os.path.exists(ref): continue
    d = json.load(open(fp))
    if len(d.get("samples") or []) < 30: continue
    runs.append((cfg, d, ref))

por_run = []          # (cfg, {cond: acierto_fraccion})
for cfg, d, ref in runs:
    segs = carga_referencia(ref, d)
    r = puntua_run(d, segs, evalua_todas, usa_pose_para(d))
    fila = {}
    for c in CONDICIONES:
        a = r.get(c)
        fila[c] = a["meta"] / a["n"] if a and a["n"] else None
    por_run.append((cfg, fila))

def q(v, p):
    v = sorted(v); return v[min(len(v)-1, int(p*len(v)))]

print("runs (unidad de analisis): %d\n" % len(por_run))
print("=== agregado A NIVEL DE RUN: mediana [IQR] entre runs ===")
for c in CONDICIONES:
    v = [f[c] for _, f in por_run if f[c] is not None]
    print("  %s  %.0f%%  [%.0f%%-%.0f%%]   (n=%d runs)"
          % (c, 100*statistics.median(v), 100*q(v,.25), 100*q(v,.75), len(v)))

print("\n=== contrastes PAREADOS por run: mediana [IQR] de la diferencia, y signo ===")
for x, y in (("C4","C3"), ("C4","C2"), ("C4","C1"), ("C3","C1")):
    d_ = [f[x]-f[y] for _, f in por_run if f[x] is not None and f[y] is not None]
    pos = sum(1 for v in d_ if v > 0); neg = sum(1 for v in d_ if v < 0)
    print("  %s-%s  %+.0f pp  [%+.0f, %+.0f]   %s>%s en %d/%d runs, %s<%s en %d"
          % (x, y, 100*statistics.median(d_), 100*q(d_,.25), 100*q(d_,.75),
             x, y, pos, len(d_), x, y, neg))

print("\n=== por configuracion (mediana entre reps, C4) ===")
cfgs = collections.defaultdict(list)
for cfg, f in por_run:
    if f["C4"] is not None: cfgs[cfg].append(f["C4"])
for cfg in ("T1","T2","T3","T4","T5","T6","T7","T8","T9","T11","T10"):
    if cfg in cfgs:
        v = cfgs[cfg]
        print("  %-4s C4 mediana %.0f%%  (reps: %s)" % (cfg, 100*statistics.median(v),
              " ".join("%.0f" % (100*x) for x in sorted(v))))
