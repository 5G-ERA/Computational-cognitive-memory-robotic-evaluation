"""Distribuciones reales de las senales que van a gobernar la resolucion de rol.
Los umbrales del resolutor deben salir de aqui, no de la intuicion."""
import glob, json, os, statistics
RAIZ = "/home/ros/Documents/G1_UNITREE_ROBOT_META_REASONING/dataset"
def carga(f):
    try: return json.load(open(f))
    except Exception: return None
def q(v, p):
    v = sorted(v); return v[min(len(v)-1, int(p*len(v)))]

campos = ["bat","cov_missing","cov_blind","cov_def","cov_n","laser_trust","illum_b",
          "perc_n","perc_age","c0_hard","iface_q","door_contra","color_near"]
vals = {c: [] for c in campos}
n_muestras = 0; n_runs = 0; presencia = {c: 0 for c in campos}
for f in sorted(glob.glob(os.path.join(RAIZ, "2026*_ours_[AB].json"))):
    d = carga(f)
    if not d or d.get("sim_id") is not None: continue
    ss = d.get("samples") or []
    if len(ss) < 30: continue
    n_runs += 1
    for m in ss:
        n_muestras += 1
        for c in campos:
            v = m.get(c)
            if isinstance(v, (int, float)):
                vals[c].append(float(v)); presencia[c] += 1

print("runs reales: %d | muestras: %d\n" % (n_runs, n_muestras))
print("%-14s %7s %8s %8s %8s %8s %8s" % ("campo","presente","p5","p25","mediana","p75","p95"))
for c in campos:
    v = vals[c]
    if not v:
        print("%-14s %6.0f%%   (nunca numerico)" % (c, 0)); continue
    print("%-14s %6.0f%% %8.2f %8.2f %8.2f %8.2f %8.2f"
          % (c, 100.0*presencia[c]/n_muestras, q(v,.05), q(v,.25), statistics.median(v), q(v,.75), q(v,.95)))

# detecciones de objeto: que hay realmente
det_n = 0; conf = []
for f in sorted(glob.glob(os.path.join(RAIZ, "2026*_ours_[AB].json"))):
    d = carga(f)
    if not d or d.get("sim_id") is not None: continue
    for m in (d.get("samples") or []):
        for z in (m.get("dets") or []):
            if z and z[0] != "door":
                det_n += 1
                if isinstance(z[1], (int,float)): conf.append(float(z[1]))
if conf:
    print("\ndetecciones de objeto (no-puerta): %d | conf p25 %.2f mediana %.2f p75 %.2f"
          % (det_n, q(conf,.25), statistics.median(conf), q(conf,.75)))
