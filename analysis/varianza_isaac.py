"""¿Esta calibrada la VARIANZA del gemelo, no solo su mediana?

Resuelve la salvedad del analisis de realismo ("4 runs afinados contra 132 reales").
Isaac = los tramos listados en dataset/campana_isaac.json (la campana N=30, sin afinar
nada por tramo). Real = todos los runs sin sim_id. Para cada metrica:
  - mediana sim dentro del IQR real (el criterio de siempre), y ademas
  - anchura del IQR sim frente a la real: razon en [0.4, 2.5] = dispersion comparable.
Las metricas POR MUESTRA (velocidad, holgura...) se reducen a UNA mediana por run antes
de comparar, para que la dispersion sea entre-runs y no entre-instantes.
"""
import collections, glob, json, math, os, statistics

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
lista = json.load(open(os.path.join(RAIZ, "dataset", "campana_isaac.json")))
ISAAC_FICH = {r["fichero"] if isinstance(r, dict) else r for r in lista["runs"]}
print("tramos de la campana:", len(ISAAC_FICH))

DX, DY = -3.90, 1.25
AX = math.radians(135.0); UX, UY = math.cos(AX), math.sin(AX)
def lat(p): return -(p["x"]-DX)*UY + (p["y"]-DY)*UX
def srob(p): return (p["x"]-DX)*UX + (p["y"]-DY)*UY

datos = collections.defaultdict(lambda: collections.defaultdict(list))
n_runs = collections.Counter()
for f in sorted(glob.glob(os.path.join(RAIZ, "dataset", "2026*_ours_[AB].json"))):
    b = os.path.basename(f)
    try:
        d = json.load(open(f))
    except Exception:
        continue
    ss = d.get("samples") or []
    if len(ss) < 20:
        continue
    if b in ISAAC_FICH:
        g = "ISAAC"
    elif d.get("sim_id") is None:
        g = "REAL"
    else:
        continue
    n_runs[g] += 1
    s = d.get("summary") or {}
    D = datos[g]
    if s.get("time_s"): D["duracion (s)"].append(float(s["time_s"]))
    D["colisiones"].append(float(s.get("collisions") or 0))
    dd = [z for m in ss for z in (m.get("dets") or []) if z[0] != "door"]
    D["detecciones por run"].append(len(dd))
    if dd:
        cf = [z[1] for z in dd if isinstance(z[1], (int, float))]
        if cf: D["confianza de objeto"].append(statistics.median(cf))
    D["% muestras con deteccion"].append(
        100.0 * sum(1 for m in ss if any(z[0] != "door" for z in (m.get("dets") or []))) / len(ss))
    vs = [m["spd"] for m in ss if isinstance(m.get("spd"), (int, float))]
    cs = [m["c0"] for m in ss if isinstance(m.get("c0"), (int, float))]
    ns = [m["nobs"] for m in ss if isinstance(m.get("nobs"), (int, float))]
    if vs: D["velocidad (m/s)"].append(statistics.median(vs))
    if cs: D["holgura c0 (m)"].append(statistics.median(cs))
    if ns: D["obstaculos vistos"].append(statistics.median(ns))
    vano = [abs(lat(m)) for m in ss if abs(srob(m)) <= 0.25]
    if vano: D["|lateral| en vano (m)"].append(statistics.median(vano))
    fs = collections.Counter()
    for m in ss:
        ph = str(m.get("phase", "")).replace("AGR-", "").split("!")[0].strip("~")
        fs[ph.split("-")[0]] += 1
    tot = sum(fs.values()) or 1
    for k in ("DWA", "ENG", "GO", "BRK"):
        D["fase %% %s" % k].append(100.0 * fs.get(k, 0) / tot)

def q(v, p):
    v = sorted(v)
    return v[min(len(v)-1, int(p*len(v)))]

print("runs comparados:", dict(n_runs))
print()
print("%-24s %-24s %-24s %-10s %s" % ("metrica", "REAL med [IQR]", "ISAAC med [IQR]", "IQRs/IQRr", "veredicto"))
print("-" * 104)
METRICAS = ["duracion (s)", "colisiones", "velocidad (m/s)", "holgura c0 (m)",
            "obstaculos vistos", "|lateral| en vano (m)", "detecciones por run",
            "% muestras con deteccion", "confianza de objeto",
            "fase % DWA", "fase % ENG", "fase % GO", "fase % BRK"]
med_ok = disp_ok = ambas = tot_m = 0
for k in METRICAS:
    r, i = datos["REAL"][k], datos["ISAAC"][k]
    if len(r) < 8 or len(i) < 8:
        print("%-24s (datos insuficientes: real %d, isaac %d)" % (k, len(r), len(i)))
        continue
    tot_m += 1
    rm, r1, r3 = statistics.median(r), q(r, .25), q(r, .75)
    im, i1, i3 = statistics.median(i), q(i, .25), q(i, .75)
    wr, wi = r3 - r1, i3 - i1
    okm = r1 <= im <= r3
    razon = (wi / wr) if wr > 1e-9 else (float("inf") if wi > 1e-9 else 1.0)
    okd = 0.4 <= razon <= 2.5
    med_ok += okm; disp_ok += okd; ambas += (okm and okd)
    v = "DENTRO+disp" if (okm and okd) else ("mediana si, disp no" if okm else
        ("disp si, mediana no" if okd else "FUERA"))
    print("%-24s %-24s %-24s %-10s %s" % (
        k, "%.2f [%.2f-%.2f]" % (rm, r1, r3), "%.2f [%.2f-%.2f]" % (im, i1, i3),
        ("%.2f" % razon) if razon != float("inf") else "inf", v))
print("-" * 104)
print("mediana dentro del IQR real: %d/%d | dispersion comparable: %d/%d | ambas: %d/%d"
      % (med_ok, tot_m, disp_ok, tot_m, ambas, tot_m))
