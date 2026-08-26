"""¿Es realista el gemelo Isaac frente a TODO el historico real? Comparacion distribucional.

Real = los 132 runs sin sim_id. Isaac = los conducidos hoy a traves del puente. Para cada
metrica se dan mediana e intervalo intercuartilico, y se marca si Isaac cae DENTRO del IQR
real (que es el criterio honesto: no basta con parecerse la mediana).
"""
import json, glob, math, os, statistics
import collections

ISAAC = {"193744", "193918", "194006", "194104"}   # mapa limpio + realismo temporal +
                                                   # emulador atenuado por el canal WebRTC
GAZEBO = {"172340","181643","181739"}

DX, DY = -3.90, 1.25
AX = math.radians(135.0); UX, UY = math.cos(AX), math.sin(AX)
def lat(p): return -(p["x"]-DX)*UY + (p["y"]-DY)*UX
def srob(p): return (p["x"]-DX)*UX + (p["y"]-DY)*UY

def clasifica(f):
    b = os.path.basename(f)
    if not b.startswith("20260822_"):
        return "REAL" if json.load(open(f)).get("sim_id") is None else None
    t = b.split("_")[1]
    if t in ISAAC: return "ISAAC"
    if t in GAZEBO: return "GAZEBO"
    return None

datos = collections.defaultdict(lambda: collections.defaultdict(list))
n_runs = collections.Counter()
for f in sorted(glob.glob("dataset/2026*_ours_[AB].json")):
    try:
        d = json.load(open(f))
    except Exception:
        continue
    ss = d.get("samples") or []
    if len(ss) < 20:
        continue
    if d.get("sim_id") is not None and not os.path.basename(f).startswith("20260822_"):
        continue
    g = clasifica(f)
    if not g:
        continue
    n_runs[g] += 1
    s = d.get("summary") or {}
    D = datos[g]
    if s.get("time_s"): D["duracion (s)"].append(float(s["time_s"]))
    if s.get("err_m") is not None: D["error de llegada (m)"].append(float(s["err_m"]))
    D["colisiones"].append(float(s.get("collisions") or 0))
    dd = [z for m in ss for z in (m.get("dets") or []) if z[0] != "door"]
    D["detecciones por run"].append(len(dd))
    D["confianza de objeto"] += [z[1] for z in dd if isinstance(z[1], (int, float))]
    D["%% muestras con deteccion"].append(
        100.0 * sum(1 for m in ss if any(z[0] != "door" for z in (m.get("dets") or []))) / len(ss))
    for m in ss:
        if isinstance(m.get("spd"), (int, float)): D["velocidad (m/s)"].append(m["spd"])
        if isinstance(m.get("c0"), (int, float)): D["holgura c0 (m)"].append(m["c0"])
        if isinstance(m.get("nobs"), (int, float)): D["obstaculos vistos"].append(m["nobs"])
    vano = [abs(lat(m)) for m in ss if abs(srob(m)) <= 0.25]
    if vano: D["|lateral| en vano (m)"].append(statistics.median(vano))
    # reparto de fases (que hace el resolutor)
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
print("%-24s %-22s %-22s %s" % ("metrica", "REAL (med [IQR])", "ISAAC (med [IQR])", "veredicto"))
print("-" * 90)
METRICAS = ["duracion (s)", "colisiones", "velocidad (m/s)",
            "holgura c0 (m)", "obstaculos vistos", "|lateral| en vano (m)",
            "detecciones por run", "%% muestras con deteccion", "confianza de objeto",
            "fase % DWA", "fase % ENG", "fase % GO", "fase % BRK"]
dentro = fuera = 0
for k in METRICAS:
    r, i = datos["REAL"][k], datos["ISAAC"][k]
    if not r or not i:
        continue
    rm, r1, r3 = statistics.median(r), q(r, .25), q(r, .75)
    im, i1, i3 = statistics.median(i), q(i, .25), q(i, .75)
    ok = r1 <= im <= r3
    dentro += ok; fuera += (not ok)
    print("%-24s %-22s %-22s %s" % (
        k, "%.2f [%.2f-%.2f]" % (rm, r1, r3), "%.2f [%.2f-%.2f]" % (im, i1, i3),
        "DENTRO del IQR real" if ok else "fuera (x%.1f)" % (im/rm if rm else 0)))
print()
print("RESUMEN: %d metricas dentro del rango intercuartilico real, %d fuera" % (dentro, fuera))
