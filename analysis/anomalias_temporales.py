"""Las dos anomalias: duracion 2.5x mas corta y frenado 4.4x mas frecuente."""
import json, glob, math, os, statistics
import collections

ISAAC = {"174839","174935","180519","180551","180658","180739","180829","180908","180959",
         "181025","181303","181304","181327","181414","181438","181518","182803","182910",
         "182937","183111","183124","183153","183209","183237","183250","183322","183337"}

def recorrido(ss):
    d = 0.0
    for a, b in zip(ss, ss[1:]):
        if a.get("x") is None or b.get("x") is None:
            continue
        d += math.hypot(b["x"]-a["x"], b["y"]-a["y"])
    return d

R = collections.defaultdict(list)
for f in sorted(glob.glob("dataset/2026*_ours_[AB].json")):
    try:
        d = json.load(open(f))
    except Exception:
        continue
    ss = d.get("samples") or []
    if len(ss) < 20:
        continue
    b = os.path.basename(f)
    if b.startswith("20260822_"):
        g = "ISAAC" if b.split("_")[1] in ISAAC else None
    else:
        g = "REAL" if d.get("sim_id") is None else None
    if not g:
        continue
    s = d.get("summary") or {}
    t = float(s.get("time_s") or 0)
    L = recorrido(ss)
    R[g + "_t"].append(t)
    R[g + "_L"].append(L)
    if t > 0:
        R[g + "_v"].append(L / t)
    # tiempo parado (velocidad casi nula)
    sp = [m.get("spd") for m in ss if isinstance(m.get("spd"), (int, float))]
    if sp:
        R[g + "_quieto"].append(100.0 * sum(1 for v in sp if v < 0.02) / len(sp))
    # cadencia de muestreo
    ts = [m.get("t") for m in ss if isinstance(m.get("t"), (int, float))]
    if len(ts) > 5 and ts[-1] > ts[0]:
        R[g + "_hz"].append(len(ts) / (ts[-1] - ts[0]))

def m(k):
    v = R[k]
    return statistics.median(v) if v else 0

print("%-28s %12s %12s" % ("", "REAL", "ISAAC"))
print("-" * 54)
print("%-28s %12.0f %12.0f" % ("duracion (s)", m("REAL_t"), m("ISAAC_t")))
print("%-28s %12.1f %12.1f" % ("recorrido real (m)", m("REAL_L"), m("ISAAC_L")))
print("%-28s %12.3f %12.3f" % ("velocidad media (m/s)", m("REAL_v"), m("ISAAC_v")))
print("%-28s %12.0f %12.0f" % ("%% del tiempo parado", m("REAL_quieto"), m("ISAAC_quieto")))
print("%-28s %12.1f %12.1f" % ("muestras por segundo", m("REAL_hz"), m("ISAAC_hz")))
print()
print("distancia A-B en linea recta: %.1f m" % math.hypot(-4.73-0.99, 3.04-0.57))
