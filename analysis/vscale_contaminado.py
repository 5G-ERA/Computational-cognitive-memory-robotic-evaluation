import os
"""¿Calibre VSCALE contra un objetivo contaminado?

VSCALE se ajusto para igualar v/cmd, con v = camino/duracion. Si el camino real esta
inflado por temblor de pose, el objetivo 0.82 estaba inflado y VSCALE=1.20 es demasiado alto.
Aqui se recalcula v/cmd a varias decimaciones, en real y en gemelo.
"""
import glob, json, math, os, statistics
RAIZ = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "dataset")
def carga(f):
    try: return json.load(open(f))
    except Exception: return None

def vcmd(ss, k):
    pts = ss[::k]
    L = sum(math.hypot(b["x"]-a["x"], b["y"]-a["y"]) for a, b in zip(pts, pts[1:]))
    T = ss[-1]["t"] - ss[0]["t"]
    mag = [math.hypot(float((m.get("cmd") or [0,0,0])[0]), float((m.get("cmd") or [0,0,0])[1]))
           for m in ss if m.get("cmd")]
    if T < 5 or not mag: return None
    mm = statistics.mean(mag)
    return (L/T)/mm if mm > 1e-6 else None

def grupo(fs, et):
    out = {k: [] for k in (1, 3, 8, 12)}
    for f in fs:
        d = carga(f)
        if not d: continue
        ss = d.get("samples") or []
        if len(ss) < 60: continue
        for k in out:
            v = vcmd(ss, k)
            if v: out[k].append(v)
    if not out[1]:
        print("%-22s sin datos" % et); return None
    print("%-22s n=%3d   " % (et, len(out[1])) + "  ".join(
        "k=%-2d %.2f" % (k, statistics.median(out[k])) for k in (1, 3, 8, 12)))
    return {k: statistics.median(out[k]) for k in out}

reales = [f for f in sorted(glob.glob(os.path.join(RAIZ, "2026*_ours_[AB].json")))
          if carga(f) and carga(f).get("sim_id") is None
          and not os.path.basename(f).startswith("2026082")]
gem = sorted(glob.glob(os.path.join(RAIZ, "20260824_09*_ours_[AB].json")))
print("v/cmd = (camino/duracion) / |cmd| medio, a varias decimaciones\n")
r = grupo(reales, "REAL")
g = grupo(gem, "GEMELO (VSCALE=1.20)")
if r and g:
    print()
    for k in (1, 3, 8, 12):
        factor = r[k]/g[k]
        print("  k=%-2d  objetivo real %.2f | gemelo %.2f | VSCALE implicado = 1.20 x %.2f = %.2f"
              % (k, r[k], g[k], factor, 1.20*factor))
