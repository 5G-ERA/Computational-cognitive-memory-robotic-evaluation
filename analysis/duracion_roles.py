"""¿Cuanto duran los episodios de rol? El umbral de confirmacion debe salir de aqui.

Si los espurios son de decimas y los genuinos de segundos, hay un valle donde cortar. Si no
hay valle, la histeresis seria arbitraria y habria que decirlo en vez de inventarse un numero.
"""
import glob, json, os, statistics, sys, collections
sys.path.insert(0, "/home/ros/Documents/G1_UNITREE_ROBOT_META_REASONING")

RAIZ = "/home/ros/Documents/G1_UNITREE_ROBOT_META_REASONING/dataset"
def carga(f):
    try: return json.load(open(f))
    except Exception: return None

def episodios(ss):
    """(rol, duracion) de cada tramo contiguo con el mismo rol."""
    out = []
    if not ss: return out
    r0 = ss[0].get("role"); t0 = ss[0]["t"]
    for m in ss[1:]:
        r = m.get("role")
        if r != r0:
            out.append((r0, m["t"] - t0)); r0 = r; t0 = m["t"]
    out.append((r0, ss[-1]["t"] - t0))
    return out

def analiza(fs, et):
    dur = []; por_rol = collections.defaultdict(list)
    for f in fs:
        d = carga(f)
        if not d: continue
        ss = d.get("samples") or []
        if len(ss) < 30 or not ss[0].get("role"): continue
        for r, t in episodios(ss):
            if r: dur.append(t); por_rol[r].append(t)
    if not dur:
        print("%-28s sin roles emitidos" % et); return
    dur.sort()
    def q(p): return dur[min(len(dur)-1, int(p*len(dur)))]
    print("\n=== %s — %d episodios ===" % (et, len(dur)))
    print("  duracion: p10 %.2fs  p25 %.2fs  mediana %.2fs  p75 %.2fs  p90 %.2fs"
          % (q(.10), q(.25), statistics.median(dur), q(.75), q(.90)))
    # histograma grueso para ver si hay valle
    cortes = [0, 0.5, 1.0, 1.5, 2.0, 3.0, 5.0, 10.0, 1e9]
    et2 = ["<0.5", "0.5-1", "1-1.5", "1.5-2", "2-3", "3-5", "5-10", ">10"]
    h = [sum(1 for x in dur if cortes[i] <= x < cortes[i+1]) for i in range(len(et2))]
    print("  " + "  ".join("%s:%d(%.0f%%)" % (e, n, 100.0*n/len(dur)) for e, n in zip(et2, h)))
    print("  por rol (mediana):  " + "  ".join(
        "%s %.1fs(n=%d)" % (r, statistics.median(v), len(v))
        for r, v in sorted(por_rol.items(), key=lambda kv: -len(kv[1]))[:6]))

analiza([os.path.join(RAIZ, "20260824_113950_ours_B.json")], "T3 escenificada (con tableteo)")
analiza(sorted(glob.glob(os.path.join(RAIZ, "20260824_11*_ours_[AB].json"))), "gemelo hoy (todas)")
