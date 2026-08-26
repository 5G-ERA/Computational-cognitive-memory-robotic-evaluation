"""Validacion cruzada del filtro en B: MISMOS parametros que en A, sin retocar nada."""
import json, math, random, statistics, glob, collections

random.seed(7)
sim = {int(k): v for k, v in json.load(open("/home/ros/isaac_ws/sim_scan_B.json"))["perfil"].items()}
HIST = {0: 0.12, 1: 0.45, 2: 0.37, 3: 0.06}
BUDGET = 62
CAP = 3.7
B_ = (-4.73, 3.04)

cand = {b: r for b, r in sim.items() if r <= CAP}
por_banda = collections.defaultdict(list)
for b, r in cand.items():
    por_banda[min(int(r), 3)].append(b)
ret = {}
for k in HIST:
    disp = len(por_banda.get(k, []))
    ret[k] = min(1.0, HIST[k] * BUDGET / disp) if disp else 0.0

visto = collections.Counter(); vals = collections.defaultdict(list)
for _ in range(9):
    for b, r in cand.items():
        if random.random() < ret[min(int(r), 3)]:
            visto[b] += 1; vals[b].append(r)
sim_f = {b: statistics.median(v) for b, v in vals.items() if visto[b] >= 3}

por_sector = collections.defaultdict(list)
nsnap = 0
for f in sorted(glob.glob("dataset/2026*_ours_[AB].json")):
    try:
        d = json.load(open(f))
    except Exception:
        continue
    if d.get("sim_id"):
        continue
    for s in d.get("laser_snapshots") or []:
        if s.get("x") is None or math.hypot(s["x"]-B_[0], s["y"]-B_[1]) > 0.15:
            continue
        nsnap += 1
        mejor = {}
        for p in (s.get("pts") or []):
            dx, dy = p[0]-s["x"], p[1]-s["y"]
            r = math.hypot(dx, dy)
            if r < 0.05:
                continue
            k = int((math.degrees(math.atan2(dy, dx)) % 360) // 2) * 2
            if k not in mejor or r < mejor[k]:
                mejor[k] = r
        for k, r in mejor.items():
            por_sector[k].append(r)
real = {b: statistics.median(v) for b, v in por_sector.items() if len(v) >= 3}
print("snapshots reales en B:", nsnap, " | real agregado: %d sectores (%.0f%%)" % (len(real), 100*len(real)/180))
print("sim filtrado en B: %d sectores (%.0f%%)  [crudo: %d]" % (len(sim_f), 100*len(sim_f)/180, len(sim)))

mejor = None
for des in range(0, 360, 2):
    errs = [abs(sim_f[(b+des) % 360] - r) for b, r in real.items() if (b+des) % 360 in sim_f]
    if len(errs) >= 15:
        m = statistics.median(errs)
        if mejor is None or m < mejor[1]:
            mejor = (des, m, len(errs), sorted(errs))
if mejor:
    des, med, ncom, errs = mejor
    print("=== VALIDACION EN B ===")
    print("desfase %d | %d comunes | mediana |dr| = %.2f m | p90 = %.2f m" % (
        des, ncom, med, errs[int(0.9*len(errs))]))
else:
    print("sin sectores comunes suficientes")
