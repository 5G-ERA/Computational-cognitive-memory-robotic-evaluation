"""Aplica el FILTRO DE LA APP (medido en 1444 snapshots reales) al scan sim, y re-compara.

Modelo del filtro, con sus numeros medidos:
  1. CAPA DE RANGO: nada mas alla de 3.7 m (max real observado; 94% < 3 m).
  2. RETENCION por banda de rango, calibrada para reproducir el histograma real
     {0-1m: 12%, 1-2m: 45%, 2-3m: 37%, 3+m: 6%} con presupuesto ~62 sectores/snapshot.
  3. El perfil "real" en A se construyo agregando 9 snapshots (sector si >=3 lo vieron);
     al sim se le hace EXACTAMENTE lo mismo: 9 extracciones aleatorias del filtro y
     misma regla de agregacion. Manzanas con manzanas.
"""
import json, math, random, statistics, glob, collections

random.seed(7)
sim = {int(k): v for k, v in json.load(open("/home/ros/isaac_ws/sim_scan_A.json"))["perfil"].items()}

# --- objetivo medido ---
HIST = {0: 0.12, 1: 0.45, 2: 0.37, 3: 0.06}
BUDGET = 62
CAP = 3.7

# candidatos del sim tras la capa de rango
cand = {b: r for b, r in sim.items() if r <= CAP}
por_banda = collections.defaultdict(list)
for b, r in cand.items():
    por_banda[min(int(r), 3)].append(b)

# retencion por banda: objetivo_n / disponibles_n (saturada a 1)
ret = {}
for k in HIST:
    disp = len(por_banda.get(k, []))
    obj = HIST[k] * BUDGET
    ret[k] = min(1.0, obj / disp) if disp else 0.0
print("candidatos sim tras capa %.1fm: %d sectores" % (CAP, len(cand)))
print("retencion por banda:", {("%d-%dm" % (k, k+1)): "%.2f" % v for k, v in sorted(ret.items())})

# --- 9 pseudo-snapshots filtrados + agregacion identica a la real ---
visto = collections.Counter()
vals = collections.defaultdict(list)
for _ in range(9):
    for b, r in cand.items():
        if random.random() < ret[min(int(r), 3)]:
            visto[b] += 1
            vals[b].append(r)
sim_f = {b: statistics.median(v) for b, v in vals.items() if visto[b] >= 3}
print("sim FILTRADO agregado: %d sectores (%.0f%%)" % (len(sim_f), 100*len(sim_f)/180))

# --- perfil real en A (identico al de antes) ---
A = (0.99, 0.57)
por_sector = collections.defaultdict(list)
for f in sorted(glob.glob("dataset/2026*_ours_[AB].json")):
    try:
        d = json.load(open(f))
    except Exception:
        continue
    if d.get("sim_id"):
        continue
    for s in d.get("laser_snapshots") or []:
        if s.get("x") is None or math.hypot(s["x"]-A[0], s["y"]-A[1]) > 0.15:
            continue
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
print("real agregado en A: %d sectores (%.0f%%)" % (len(real), 100*len(real)/180))

# --- comparacion con el desfase ya conocido (232) y barrido fino por si acaso ---
mejor = None
for des in range(0, 360, 2):
    errs = [abs(sim_f[(b+des) % 360] - r) for b, r in real.items() if (b+des) % 360 in sim_f]
    if len(errs) >= 15:
        m = statistics.median(errs)
        if mejor is None or m < mejor[1]:
            mejor = (des, m, len(errs), errs)
des, med, ncom, errs = mejor
errs.sort()
print("\n=== RESULTADO PELDANO 1 (tras filtro) ===")
print("desfase %d | %d sectores comunes | mediana |dr| = %.2f m | p90 = %.2f m" % (
    des, ncom, med, errs[int(0.9*len(errs))]))
print("cobertura: real %.0f%%  sim-filtrado %.0f%%   (antes del filtro: 69%%)" % (
    100*len(real)/180, 100*len(sim_f)/180))
