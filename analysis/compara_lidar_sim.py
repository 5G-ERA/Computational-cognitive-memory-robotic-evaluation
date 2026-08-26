"""Peldano 1 de realismo, la METRICA: perfil lidar SIM vs REAL en el waypoint A.

Real: todos los laser_snapshots con pose a <0.15 m de A (runs reales), por sector de 2 grados
el rango minimo (primer retorno), y la MEDIANA entre snapshots. Sim: el FlatScan RTX en A.
"""
import json, glob, math, statistics
import collections

A = (0.99, 0.57)
OC_SECT = 2

# --- real ---
por_sector = collections.defaultdict(list)
n_snaps = 0
for f in sorted(glob.glob("dataset/2026*_ours_[AB].json")):
    try:
        d = json.load(open(f))
    except Exception:
        continue
    if d.get("sim_id"):
        continue
    for s in d.get("laser_snapshots") or []:
        if s.get("x") is None:
            continue
        if math.hypot(s["x"] - A[0], s["y"] - A[1]) > 0.15:
            continue
        n_snaps += 1
        mejor = {}
        for p in (s.get("pts") or []):
            dx, dy = p[0] - s["x"], p[1] - s["y"]
            r = math.hypot(dx, dy)
            if r < 0.05:
                continue
            b = int((math.degrees(math.atan2(dy, dx)) % 360.0) // OC_SECT) * OC_SECT
            if b not in mejor or r < mejor[b]:
                mejor[b] = r
        for b, r in mejor.items():
            por_sector[b].append(r)

real = {b: statistics.median(v) for b, v in por_sector.items() if len(v) >= 3}
print("snapshots reales en A:", n_snaps, " sectores reales:", len(real))

# --- sim ---
sim = {int(k): v for k, v in json.load(open("/home/ros/isaac_ws/sim_scan_A.json"))["perfil"].items()}
print("sectores sim:", len(sim))

# la orientacion del sim es arbitraria (el lidar RTX arranca en su propio cero):
# busca el desfase que MINIMIZA la mediana de |dr| en sectores comunes
mejor = None
for des in range(0, 360, 2):
    errs = []
    for b, r in real.items():
        bs = (b + des) % 360
        if bs in sim:
            errs.append(abs(sim[bs] - r))
    if len(errs) >= 20:
        m = statistics.median(errs)
        if mejor is None or m < mejor[1]:
            mejor = (des, m, len(errs))
des, med, ncom = mejor
print("\nmejor alineacion: desfase %d grados" % des)
print("RESULTADO: %d sectores comunes | mediana |dr| = %.2f m | p90 = %.2f m" % (
    ncom, med,
    sorted(abs(sim[(b + des) % 360] - r) for b, r in real.items() if (b + des) % 360 in sim)[int(0.9 * ncom)]))
cobertura_real = len(real) / (360 // OC_SECT)
cobertura_sim = len(sim) / (360 // OC_SECT)
print("cobertura: real %.0f%%  sim %.0f%%" % (100 * cobertura_real, 100 * cobertura_sim))
