import os
"""La memoria de voxels reproducida OFFLINE sobre runs reales, con y sin barrido por rayos.

Se replica el bucle de g1_goto (memorizar tras K confirmaciones sanas, reinyectar dentro de
VOXMEM_R, caducar por TTL) sobre los laser_snapshots, que traen pose y celdas del barrido.

SALVEDAD DECLARADA: los snapshots son periodicos, no un tick de control, asi que la cadencia
es mas gruesa que en ejecucion. Sirve para comparar TTL-solo contra TTL+rayos en el MISMO
material, no para predecir el numero absoluto de celdas en vivo.
"""
import glob, json, math, os, statistics, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src"))
from vox_rayos import despeja

OC = 0.20; NEAR_BLIND = 0.60
TTL = 3.0; R = 1.2; K = 2

def replay(snaps, con_rayos):
    mem = {}; seen = {}
    picos = []; sostenidas = []
    por_rayo = 0; por_ttl = 0; por_purga = 0
    for s in snaps:
        t = s.get("t"); px, py = s.get("x"), s.get("y")
        if t is None or px is None:
            continue
        fresco = bool(s.get("fresh", True))
        pts = s.get("pts") or []
        live = {(round(a / OC), round(b / OC)) for a, b in pts}
        # celdas confirmadas a distancia SANA (fuera de la banda ciega)
        sanas = {c for c in live
                 if math.hypot(c[0]*OC - px, c[1]*OC - py) >= NEAR_BLIND}
        if fresco:
            for c in sanas:
                seen[c] = seen.get(c, 0) + 1
                if seen[c] >= K:
                    mem[c] = t
        # --- despeje por RAYOS: evidencia positiva de ausencia ---
        if con_rayos and fresco:
            libres, _, _ = despeja((px, py), pts, set(mem), OC, NEAR_BLIND, True)
            for c in libres:
                mem.pop(c, None); seen.pop(c, None); por_rayo += 1
        # --- caducidad y purga por barrido, como en el codigo actual ---
        vivas = 0
        for c, ts in list(mem.items()):
            if t - ts > TTL:
                mem.pop(c, None); seen.pop(c, None); por_ttl += 1; continue
            if math.hypot(c[0]*OC - px, c[1]*OC - py) > R:
                continue
            if fresco and c in live:
                por_purga += 1; continue
            vivas += 1
        picos.append(vivas)
        sostenidas.append(len(mem))
    return {"max_inyectadas": max(picos) if picos else 0,
            "med_inyectadas": statistics.median(picos) if picos else 0,
            "max_memoria": max(sostenidas) if sostenidas else 0,
            "por_rayo": por_rayo, "por_ttl": por_ttl, "por_purga": por_purga}

RAIZ = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "dataset")
fs = []
for f in sorted(glob.glob(os.path.join(RAIZ, "2026*_ours_[AB].json"))):
    try: d = json.load(open(f))
    except Exception: continue
    if d.get("sim_id") is not None: continue
    sn = d.get("laser_snapshots") or []
    if len(sn) >= 20: fs.append((f, sn))
print("runs reales con snapshots suficientes: %d" % len(fs))

tot = {"sin": [], "con": []}
sueltas = {"rayo": 0, "ttl": 0, "purga": 0}
for f, sn in fs:
    a = replay(sn, False); b = replay(sn, True)
    tot["sin"].append(a["max_inyectadas"]); tot["con"].append(b["max_inyectadas"])
    sueltas["rayo"] += b["por_rayo"]; sueltas["ttl"] += b["por_ttl"]; sueltas["purga"] += b["por_purga"]

def q(v, p):
    v = sorted(v); return v[min(len(v)-1, int(p*len(v)))]
for k, et in (("sin", "TTL solo (lo actual)"), ("con", "TTL + barrido por rayos")):
    v = tot[k]
    print("%-26s celdas inyectadas: mediana %.0f  p90 %.0f  MAXIMO %.0f"
          % (et, statistics.median(v), q(v, .90), max(v)))
n = sum(sueltas.values()) or 1
print("\nde donde viene cada liberacion (con rayos): rayo %.0f%%  |  TTL %.0f%%  |  purga por barrido %.0f%%"
      % (100.0*sueltas["rayo"]/n, 100.0*sueltas["ttl"]/n, 100.0*sueltas["purga"]/n))
peor_sin = max(tot["sin"]); i = tot["sin"].index(peor_sin)
print("\npeor run con TTL solo: %d celdas -> con rayos: %d   (%s)"
      % (peor_sin, tot["con"][i], os.path.basename(fs[i][0])))
