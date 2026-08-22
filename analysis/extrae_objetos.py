"""Reconstruye la posicion en el MAPA de los objetos que la camara del G1 fue viendo.

Cada muestra lleva pose (x,y,yaw) y 'dets' = [etiqueta, confianza, rumbo_deg, rango_m].
Proyectando rumbo+rango desde la pose se obtiene la posicion del objeto en frame del mapa.
Agrupando por etiqueta con un clustering simple, salen los MUEBLES REALES y donde estan.
"""
import json, glob, math, collections

CONF_MIN = 0.45
RANGO_MAX = 4.0
UTIL = {"couch", "chair", "refrigerator", "suitcase", "bed", "diningtable", "table", "tv", "book"}

obs = collections.defaultdict(list)
nruns = 0
for f in sorted(glob.glob("dataset/2026*_ours_[AB].json")):
    try:
        d = json.load(open(f))
    except Exception:
        continue
    if d.get("sim_id"):
        continue
    nruns += 1
    for m in d.get("samples") or []:
        x, y, yaw = m.get("x"), m.get("y"), m.get("yaw")
        if x is None or yaw is None:
            continue
        for dd in (m.get("dets") or []):
            try:
                lab, conf, brg, rng = dd[0], float(dd[1]), dd[2], dd[3]
            except (TypeError, IndexError, ValueError):
                continue
            if lab not in UTIL or conf < CONF_MIN or brg is None or rng is None:
                continue
            rng = float(rng)
            if not (0.3 < rng <= RANGO_MAX):
                continue
            a = math.radians(yaw + float(brg))
            obs[lab].append((x + rng * math.cos(a), y + rng * math.sin(a), conf))

print("runs reales usados:", nruns)
salida = {}
for lab, pts in obs.items():
    # clustering codicioso por distancia
    clusters = []
    for (px, py, c) in sorted(pts, key=lambda p: -p[2]):
        for cl in clusters:
            if math.hypot(px - cl["x"], py - cl["y"]) < 0.9:
                n = cl["n"]
                cl["x"] = (cl["x"] * n + px) / (n + 1)
                cl["y"] = (cl["y"] * n + py) / (n + 1)
                cl["n"] = n + 1
                cl["conf"] = max(cl["conf"], c)
                break
        else:
            clusters.append({"x": px, "y": py, "n": 1, "conf": c})
    fuertes = [c for c in clusters if c["n"] >= 4]
    if fuertes:
        salida[lab] = [{"x": round(c["x"], 2), "y": round(c["y"], 2),
                        "n": c["n"], "conf": round(c["conf"], 2)} for c in fuertes]
    print("%-14s observaciones %4d -> %d grupos (%d con >=4 vistas)" % (
        lab, len(pts), len(clusters), len(fuertes)))

json.dump(salida, open("dataset/objetos_vistos.json", "w"), indent=1)
print("\nescrito dataset/objetos_vistos.json")
for lab, cs in salida.items():
    for c in cs:
        print("  %-12s (%+.2f, %+.2f)  vistas=%d conf=%.2f" % (lab, c["x"], c["y"], c["n"], c["conf"]))
