"""¿Como FILTRA la app la nube del lidar? Estadisticas de los snapshots reales."""
import json, glob, math, statistics
import collections

pts_por_snap, rangos, sect_por_snap = [], [], []
n = 0
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
        pts = s.get("pts") or []
        n += 1
        pts_por_snap.append(len(pts))
        secs = set()
        for p in pts:
            dx, dy = p[0] - s["x"], p[1] - s["y"]
            r = math.hypot(dx, dy)
            if r < 0.05:
                continue
            rangos.append(r)
            secs.add(int((math.degrees(math.atan2(dy, dx)) % 360) // 2))
        sect_por_snap.append(len(secs))

def q(v, p):
    return sorted(v)[int(p * len(v))]

print("snapshots reales:", n)
print("PUNTOS por snapshot:   med %d   p10 %d   p90 %d   max %d" % (
    statistics.median(pts_por_snap), q(pts_por_snap, .1), q(pts_por_snap, .9), max(pts_por_snap)))
print("SECTORES (de 180) por snapshot: med %d  p10 %d  p90 %d" % (
    statistics.median(sect_por_snap), q(sect_por_snap, .1), q(sect_por_snap, .9)))
print("RANGOS: med %.2f  p10 %.2f  p90 %.2f  max %.2f" % (
    statistics.median(rangos), q(rangos, .1), q(rangos, .9), max(rangos)))
h = collections.Counter(min(int(r), 7) for r in rangos)
tot = sum(h.values())
print("histograma de rango:", {("%d-%dm" % (k, k+1)): "%.0f%%" % (100*v/tot) for k, v in sorted(h.items())})
