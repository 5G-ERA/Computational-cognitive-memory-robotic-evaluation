"""Geolocaliza las fotos reales del dataset (pose interpolada del JSON de su run)
y elige las que MIRAN a cada zona del plano. Criterio: el rayo del rumbo entra
en la region objetivo entre 0.6 y 5 m, y cuanto mas frontal y cercano, mejor."""
import glob, json, math, os, re
from collections import defaultdict

ZONAS = {
    "Z2_muroN":  (-0.5, 5.2, 2.2),   # cristales + mesa pegada (dice Adrian)
    "Z7_caja":   (3.2, -3.6, 2.0),   # puerta al pasillo + caja del Summit
    "Z1_muroE":  (4.8, 1.5, 2.0),
    "Z4_salaB":  (-5.8, 2.4, 2.0),   # oficina de Renisa
    "Z5_isla":   (1.6, 1.8, 1.6),    # isla de mesas central
}
cand = defaultdict(list)

pose_cache = {}
def poses(run):
    if run not in pose_cache:
        try:
            d = json.load(open("dataset/%s.json" % run))
            pose_cache[run] = [(s["t"], s["x"], s["y"], s["yaw"]) for s in d.get("samples", [])]
        except Exception:
            pose_cache[run] = []
    return pose_cache[run]

n_real = 0
for f in glob.glob("dataset/*_t[0-9][0-9][0-9]s.jpg"):
    if os.path.getsize(f) < 2500:
        continue
    n_real += 1
    m = re.match(r"dataset/(.+)_t(\d+)s\.jpg", f)
    run, t = m.group(1), int(m.group(2))
    ps = poses(run)
    if not ps:
        continue
    # pose mas cercana en tiempo
    p = min(ps, key=lambda s: abs(s[0] - t))
    if abs(p[0] - t) > 3:
        continue
    x, y, yaw = p[1], p[2], math.radians(p[3])
    for z, (zx, zy, zr) in ZONAS.items():
        dx, dy = zx - x, zy - y
        dist = math.hypot(dx, dy)
        if not (0.6 < dist < 5.0):
            continue
        err = abs((math.atan2(dy, dx) - yaw + math.pi) % (2*math.pi) - math.pi)
        if err < math.radians(22):
            # puntua: frontal y cerca, y fotos grandes (mas nitidas)
            sc = err + dist*0.08 - os.path.getsize(f)/9000.0*0.3
            cand[z].append((sc, f, dist, math.degrees(err), os.path.getsize(f)))

print("fotos reales con pose:", n_real)
for z in ZONAS:
    cand[z].sort()
    print("\n%s: %d candidatas" % (z, len(cand[z])))
    for sc, f, dist, err, sz in cand[z][:4]:
        print("  %-55s d=%.1f err=%4.1f  %5dB" % (f, dist, err, sz))
