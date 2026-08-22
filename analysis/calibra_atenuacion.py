"""Calibra la atenuacion para reproducir el reparto REAL de detecciones por muestra."""
import sys, os, json, glob, math, collections
sys.path.insert(0, "sim")

# objetivo real
real = collections.Counter()
for f in sorted(glob.glob("dataset/2026*_ours_[AB].json")):
    try: d = json.load(open(f))
    except Exception: continue
    if d.get("sim_id") is not None: continue
    ss = d.get("samples") or []
    if len(ss) < 20: continue
    for m in ss:
        real[min(3, len([z for z in (m.get("dets") or []) if z[0] != "door"]))] += 1
t = sum(real.values())
obj = {k: 100*real[k]/t for k in range(4)}
print("REAL: 0:%.0f%% 1:%.0f%% 2:%.0f%% 3+:%.0f%%" % tuple(obj[k] for k in range(4)))

# poses reales para muestrear en las mismas condiciones
poses = []
for f in sorted(glob.glob("dataset/2026*_ours_[AB].json"))[:40]:
    try: d = json.load(open(f))
    except Exception: continue
    if d.get("sim_id") is not None: continue
    for m in (d.get("samples") or []):
        if m.get("x") is not None:
            poses.append((m["x"], m["y"], m["yaw"]))
print("poses de muestreo:", len(poses))

for aten in (0.30, 0.22, 0.18, 0.14, 0.10, 0.07):
    os.environ["G1_EMU_ATEN"] = str(aten)
    for mod in list(sys.modules):
        if "emulador" in mod: del sys.modules[mod]
    from emulador_deteccion import carga_por_defecto
    e = carga_por_defecto(".")
    c = collections.Counter()
    for (x, y, yaw) in poses:
        c[min(3, len(e.detecta(x, y, yaw, 110)))] += 1
    n = sum(c.values())
    sim = {k: 100*c[k]/n for k in range(4)}
    err = sum(abs(sim[k]-obj[k]) for k in range(4))
    print("aten %.2f -> 0:%.0f%% 1:%.0f%% 2:%.0f%% 3+:%.0f%%  | error total %.0f pp" % (
        aten, sim[0], sim[1], sim[2], sim[3], err))
