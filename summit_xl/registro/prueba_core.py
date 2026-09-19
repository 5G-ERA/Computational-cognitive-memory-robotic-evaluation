#!/usr/bin/env python3
"""prueba_core.py — comprueba en frio, sin ROS y sin robot, el registro del Summit.

  python3 summit_xl/registro/prueba_core.py        # usa src/ y tools/ de este mismo repo
"""
import csv, json, math, os, re, shutil, subprocess, sys, tempfile
AQUI = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else os.path.join(AQUI, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "src")); sys.path.insert(0, AQUI)
import summit_run_core as core
from g1_metrics import SEIMetrics, SensingMonitor

fallos = []


def comprueba(nombre, ok, detalle=""):
    print("  %-4s %s%s" % ("ok" if ok else "FALLA", nombre, ("  · " + detalle) if detalle else ""))
    if not ok:
        fallos.append(nombre)


# 1 · paridad de columnas contra el FUENTE del G1 -------------------------------------------
print("1 · columnas de runs_stats.csv contra g1_goto.py")
src = open(os.path.join(REPO, "src", "g1_goto.py")).read()
ini = src.index("        row = {\n            \"run\":")
fin = src.index("\n        }\n", ini)
cols_g1 = re.findall(r'"([a-z0-9_]+)"\s*:', src[ini:fin])
comprueba("mismas columnas, mismo orden", cols_g1 == core.COLS_G1,
          "G1 %d · aqui %d · faltan %s · sobran %s" % (len(cols_g1), len(core.COLS_G1),
          [c for c in cols_g1 if c not in core.COLS_G1], [c for c in core.COLS_G1 if c not in cols_g1]))
import g1_metrics as _gm
comprueba("g1_metrics se importa de src/ del repo, no de una copia",
          os.path.abspath(_gm.__file__) == os.path.join(REPO, "src", "g1_metrics.py"), _gm.__file__)

# 2 · el detector de contacto ---------------------------------------------------------------
print("2 · detector de contacto")


def corre(perfil, dt=0.1):
    """perfil: lista de (segundos, v_cmd, v_rueda, esfuerzo). Devuelve los eventos."""
    det = core.ContactDetector(); t = 0.0; out = []
    for dur, vc, vr, ef in perfil:
        for _ in range(int(round(dur / dt))):
            t += dt
            r = det.update(t, vc, vr, ef)
            if r:
                out.append((round(t, 1), r[0], r[1]))
    return out


ev = corre([(12.0, 0.05, 0.049, 40.0)])
comprueba("cruce lento a 0.05 m/s durante 12 s: CERO colisiones", ev == [], str(ev))
ev = corre([(5.0, 0.20, 0.19, 40.0), (3.0, 0.20, 0.0, 95.0)])
comprueba("rodando libre y luego clavado empujando: una colision", [e[1] for e in ev] == ["collision"], str(ev))
ev = corre([(5.0, 0.20, 0.19, 40.0), (3.0, 0.20, 0.0, 2.0)])
comprueba("ruedas paradas SIN esfuerzo: drive_stall, no colision", [e[1] for e in ev] == ["drive_stall"], str(ev))
ev = corre([(3.0, 0.0, 0.0, 5.0), (1.5, 0.20, 0.10, 45.0), (5.0, 0.20, 0.19, 40.0)])
comprueba("arranque con rampa de aceleracion: CERO colisiones", ev == [], str(ev))
ev = corre([(5.0, 0.20, 0.19, 40.0), (16.0, 0.20, 0.0, 95.0)])
comprueba("clavado 16 s empujando: UN contacto = UNA colision", len(ev) == 1, str(ev))
ev = corre([(5.0, 0.20, 0.19, 40.0), (3.0, 0.20, 0.0, 95.0), (6.0, 0.20, 0.19, 40.0), (3.0, 0.20, 0.0, 95.0)])
comprueba("choca, se libera 6 s y vuelve a chocar: DOS colisiones", len(ev) == 2, str(ev))
det = core.ContactDetector()
comprueba("sin ruedas no inventa nada", all(det.update(0.1 * i, 0.2, None, None) is None for i in range(60)))

# 3 · una run sintetica entera: A -> puerta, 30 s -------------------------------------------
print("3 · run sintetica, JSON y fila de stats")
tmp = tempfile.mkdtemp(prefix="summit_run_")
ds = os.path.join(tmp, "dataset")
rd = core.RunRecorder(ds, "ours", "vano_antes", (3.0, 0.0), env="sim", arm="C0-PRUEBA")
sei = SEIMetrics(); sens = SensingMonitor(); trail = []; x = 0.0; minc0 = 9.0
for i in range(300):
    t = i * 0.1; v = 0.2 if i < 140 else 0.0
    x += v * 0.1; trail.append((x, 0.0)); d = abs(3.0 - x)
    c0 = max(0.4, 2.5 - x); minc0 = min(minc0, c0)
    m = sei.update(t, d, c0); s2 = sens.update(t, {(i % 5, 0), (1, 1), (2, 2)}, c0, 0.9)
    rd.sample(t, x, 0.0, 0.0, d, v, c0, 3, cmd=(v, 0.0), phase="DWA-TEB" if v else "STOP",
              extra={**m, **s2, "spd_cmd": v})
rd.event("collision", 5.0, 1.0, 0.0, {"src": "wheel"})
rd.event("collision", 9.0, 1.8, 0.0, {"src": "human"})
for _ in range(3):
    rd.event("collision_ahead", 11.0, 2.2, 0.0)
rd.event("amcl_jump", 12.0, 2.4, 0.0, {"dist": 0.088})
plen = core.path_len(trail); recta = math.hypot(trail[-1][0] - trail[0][0], 0.0)
fjson, fcsv, fila = rd.finish("aborted", {"time_s": 30.0, "path_m": round(plen, 2), "straight_m": round(recta, 2),
                                           "efficiency": round(recta / plen, 2), "c0min": round(minc0, 2)})
d = json.load(open(fjson))
comprueba("schema g1_goto_run/v1 y robot summit_xl", d["schema"] == "g1_goto_run/v1" and d["robot"] == "summit_xl")
comprueba("300 muestras con las claves del G1",
          len(d["samples"]) == 300 and all(k in d["samples"][0] for k in ("t", "x", "y", "yaw", "d", "spd", "c0", "nobs", "phase", "cmd")))
comprueba("las definiciones viajan en el JSON", "collision" in d["defs"])
comprueba("collisions=2 y collision_ahead=3 van en columnas SEPARADAS",
          fila["collisions"] == 2 and fila["collision_ahead"] == 3 and fila["collisions_human"] == 1, str(fila["collisions"]))
comprueba("path 2.78 m (139 pasos de 2 cm), eficiencia 1.0", fila["path_m"] == 2.78 and fila["efficiency"] == 1.0,
          "%s %s" % (fila["path_m"], fila["efficiency"]))
comprueba("un atasco contado (16 s parado lejos del objetivo)", fila["stuck_episodes"] == 1, str(fila["stuck_episodes"]))
comprueba("c0_body_min = c0_min - 0.361", fila["c0_body_min"] == round(fila["c0_min"] - 0.361, 3))
comprueba("bateria en blanco, no inventada", fila["bat_start"] == "" and fila["bat_used"] == "")
filas = list(csv.DictReader(open(fcsv)))
comprueba("el CSV se relee y sus primeras columnas son las del G1",
          len(filas) == 1 and list(filas[0].keys())[:len(core.COLS_G1)] == core.COLS_G1)

# 4 · el summarize_runs.py DEL G1 se traga el JSON del Summit -------------------------------
print("4 · el resumidor del propio G1 leyendo una run del Summit")
r = subprocess.run([sys.executable, os.path.join(REPO, "tools", "summarize_runs.py")], cwd=tmp,
                   capture_output=True, text=True, timeout=60)
ok = r.returncode == 0 and os.path.exists(os.path.join(tmp, "runs_summary.csv"))
fs = list(csv.DictReader(open(os.path.join(tmp, "runs_summary.csv")))) if ok else []
comprueba("summarize_runs.py del G1 corre sin tocar y saca la run", ok and len(fs) == 1, r.stderr[-200:])
if fs:
    comprueba("y lee bien tiempo, colisiones y clearance",
              fs[0]["time_s"] == "30.0" and fs[0]["collisions"] == "2" and fs[0]["mean_clearance"] != "",
              "t=%s col=%s cl=%s" % (fs[0]["time_s"], fs[0]["collisions"], fs[0]["mean_clearance"]))
shutil.rmtree(tmp)
print()
print("TODO OK" if not fallos else "FALLAN %d: %s" % (len(fallos), fallos))
sys.exit(1 if fallos else 0)
