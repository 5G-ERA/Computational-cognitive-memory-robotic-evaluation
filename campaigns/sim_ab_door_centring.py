#!/usr/bin/env python3
"""Campana A/B BAJO RUIDO CALIBRADO (G1_SIM_NOISE=1): maquina META ON vs OFF, B->A.
Orden ABBA intercalado + 2 runs ON+HUMANO al final. Progreso por stdout."""
import subprocess, time, os, sys, json, glob, math, socket, threading

HERE = "/Users/adrianlendinezibanez/Claude/Projects/G1 ROBOT"
PY = "/Users/adrianlendinezibanez/unitree_webrtc_connect/.venv/bin/python"
SCRATCH = os.path.dirname(os.path.abspath(__file__))

COMMON = {"G1_META2": "2", "G1_M2_CFG": "config_meta2_g1payload.json",
          "G1_SIM_ID": "lab_v1_ctr", "G1_SIM_NOISE": "1", "G1_M2_L3": "1", "G1_M2_L4": "1",
          "G1_M2_PROFILES": "1", "G1_M2_DOORLIB": "1", "G1_M2_FRAGSPEED": "1",
          "G1_M2_ABORT_WIN": "150", "G1_M2_ABORT_PROG": "0.25", "G1_M2_HELP_S": "16"}
# A/B del CENTRADO EN EL VANO: puertas actuales vs apretadas. Todo lo demas identico,
# con ruido calibrado + camara sintetica + DOOR-VIS = la configuracion realista.
ARMS = {
    "base":  {"G1_DOOR_CTR_TOL": "0.14", "G1_DOOR_CTR_S": "0.35",
              "G1_SIM_PERC": "1", "G1_PERC": "127.0.0.1:8010", "G1_DOOR_VIS": "1",
              "G1_M2_STATE": "meta2_state_ctr_base.json"},
    "tight": {"G1_DOOR_CTR_TOL": "0.07", "G1_DOOR_CTR_S": "0.18",
              "G1_SIM_PERC": "1", "G1_PERC": "127.0.0.1:8010", "G1_DOOR_VIS": "1",
              "G1_M2_STATE": "meta2_state_ctr_tight.json"},
}
ORDER = ["base", "tight", "tight", "base", "base", "tight", "tight", "base",
         "base", "tight", "tight", "base"]


def sh(cmd):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=60)


def teleport():
    sh("docker exec g1sim bash -c 'gz model -m g1 -x -4.75 -y 2.60 -z 0.05 -Y -0.9'")


def newest_run():
    fs = [f for f in glob.glob(os.path.join(HERE, "dataset", "2026*_ours_A.json"))
          if "_col" not in f and "_end" not in f]
    return max(fs, key=os.path.getmtime) if fs else None


def humano(logpath, stop):
    """Humano simulado: responde a cada peticion ASSIST con hmove:40 (max 4)."""
    dado = 0
    seen = 0
    while not stop.is_set() and dado < 4:
        try:
            txt = open(logpath).read()
        except Exception:
            txt = ""
        n = txt.count("ASISTENCIA")
        if n > seen:
            time.sleep(2)
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.sendto(b"hmove:40", ("127.0.0.1", 7777))
            dado += 1
            seen = txt.count("ASISTENCIA") + 50   # no reaccionar a los prints de ESTA espera
            print("    [humano] asistencia #%d dada" % dado, flush=True)
            time.sleep(20)                        # dejar reanudar antes de volver a mirar
            seen = open(logpath).read().count("ASISTENCIA")
        time.sleep(3)


def _lat_umbral(ss):
    """Desvio lateral mediano respecto al eje del vano en el ultimo tramo (0.7-0.45 m)."""
    DX, DY = -3.90, 1.25
    ax = math.radians(135.0); ux, uy = math.cos(ax), math.sin(ax); px, py = -uy, ux
    v = []
    for s in ss:
        dd = math.hypot(s["x"] - DX, s["y"] - DY)
        if 0.45 <= dd < 0.7:
            v.append((s["x"] - DX) * px + (s["y"] - DY) * py)
    if not v:
        return None
    v.sort()
    return round(v[len(v) // 2], 3)


def analyze(path, arm):
    d = json.load(open(path))
    ss = d["samples"]; ev = d["events"]
    goal = d.get("goal") or {}
    gx, gy = goal.get("x", 0), goal.get("y", 0)
    dfin = math.hypot(ss[-1]["x"] - gx, ss[-1]["y"] - gy)
    return {"arm": arm, "file": os.path.basename(path),
            "dur_s": round(ss[-1]["t"], 1), "d_final": round(dfin, 2),
            "reached": dfin < 0.45,
            "n_col": sum(1 for e in ev if e.get("kind") == "collision"),
            "retreats": sum(1 for e in ev if e.get("kind") == "retreat_start"),
            "assists": sum(1 for e in ev if e.get("kind") == "human_assist"),
            "lat_umbral": _lat_umbral(ss)}


results = []
for i, arm in enumerate(ORDER):
    teleport(); time.sleep(3)
    env = dict(os.environ); env.update(COMMON); env.update(ARMS[arm])
    logpath = os.path.join(SCRATCH, "ctr_%02d_%s.log" % (i, arm))
    before = newest_run()
    print("RUN %02d/%d arm=%s ..." % (i + 1, len(ORDER), arm), flush=True)
    with open(logpath, "w") as lf:
        p = subprocess.Popen([PY, "g1_sim_adapter.py", "goto", "A"], cwd=HERE,
                             env=env, stdout=lf, stderr=subprocess.STDOUT)
        stop = threading.Event()
        try:
            p.wait(timeout=900)
        except subprocess.TimeoutExpired:
            p.kill()
            print("    TIMEOUT 900s -> run matado", flush=True)
        stop.set()
    time.sleep(2)
    after = newest_run()
    if after and after != before:
        row = analyze(after, arm)
        results.append(row)
        print("  -> %s" % json.dumps(row, ensure_ascii=False), flush=True)
    else:
        print("  -> SIN DATASET (run no registrado)", flush=True)

json.dump(results, open(os.path.join(SCRATCH, "ctr_results.json"), "w"), indent=1)
print("CAMPANA FIN: %d runs registrados" % len(results), flush=True)
