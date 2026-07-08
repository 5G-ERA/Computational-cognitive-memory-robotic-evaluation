#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_campaign.py — Campana HEADLESS para la validacion empirica del paper DCA (seccion VII):
condicion PAYLOAD (taza de agua, modelo g1_spill_model) con dos brazos alternados:

    base   : G1_META2=0  (sin gobernanza; crucero 0.30 m/s)
    meta2  : G1_META2=2  + config_meta2_g1payload.json (Cautious preferida -> techo 0.28 stick
             = ~0.18 m/s: bajo el umbral de sloshing)

Cada run: reset del gzserver (mundo fresco, robot en A) -> adaptador `goto B` headless con
timeout por SIGINT (g1_goto guarda el dataset como con Ctrl+C). Resultados incrementales en
campaign_results.json; los datasets llevan G1_SIM_ID por brazo para filtrar en el CSV.

USO:  python3 sim_campaign.py [n_por_brazo]      (default 8; ~2-4 min por run)
"""
import glob
import json
import os
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
VENV_PY = "/Users/adrianlendinezibanez/unitree_webrtc_connect/.venv/bin/python"
CONT = "48e7c50b26d6"
RUN_TIMEOUT = 360          # s; el brazo gobernado va a ~mitad de velocidad (RTF sim ~0.5)
ARMS = {
    "base": {"G1_META2": "0", "G1_SIM_ID": "lab_v1_payload_base"},
    "meta2": {"G1_META2": "2", "G1_M2_CFG": "config_meta2_g1payload.json",
              "G1_SIM_ID": "lab_v1_payload_meta2",
              # escalada P9 escalada al RTF~0.5 de la sim (ventanas pensadas para 0.3 m/s reales;
              # el brazo gobernado va a 0.18 sim = ~0.09 m/s de pared): x2 tiempo, progreso /2
              "G1_M2_ABORT_WIN": "150", "G1_M2_ABORT_PROG": "0.25", "G1_M2_HELP_S": "16"},
    # brazo M1 de la tesis (Cap.5): politica conservadora FIJA (solo Cautious, sin adaptacion)
    "conserv": {"G1_META2": "2", "G1_M2_CFG": "config_meta2_g1conservative.json",
                "G1_SIM_ID": "lab_v1_payload_conserv",
                "G1_M2_ABORT_WIN": "150", "G1_M2_ABORT_PROG": "0.25", "G1_M2_HELP_S": "16"},
    # brazo PRIOR ERRONEO (5.3.6.2 de la tesis): META2 activo con la config de puerta NO-payload
    # (Efficient preferida) = analogia mal calibrada para la tarea de payload
    # SIN memoria entre runs = el M2+Wrong-LOCKED de la tesis (Tabla 18)
    "wrong": {"G1_META2": "2", "G1_M2_CFG": "config_meta2_g1door.json",
              "G1_SIM_ID": "lab_v1_payload_wrongprior",
              "G1_M2_ABORT_WIN": "150", "G1_M2_ABORT_PROG": "0.25", "G1_M2_HELP_S": "16"},
    # brazo M2+Wrong->DST: mismo prior erroneo + LAYER 2 (creencia DST persistida entre runs,
    # G1_M2_STATE). Las primeras runs derraman bajo Efficient; la creencia se erosiona y las
    # siguientes arrancan corregidas en Cautious = la recuperacion de la tesis 5.3.6.3.
    "wrongdst": {"G1_META2": "2", "G1_M2_CFG": "config_meta2_g1door.json",
                 "G1_SIM_ID": "lab_v1_payload_wrongdst",
                 "G1_M2_STATE": "meta2_state_wrongdst.json",
                 "G1_M2_ABORT_WIN": "150", "G1_M2_ABORT_PROG": "0.25", "G1_M2_HELP_S": "16"},
    # M1 FIEL a la tesis: techo fijo SIN gobernanza (sin reasoner, sin vetos, sin abortos).
    # El brazo 'conserv' (config solo-Cautious) resulto NO ser M1: un veto de mobility sin
    # analogia alternativa -> HELP sostenido -> aborto P9 (3/8 runs). Se mantiene como dato
    # (rigidez CON gobernanza de un solo candidato) pero el M1 comparable es este.
    "conservfix": {"G1_META2": "0", "G1_VCAP": "0.28",
                   "G1_SIM_ID": "lab_v1_payload_conservfix"},
    # condicion TAPA CERRADA (tesis 5.3.6.4): riesgo x0.25; el M2 'informado por la tarea'
    # usa la config de puerta (Efficient preferida = el covered_delivery del G1).
    "baseclosed": {"G1_META2": "0", "G1_SPILL_LID": "closed",
                   "G1_SIM_ID": "lab_v1_lidclosed_base"},
    "m2closed": {"G1_META2": "2", "G1_M2_CFG": "config_meta2_g1door.json",
                 "G1_SPILL_LID": "closed", "G1_SIM_ID": "lab_v1_lidclosed_meta2",
                 "G1_M2_ABORT_WIN": "150", "G1_M2_ABORT_PROG": "0.25", "G1_M2_HELP_S": "16"},
    # ---- FASE 4: config CALIBRADA AL SIM (la de puerta real dejaba a Efficient sin certificar
    # nunca en el gemelo -> el prior erroneo moria a t~4s y ni wrongdst ni la tapa median nada) ----
    "wrongsim": {"G1_META2": "2", "G1_M2_CFG": "config_meta2_g1door_sim.json",
                 "G1_SIM_ID": "lab_v1_payload_wrongsim",
                 "G1_M2_ABORT_WIN": "150", "G1_M2_ABORT_PROG": "0.25", "G1_M2_HELP_S": "16"},
    "wrongdstsim": {"G1_META2": "2", "G1_M2_CFG": "config_meta2_g1door_sim.json",
                    "G1_SIM_ID": "lab_v1_payload_wrongdstsim",
                    "G1_M2_STATE": "meta2_state_wrongdstsim.json",
                    "G1_M2_ABORT_WIN": "150", "G1_M2_ABORT_PROG": "0.25", "G1_M2_HELP_S": "16"},
    "m2closedsim": {"G1_META2": "2", "G1_M2_CFG": "config_meta2_g1door_sim.json",
                    "G1_SPILL_LID": "closed", "G1_SIM_ID": "lab_v1_lidclosed_meta2sim",
                    "G1_M2_ABORT_WIN": "150", "G1_M2_ABORT_PROG": "0.25", "G1_M2_HELP_S": "16"},
    # tanda PRE-REAL: la configuracion EXACTA recomendada para la campana fisica de Adrian
    # (4 capas activas + estado persistido + rho logueado) — validacion en el bucle antes
    # de tocar el robot. No entra en la tabla del paper.
    # tanda de humo de la RAMA analogy-profiles: las 4 extensiones activas in-loop
    "extsmoke": {"G1_META2": "2", "G1_M2_CFG": "config_meta2_g1payload.json",
                 "G1_SIM_ID": "lab_v1_extsmoke",
                 "G1_M2_STATE": "meta2_state_extsmoke.json",
                 "G1_M2_L3": "1", "G1_M2_L4": "1",
                 "G1_M2_PROFILES": "1", "G1_M2_DOORLIB": "1", "G1_M2_FRAGSPEED": "1",
                 "G1_M2_ABORT_WIN": "150", "G1_M2_ABORT_PROG": "0.25", "G1_M2_HELP_S": "16"},
    "prereal": {"G1_META2": "2", "G1_M2_CFG": "config_meta2_g1payload.json",
                "G1_SIM_ID": "lab_v1_prereal_4capas",
                "G1_M2_STATE": "meta2_state_prereal.json",
                "G1_M2_L3": "1", "G1_M2_L4": "1",
                "G1_M2_ABORT_WIN": "150", "G1_M2_ABORT_PROG": "0.25", "G1_M2_HELP_S": "16"},
    # ---- A/B FORMAL PRE-MERGE (rama analogy-profiles): factorial ligero, estado fresco
    # por brazo (zero-shot en igualdad). Objetivo: watch-item (tasa de abortos HELP cerca
    # de B con robot_r gobernado 0.28-0.30 vs 0.22 fijo de main) + biblioteca on/off ----
    "abctrl": {"G1_META2": "2", "G1_M2_CFG": "config_meta2_g1payload.json",
               "G1_SIM_ID": "lab_v1_ab_ctrl",
               "G1_M2_STATE": "meta2_state_abctrl.json",
               "G1_M2_L3": "1", "G1_M2_L4": "1",
               "G1_M2_ABORT_WIN": "150", "G1_M2_ABORT_PROG": "0.25", "G1_M2_HELP_S": "16"},
    "abprof": {"G1_META2": "2", "G1_M2_CFG": "config_meta2_g1payload.json",
               "G1_SIM_ID": "lab_v1_ab_prof",
               "G1_M2_STATE": "meta2_state_abprof.json",
               "G1_M2_L3": "1", "G1_M2_L4": "1", "G1_M2_PROFILES": "1",
               "G1_M2_ABORT_WIN": "150", "G1_M2_ABORT_PROG": "0.25", "G1_M2_HELP_S": "16"},
    "abdoor": {"G1_META2": "2", "G1_M2_CFG": "config_meta2_g1payload.json",
               "G1_SIM_ID": "lab_v1_ab_door",
               "G1_M2_STATE": "meta2_state_abdoor.json",
               "G1_M2_L3": "1", "G1_M2_L4": "1", "G1_M2_DOORLIB": "1",
               "G1_M2_ABORT_WIN": "150", "G1_M2_ABORT_PROG": "0.25", "G1_M2_HELP_S": "16"},
    "abfull": {"G1_META2": "2", "G1_M2_CFG": "config_meta2_g1payload.json",
               "G1_SIM_ID": "lab_v1_ab_full",
               "G1_M2_STATE": "meta2_state_abfull.json",
               "G1_M2_L3": "1", "G1_M2_L4": "1",
               "G1_M2_PROFILES": "1", "G1_M2_DOORLIB": "1", "G1_M2_FRAGSPEED": "1",
               "G1_M2_ABORT_WIN": "150", "G1_M2_ABORT_PROG": "0.25", "G1_M2_HELP_S": "16"},
}
# brazos activos de esta invocacion: por env CAMPAIGN_ARMS="base,meta2" (default los 4)
ACTIVE_ARMS = [a for a in os.environ.get("CAMPAIGN_ARMS", ",".join(ARMS)).split(",") if a in ARMS]


def sh(cmd, timeout=60):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)


def reset_sim():
    """Mata gz/rsp por PID (pkill -f se auto-empareja y cuelga) y relanza lab.launch headless."""
    r = sh(f"docker exec {CONT} ps -eo pid,cmd")
    for line in r.stdout.splitlines():
        if any(k in line for k in ("gzserver", "gzclient", "robot_state_publisher",
                                   "lab.launch", "spawn_entity")):
            pid = line.strip().split()[0]
            if pid.isdigit():
                sh(f"docker exec {CONT} kill -9 {pid}")
    time.sleep(2)
    sh(f"docker exec -d {CONT} bash -c 'source /opt/ros/humble/setup.bash && "
       f"source /home/ubuntu/g1_ws/install/setup.bash && "
       f"exec ros2 launch g1_sim lab.launch.py gui:=false > /tmp/lab_launch.log 2>&1'")
    for _ in range(45):
        time.sleep(2)
        r = sh(f"docker exec {CONT} grep -c 'Successfully spawned' /tmp/lab_launch.log")
        if r.stdout.strip() and r.stdout.strip()[0] != "0":
            time.sleep(2)          # dar aire a planar_move/scan
            return True
    return False


def newest_run():
    fs = [f for f in glob.glob(os.path.join(HERE, "dataset", "2026*_ours_B.json"))
          if "_col" not in os.path.basename(f) and "_end" not in os.path.basename(f)]
    return max(fs, key=os.path.getmtime) if fs else None


def one_run(arm):
    env = dict(os.environ)
    env.update(ARMS[arm])
    before = newest_run()
    log = open(os.path.join(HERE, f"campaign_{arm}.log"), "a")
    p = subprocess.Popen([VENV_PY, "g1_sim_adapter.py", "goto", "B"], cwd=HERE, env=env,
                         stdout=log, stderr=subprocess.STDOUT)
    t0 = time.time()
    while p.poll() is None:
        if time.time() - t0 > RUN_TIMEOUT:
            p.send_signal(signal.SIGINT)               # = Ctrl+C: g1_goto guarda el dataset
            try:
                p.wait(30)
            except subprocess.TimeoutExpired:
                p.kill()
            break
        time.sleep(2)
    log.close()
    time.sleep(2)
    f = newest_run()
    if not f or f == before:
        print("  RUN PERDIDA (sin dataset nuevo)", flush=True)
        return None
    d = json.load(open(f))
    sm = d.get("summary") or {}
    row = {"file": os.path.basename(f), "arm": arm, "result": d.get("result"),
           "time_s": sm.get("time_s"), "collisions": sm.get("collisions"),
           "spills": sm.get("spills_sim"), "spill_expected": sm.get("spill_expected"),
           "spill_risk_pct": sm.get("spill_risk_pct"), "eta_max": sm.get("spill_eta_max"),
           "meta2_mode": d.get("meta2_mode")}
    print(f"  RUN {row}", flush=True)
    return row


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    out = os.path.join(HERE, "campaign_results.json")
    rows = json.load(open(out)) if os.path.exists(out) else []
    for i in range(n):
        for arm in ACTIVE_ARMS:
            print(f"== run {i + 1}/{n} brazo={arm} ({time.strftime('%H:%M:%S')})", flush=True)
            if not reset_sim() and not reset_sim():
                print("SIM NO ARRANCA: abortando campana", flush=True)
                json.dump(rows, open(out, "w"), indent=1)
                return
            r = one_run(arm)
            if r:
                rows.append(r)
            json.dump(rows, open(out, "w"), indent=1)
    print(f"CAMPANA COMPLETA: {len(rows)} runs -> {out}", flush=True)


if __name__ == "__main__":
    main()
