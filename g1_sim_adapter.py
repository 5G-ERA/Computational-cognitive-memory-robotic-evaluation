#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
g1_sim_adapter.py — Corre g1_goto.py COMPLETO (META2, engagement, guards, dataset) contra la
SIMULACION de Gazebo del contenedor, sin tocar una linea del stack real.

Como funciona: g1_goto habla con el robot real a traves de un objeto CDP que evalua snippets
JS en el WebView de la app (pose, nube 'location', driver window.__cmd). Este adaptador
suplanta ese objeto con un SimCDP que resuelve LOS MISMOS snippets contra ROS2 via rosbridge:

    window.__cmd={lx,ly,rx,ry}  ->  /cmd_vel (Twist)  [misma convencion fisica que el robot:
                                    deadzone 0.3; lx>0=DERECHA -> vy<0; rx>0 -> wz<0 (~-1.55*rx);
                                    ly 0.4 ~ 0.30 m/s -> vx=0.75*ly]
    window.__pose               <-  /odom (nav_msgs/Odometry)  [x,y,z,qx,qy,qz,qw]
    window.__relocbuf           <-  /scan (LaserScan 360)  proyectado a nube plana [x,y,0,...]
                                    en frame del MAPA (en sim odom==map, sin saltos de reloc)
    telemetria/camara           ->  fake benigno (bat=100; sin camara -> G1_NOVIS=1 por defecto)

Ademas parchea el ESCENARIO: refmap generado de room.world (4 paredes + pilar), waypoints y
nav_map PROPIOS de la sim (no toca los del lab), engagement de puerta OFF (room.world no tiene
puerta) y etiquetado G1_ENV=sim automatico.

PRE (en el contenedor, 2 terminales):
    ros2 launch g1_sim sim.launch.py gui:=false
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=8765

USO (desde el Mac, carpeta G1 ROBOT):
    python g1_sim_adapter.py gotoviz B          # igual que g1_goto, pero contra la sim
    python g1_sim_adapter.py waypoint A         # grabar waypoints sim, sweep, etc.
    G1_SIM_URL=ws://otrohost:8765 python g1_sim_adapter.py goto B
"""
import json
import math
import os
import re
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.join(HERE, "sim")
SIM_URL = os.environ.get("G1_SIM_URL", "ws://localhost:8765")

# --- mapeo stick -> velocidad (MISMA fisica calibrada en el robot real) ---
# MAPEO DE STICKS — CALIBRADO CONTRA EL ROBOT REAL (2 puntos medidos, run 20260703_093703):
#   ly=0.40 -> 0.30 m/s   y   ly=0.28 -> 0.18 m/s (mediana de 75 muestras ENG-F/DWA lentas)
# El modelo correcto es RESTAR la deadzone, no cortar: v = K * (|s| - DZ), s>DZ.
# El mapeo antiguo (corte duro en 0.30) ANULABA el 0.28 del engagement de puerta: el robot
# se congelaba en ENG-F, el detector de atasco marcaba "colisiones" fantasma que sembraban
# celdas de obstaculo en el pasillo (la "columna que no existe") y la run acababa abortada.
# La topologia del lab.world era correcta: cero cajas a <0.45 m de las 11 "colisiones".
DEADZONE = float(os.environ.get("G1_SIM_DZ", "0.10"))    # por debajo, cero de verdad
K_FWD = float(os.environ.get("G1_SIM_KFWD", "1.0"))      # 1.0*(0.40-0.10)=0.30 ; 1.0*(0.28-0.10)=0.18
K_LAT = float(os.environ.get("G1_SIM_KLAT", "1.0"))      # misma escala; lx>0 = DERECHA -> vy NEGATIVA
K_YAW = float(os.environ.get("G1_SIM_KYAW", "2.0"))      # 2.0*(0.45-0.10)=0.70 rad/s = 40 deg/s ; signo -

ROOM_WALLS = [           # room.world: (cx, cy, sx, sy) de cada caja (proyeccion 2D)
    (0.0, 3.0, 8.0, 0.1), (0.0, -3.0, 8.0, 0.1),
    (4.0, 0.0, 0.1, 6.0), (-4.0, 0.0, 0.1, 6.0),
    (1.0, -0.5, 0.6, 0.6),                        # pilar
]
SIM_WAYPOINTS = {
    "A": {"x": 2.5, "y": -1.8, "yaw": 180.0, "src": "sim", "pcd": "SIM_room", "t": "sim"},
    "B": {"x": -2.5, "y": 1.8, "yaw": 0.0, "src": "sim", "pcd": "SIM_room", "t": "sim"},
    "C": {"x": -2.5, "y": -1.8, "yaw": 0.0, "src": "sim", "pcd": "SIM_room", "t": "sim"},
}

_CMD_RE = re.compile(r"__cmd=\{lx:([-\d.e]+),ly:([-\d.e]+),rx:([-\d.e]+),ry:([-\d.e]+)\}")


class RosBridge:
    """Cliente rosbridge minimo (JSON sobre websocket) con dead-man de /cmd_vel."""

    def __init__(self, url):
        import websocket
        self._ws_mod = websocket
        self.url = url
        self.ws = None
        self.lock = threading.Lock()
        self.odom = None          # ultimo nav_msgs/Odometry (msg dict)
        self.scan = None          # ultimo sensor_msgs/LaserScan (msg dict)
        self.scan_t = 0.0
        self.last_cmd_t = 0.0
        self.connected = False
        self._connect()
        threading.Thread(target=self._rx, daemon=True).start()
        threading.Thread(target=self._deadman, daemon=True).start()

    def _connect(self):
        self.ws = self._ws_mod.create_connection(self.url, timeout=5)
        self.connected = True
        for topic, mtype, throttle in (("/odom", "nav_msgs/Odometry", 100),
                                       ("/scan", "sensor_msgs/LaserScan", 250)):
            self.ws.send(json.dumps({"op": "subscribe", "topic": topic, "type": mtype,
                                     "throttle_rate": throttle, "queue_length": 1}))
        self.ws.send(json.dumps({"op": "advertise", "topic": "/cmd_vel",
                                 "type": "geometry_msgs/Twist"}))

    def _rx(self):
        while True:
            try:
                m = json.loads(self.ws.recv())
            except Exception:
                self.connected = False
                time.sleep(1.0)
                try:
                    self._connect()
                except Exception:
                    continue
                continue
            if m.get("op") != "publish":
                continue
            with self.lock:
                if m["topic"] == "/odom":
                    self.odom = m["msg"]
                elif m["topic"] == "/scan":
                    self.scan = m["msg"]
                    self.scan_t = time.time()

    def _deadman(self):
        """Como el driver real (600ms): si el control muere, la sim se para sola."""
        while True:
            time.sleep(0.2)
            if self.last_cmd_t and time.time() - self.last_cmd_t > 0.8:
                self.publish_cmd(0.0, 0.0, 0.0)
                self.last_cmd_t = 0.0

    def publish_cmd(self, vx, vy, wz):
        try:
            self.ws.send(json.dumps({"op": "publish", "topic": "/cmd_vel",
                                     "msg": {"linear": {"x": vx, "y": vy, "z": 0.0},
                                             "angular": {"x": 0.0, "y": 0.0, "z": wz}}}))
        except Exception:
            pass


class SimCDP:
    """Suplanta al CDP del WebView: resuelve los snippets JS de g1_goto/g1_nav_v2 contra ROS."""

    def __init__(self, bridge):
        self.b = bridge
        self._cloud = []          # nube plana [x,y,z,...] en frame mapa (z=0: banda de torso)
        self._cloud_t = 0.0

    # --- helpers ---
    def _pose7(self):
        od = self.b.odom
        if not od:
            return None
        p = od["pose"]["pose"]["position"]; q = od["pose"]["pose"]["orientation"]
        return [p["x"], p["y"], p["z"], q["x"], q["y"], q["z"], q["w"]]

    def _refresh_cloud(self):
        sc = self.b.scan; po = self._pose7()
        if not sc or not po or self.b.scan_t <= self._cloud_t:
            return
        yaw = math.atan2(2 * (po[6] * po[5] + po[3] * po[4]),
                         1 - 2 * (po[4] * po[4] + po[5] * po[5]))
        a = sc["angle_min"]; inc = sc["angle_increment"]; rmax = sc.get("range_max", 12.0)
        flat = []
        for i, r in enumerate(sc["ranges"]):
            if r is None or not isinstance(r, (int, float)):
                continue
            if not math.isfinite(r) or r <= sc.get("range_min", 0.15) or r >= rmax * 0.98:
                continue
            th = yaw + a + i * inc
            flat.extend((round(po[0] + r * math.cos(th), 3),
                         round(po[1] + r * math.sin(th), 3), 0.0))
        self._cloud = flat
        self._cloud_t = self.b.scan_t

    # --- interfaz CDP ---
    def call(self, method, params=None, timeout=0, debug=False):
        return {}

    def eval(self, expr):
        e = expr if isinstance(expr, str) else str(expr)
        m = _CMD_RE.search(e.replace(" ", ""))
        if m:                                             # driver: window.__cmd={lx,ly,rx,ry}
            lx, ly, rx, _ = (float(v) for v in m.groups())
            # deadzone RESTADA (como el robot real), no corte duro: v = K*(|s|-DZ)
            dz = lambda v: 0.0 if abs(v) <= DEADZONE else math.copysign(abs(v) - DEADZONE, v)
            lx, ly, rx = dz(lx), dz(ly), dz(rx)
            self.b.publish_cmd(K_FWD * ly, -K_LAT * lx, -K_YAW * rx)
            self.b.last_cmd_t = time.time()
            return "ok"
        if "__relocbuf||[]).length" in e:
            self._refresh_cloud()
            return len(self._cloud)
        if "JSON.stringify(window.__relocbuf" in e:
            self._refresh_cloud()
            return json.dumps(self._cloud)
        if "__relocbuf_t" in e:
            return int(self._cloud_t * 1000)
        if "JSON.stringify({pose:" in e:                  # read_pose()
            po = self._pose7()
            return json.dumps({"pose": po, "reloc": None, "map": po, "pcd": "SIM",
                               "pt": int(time.time() * 1000), "rt": 0})
        if "JSON.stringify({err:" in e:                   # read_telemetry()
            return json.dumps({"err": None, "h": {"bat": 100, "vol": 54.0, "cpuT": 40,
                                                  "motTmax": 35, "merr": 0},
                               "imu": None, "cov": None})
        if "toDataURL" in e or "__camc" in e:             # camara: no hay en la sim
            return ""
        if "JSON.stringify" in e:
            return "{}"
        if ".length" in e:
            return 0
        return "installed"                                # hooks/instaladores varios


def _sim_ref_points(step=0.05):
    """Perimetro 2D de las cajas de room.world -> puntos de pared para el refmap del plan."""
    pts = []
    for cx, cy, sx, sy in ROOM_WALLS:
        hx, hy = sx / 2.0, sy / 2.0
        x = -hx
        while x <= hx + 1e-9:
            pts.append((round(cx + x, 3), round(cy - hy, 3)))
            pts.append((round(cx + x, 3), round(cy + hy, 3)))
            x += step
        y = -hy
        while y <= hy + 1e-9:
            pts.append((round(cx - hx, 3), round(cy + y, 3)))
            pts.append((round(cx + hx, 3), round(cy + y, 3)))
            y += step
    return pts


def main():
    # --- escenario: 'lab' (mundo generado del MAPA REAL; mismos waypoints/puerta que el robot)
    #     o 'room' (sala sintetica 8x6 con pilar). G1_SIM_SCENARIO=room para la sintetica.
    scenario = os.environ.get("G1_SIM_SCENARIO", "lab").strip().lower()
    os.environ.setdefault("G1_ENV", "sim")
    os.environ.setdefault("G1_SIM_ID", "lab_v1" if scenario == "lab" else "room_v1")
    os.environ.setdefault("G1_NOVIS", "1")            # sin camara/percepcion en la sim (por ahora)
    if scenario != "lab":
        os.environ.setdefault("G1_DOOR_ENGAGE", "0")  # room.world no tiene puerta
    os.environ.setdefault("G1_RELOCGUARD", "0")       # en sim la pose es perfecta: guardia innecesaria
    os.environ.setdefault("G1_NOGATE", "1")           # gate de "reloc dudosa" (arrancar lejos de un
                                                      # waypoint): sin sentido en sim, el spawn es (0,0)
    # backend GRAFICO: en el venv del Mac matplotlib cae por defecto a 'Agg' (sin ventana, sin
    # error) y gotoviz "abria" una ventana invisible. Forzamos un backend con GUI si hay pantalla.
    if sys.platform == "darwin" and not os.environ.get("MPLBACKEND"):
        # TkAgg PRIMERO: el backend MacOSX de matplotlib tiene un bug con plt.pause() (ventana
        # invisible, confirmado 2026-07-03); Tk pinta bien. Requiere: brew install python-tk@3.11
        try:
            import matplotlib
            matplotlib.use("TkAgg")
        except Exception:
            try:
                import matplotlib
                matplotlib.use("MacOSX")
            except Exception:
                pass
    try:
        import matplotlib
        _bk = matplotlib.get_backend()
        print(f"  backend grafico: {_bk}"
              + ("  (SIN VENTANA: backend no interactivo!)" if _bk.lower() in ("agg", "pdf", "svg", "ps", "template") else ""))
    except Exception as e:
        print("  matplotlib no importable:", repr(e))

    os.makedirs(SIM_DIR, exist_ok=True)
    if scenario != "lab":
        wp_file = os.path.join(SIM_DIR, "waypoints_sim.json")
        if not os.path.exists(wp_file):
            json.dump(SIM_WAYPOINTS, open(wp_file, "w"), indent=2)
            print(f"  waypoints de sim creados: {wp_file}")
        ref_file = os.path.join(SIM_DIR, "ref_map_room.json")
        if not os.path.exists(ref_file):
            pts = _sim_ref_points()
            json.dump({"frame": "sim room.world", "src": "g1_sim_adapter", "npts": len(pts),
                       "points": [list(p) for p in pts]}, open(ref_file, "w"))
            print(f"  refmap de sim creado: {ref_file} ({len(pts)} puntos)")

    print(f">>> SIM ADAPTER: conectando a rosbridge {SIM_URL} ...")
    bridge = RosBridge(SIM_URL)
    cdp = SimCDP(bridge)
    print("  esperando /odom (hasta 30s; el mundo del lab tarda en cargar)...", end="", flush=True)
    for i in range(120):
        if bridge.odom is not None:
            print(" ok")
            break
        if i % 8 == 7:
            print(".", end="", flush=True)
        time.sleep(0.25)
    if bridge.odom is None:
        print("\n  SIN /odom: la sim no esta publicando. Comprueba en la T1 del contenedor que salio")
        print("  'Successfully spawned entity [g1]' (con GUI la carga puede matar el spawn: usa headless).")
        sys.exit(1)
    po = cdp._pose7()
    print(f"  /odom OK pose=({po[0]:+.2f},{po[1]:+.2f}) | /scan: "
          f"{'OK' if bridge.scan else 'esperando...'}")

    import g1_nav_v2 as g
    import g1_goto
    g.get_cdp = lambda: cdp                                    # todo el stack usa el SimCDP
    g1_goto.get_live_cdp = lambda *a, **k: cdp

    # --- MODELO DE DERRAME (condicion payload; G1_SPILL=0 lo apaga) --------------------------
    # Fisica del sloshing de la taza sobre /odom (ver g1_spill_model.py). Solo OBSERVA: no toca
    # la fisica ni el control. Eventos kind='spill' al RunRecorder + spills_sim/... en summary.
    if os.environ.get("G1_SPILL", "1") == "1":
        import g1_spill_model as _sp
        _active = {"rec": None, "model": None}

        _orig_init = g1_goto.RunRecorder.__init__
        def _sp_init(self, *a, **k):
            _orig_init(self, *a, **k)
            seed = os.environ.get("G1_SPILL_SEED") or os.path.basename(self.fname)
            _active["rec"], _active["model"] = self, _sp.SpillModel(seed=seed)
            print(f"  [spill] payload de agua simulado (semilla={seed})")
        g1_goto.RunRecorder.__init__ = _sp_init

        _orig_pub = bridge.publish_cmd
        def _sp_pub(vx, vy, wz):
            m = _active.get("model")
            if m is not None:
                m.set_cmd(vx, vy, wz)
            return _orig_pub(vx, vy, wz)
        bridge.publish_cmd = _sp_pub

        _orig_finish = g1_goto.RunRecorder.finish
        def _sp_finish(self, result, summary):
            m = _active.get("model")
            if m is not None and _active.get("rec") is self:
                summary = dict(summary or {})
                summary.update(m.summary())
                print(f"  [spill] derrames={len(m.spills)} E[N]={m.expected:.2f} "
                      f"eta_max={m.eta_max_ratio:.2f} riesgo={summary['spill_risk_pct']}%")
            return _orig_finish(self, result, summary)
        g1_goto.RunRecorder.finish = _sp_finish

        def _sp_thread():
            last = None
            while True:
                time.sleep(0.03)
                m, rec = _active.get("model"), _active.get("rec")
                od = bridge.odom
                if m is None or od is None:
                    continue
                st = od.get("header", {}).get("stamp", {})
                key = (st.get("sec", 0), st.get("nanosec", st.get("nsec", 0)))
                if key == last:
                    continue
                last = key
                t = key[0] + key[1] * 1e-9
                p = od["pose"]["pose"]["position"]; q = od["pose"]["pose"]["orientation"]
                yaw = math.atan2(2 * (q["w"] * q["z"] + q["x"] * q["y"]),
                                 1 - 2 * (q["y"] ** 2 + q["z"] ** 2))
                ev = m.step(t, p["x"], p["y"], yaw)
                if ev and rec is not None:
                    try:
                        rec.event("spill", time.time() - rec.t0, p["x"], p["y"],
                                  extra={"src": "sim_model", "eta_ratio": ev["eta_ratio"],
                                         "v": ev["v"], "a": ev["a"]})
                        print(f"  [spill] DERRAME t={ev['t']} eta={ev['eta_ratio']} v={ev['v']}")
                    except Exception:
                        pass
        threading.Thread(target=_sp_thread, daemon=True).start()
    # ------------------------------------------------------------------------------------------
    if scenario == "lab":
        # escenario LAB: ficheros REALES (waypoints.json, refmap summit, nav_map) y engagement ON —
        # el mundo lab.world esta en el MISMO frame G1, asi que no hay nada que parchear salvo
        # proteger el nav_map.json real de escrituras (waypoint/sweep en sim escriben en copia).
        nm_lab = os.path.join(SIM_DIR, "nav_map_lab.json")
        if not os.path.exists(nm_lab):
            if os.environ.get("G1_SIM_FURN") == "1" and os.path.exists(g1_goto.MAP_FILE):
                import shutil                                  # mundo CON muebles: sembrar los reales
                shutil.copy(g1_goto.MAP_FILE, nm_lab)
            else:                                              # lab.world v1 = SOLO PAREDES: mapa de
                json.dump({"cells": [], "OCELL": 0.2,          # muebles vacio (coherente con el mundo)
                           "frame": "map", "hband": [-0.5, 0.6]}, open(nm_lab, "w"))
        g1_goto.MAP_FILE = nm_lab
        print("  escenario LAB: mapa/waypoints/puerta REALES (lab.world solo-paredes, frame G1)")
    else:
        g1_goto.WP_FILE = wp_file                              # escenario sintetico (no toca lo del lab)
        g1_goto.MAP_FILE = os.path.join(SIM_DIR, "nav_map_sim.json")
        _pts = [(p[0], p[1]) for p in json.load(open(ref_file))["points"]]
        g1_goto.ref_points = lambda: list(_pts)
        g1_goto.load_static_map = lambda: set()                # sin muebles conocidos en la room v1

    sys.argv = ["g1_goto.py"] + sys.argv[1:]
    g1_goto.main()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(0)
    main()
