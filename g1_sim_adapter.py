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
DEADZONE = 0.30          # por debajo el G1 real no se mueve; la sim lo replica
K_FWD = 0.75             # ly=0.40 -> 0.30 m/s (medido en real)
K_LAT = 0.75             # misma escala lateral; lx>0 = DERECHA fisica -> vy NEGATIVA en ROS
K_YAW = 1.55             # rad/s por unidad de rx; rx=+0.45 -> -40 deg/s (TURN-CAL real) -> signo -

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
            dz = lambda v: 0.0 if abs(v) < DEADZONE else v
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
    os.environ.setdefault("G1_ENV", "sim")
    os.environ.setdefault("G1_SIM_ID", "room_v1")
    os.environ.setdefault("G1_NOVIS", "1")            # sin camara/percepcion en la sim (por ahora)
    os.environ.setdefault("G1_DOOR_ENGAGE", "0")      # room.world no tiene puerta
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
        print(f"  backend grafico: {matplotlib.get_backend()}"
              + ("  (SIN VENTANA: backend no interactivo!)" if "agg" in matplotlib.get_backend().lower() else ""))
    except Exception as e:
        print("  matplotlib no importable:", repr(e))

    os.makedirs(SIM_DIR, exist_ok=True)
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
    for _ in range(20):
        if bridge.odom is not None:
            break
        time.sleep(0.25)
    if bridge.odom is None:
        print("  SIN /odom: ¿esta la sim lanzada? (ros2 launch g1_sim sim.launch.py gui:=false)")
        sys.exit(1)
    po = cdp._pose7()
    print(f"  /odom OK pose=({po[0]:+.2f},{po[1]:+.2f}) | /scan: "
          f"{'OK' if bridge.scan else 'esperando...'}")

    import g1_nav_v2 as g
    import g1_goto
    g.get_cdp = lambda: cdp                                    # todo el stack usa el SimCDP
    g1_goto.get_live_cdp = lambda *a, **k: cdp
    g1_goto.WP_FILE = wp_file                                  # escenario SIM (no toca lo del lab)
    g1_goto.MAP_FILE = os.path.join(SIM_DIR, "nav_map_sim.json")
    _pts = [(p[0], p[1]) for p in json.load(open(ref_file))["points"]]
    g1_goto.ref_points = lambda: list(_pts)
    g1_goto.load_static_map = lambda: set()                    # sin muebles conocidos en la sim v1

    sys.argv = ["g1_goto.py"] + sys.argv[1:]
    g1_goto.main()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(0)
    main()
