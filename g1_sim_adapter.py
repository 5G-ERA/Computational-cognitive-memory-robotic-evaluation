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
import random
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
                                       ("/scan", "sensor_msgs/LaserScan", 250),
                                       ("/spill_event", "std_msgs/Empty", 0)):
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
                elif m["topic"] == "/spill_event":
                    # derrame marcado a mano (ros2 topic pub /spill_event std_msgs/msg/Empty)
                    # -> reenviar al listener UDP de g1_goto (mismo canal que el robot real)
                    try:
                        import socket as _sk
                        _p = int(os.environ.get("G1_SPILL_GT_PORT", "7777") or 0)
                        if _p:
                            _sk.socket(_sk.AF_INET, _sk.SOCK_DGRAM).sendto(b"spill", ("127.0.0.1", _p))
                    except Exception:
                        pass

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


# --- RUIDO DE SENSORES (G1_SIM_NOISE=1; calibrado contra runs reales del 24-jul) ---
# Objetivo medido: laser_noise real p50=0.147 (gemelo limpio 0.058), c0_std real p50=0.072
# (gemelo 0.030), loc_conf real p50=0.90 (gemelo 1.0 constante). El ruido va SOLO aqui
# (el codigo del robot no se toca): rayo laser gaussiano + dropout, y deriva de odometria
# random-walk por metro; la loc_conf baja de forma EMERGENTE via el relocalizador real.
SIM_NOISE = os.environ.get("G1_SIM_NOISE", "") == "1"
# --- CRISTAL SIMULADO (G1_SIM_GLASS, 17-ago): el par testigo W1 en el gemelo -----------------
# Un lidar no ve el vidrio. Aqui se reproduce donde de verdad ocurre -- en el sensor -- y no
# tocando el mundo: los retornos que caen dentro de un rectangulo declarado se DESCARTAN, mientras
# la pared sigue existiendo en el mundo de Gazebo (el robot choca con ella) y en el mapa de
# referencia (el planificador la conoce). Eso es exactamente un cristal, y monta las dos mitades
# de W1 con la misma geometria:
#     cristal      -> el mapa predice retorno y el barrido NO lo da  -> cov_def alto
#     vano abierto -> el mapa tampoco predice retorno                -> cov_def ~0
# Formato: "x0,y0,x1,y1" en coordenadas del mapa; varios rectangulos separados por ';'.
def _rects(sv):
    out = []
    for r in (sv or "").split(";"):
        v = [t for t in r.replace(" ", "").split(",") if t]
        if len(v) == 4:
            try:
                x0, y0, x1, y1 = (float(t) for t in v)
                out.append((min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)))
            except ValueError:
                pass
    return out


SIM_GLASS = _rects(os.environ.get("G1_SIM_GLASS", ""))
NZ_SIG_R = float(os.environ.get("G1_SIM_NOISE_R", "0.09"))        # sigma por rayo (m)
NZ_P_DROP = float(os.environ.get("G1_SIM_NOISE_DROP", "0.01"))    # prob. dropout por rayo
NZ_DRIFT = float(os.environ.get("G1_SIM_NOISE_DRIFT", "0.07"))    # deriva pos (m por m) [calibrado 31-jul]
NZ_DYAW = math.radians(float(os.environ.get("G1_SIM_NOISE_DYAW", "1.5")))  # deriva yaw (rad por m) [calibrado]
NZ_BIAS_S = float(os.environ.get("G1_SIM_NOISE_BIAS", "0.03"))    # sesgo comun AR(1) por barrido (m)
NZ_BURST_P = float(os.environ.get("G1_SIM_NOISE_BURST_P", "0.05"))   # prob. de rafaga por barrido
NZ_BURST_LEN = int(os.environ.get("G1_SIM_NOISE_BURST_LEN", "3"))    # barridos que dura la rafaga
NZ_BURST_GAIN = float(os.environ.get("G1_SIM_NOISE_BURST_GAIN", "4.0"))  # sigma x gain en rafaga
# --- MARCHA BIPEDA (G1_SIM_GAIT=1, 21-ago): el bamboleo que el gemelo rigido no tiene ---------
# El G1 real CAMINA: la IMU en marcha (8 runs, 20-ago) da pitch mediana 0.028 rad (p90 0.057),
# roll 0.019, acel lateral 0.98 m/s2 (p90 2.4) y vaiven de yaw 0.098 rad/s -- y su firma en el
# barrido es RUIDO CONTINUO Y CORRELADO: c0_std real 0.087 vs 0.035 del gemelo (2.5x), con
# loc_conf real 0.964 vs 1.000. El modelo blanco+rafagas no puede producir eso.
# Aqui: oscilador de ZANCADA cuya fase avanza solo en movimiento. Cuatro efectos coherentes:
#   - modulacion de rango proa/popa a frecuencia de PASO (2x zancada): el cabeceo,
#   - modulacion babor/estribor a frecuencia de zancada: el balanceo,
#   - vaiven de yaw y bamboleo lateral de la pose reportada.
# El gemelo es 2D: el mecanismo fisico real (el plano del laser barriendo bandas de altura de
# los muebles) NO es representable, asi que las amplitudes son PARAMETROS EFECTIVOS en metros,
# calibrados contra los estadisticos de barrido reales -- no angulos fisicos. La frecuencia no
# es identificable con muestras a 0.32 s (aliasing): va declarada (zancada ~0.7 Hz del G1).
# Amplitudes por defecto CONGELADAS el 20-ago (noche) tras validar en la estacion (ver
# sim/NOISE_GAIT.md): laser_noise 0.181 vs 0.173 real, c0_std 0.080 vs 0.087, 6/6 sin colision.
SIM_GAIT = os.environ.get("G1_SIM_GAIT", "") == "1"
GAIT_F = float(os.environ.get("G1_SIM_GAIT_F", "0.7"))            # zancada (Hz); paso = 2x
GAIT_AP = float(os.environ.get("G1_SIM_GAIT_AP", "0.04"))         # rango proa/popa (m, efectivo)
GAIT_AR = float(os.environ.get("G1_SIM_GAIT_AR", "0.03"))         # rango babor/estribor (m)
GAIT_AW = float(os.environ.get("G1_SIM_GAIT_AW", "0.015"))        # vaiven de yaw (rad)
GAIT_AY = float(os.environ.get("G1_SIM_GAIT_AY", "0.025"))         # bamboleo lateral pose (m)
GAIT_VMIN = 0.02                                                  # m/s: por debajo, parado
# --- CAMARA SINTETICA (G1_SIM_PERC=1): habilita DOOR-VIS en el gemelo ---
# g1_goto exige un frame real (CAM_JS) y un servidor /perceive vivos. Aqui: SimCDP sirve un
# JPEG dummy 320x180 y un hilo HTTP responde /health + /perceive con la deteccion 'door'
# calculada desde la pose VERDADERA del sim (una camara real mide el bearing FISICO), con
# realismo medido en runs reales: latencia ~0.28s, dropout 4%, bearing +-1.5deg, rango 5%.
# El codigo del robot NO se toca: para g1_goto es un perception_server normal.
SIM_PERC = os.environ.get("G1_SIM_PERC", "") == "1"
SIM_PERC_PORT = int(os.environ.get("G1_SIM_PERC_PORT", "8010"))
SP_DOOR = (float(os.environ.get("G1_DOOR_X", "-3.90")),
           float(os.environ.get("G1_DOOR_Y", "1.25")))     # mismo centro que usa g1_goto
SP_LAT = float(os.environ.get("G1_SIM_PERC_LAT", "0.28"))  # latencia media (s)

# LATENCIA DE INTERFAZ (23-ago). El lazo de control es el MISMO codigo en real y en gemelo
# (duerme 0.1 s fijos al final del tic); lo que cambia es cuanto tarda la interfaz en
# responder. Medido sobre 38647 tics reales: periodo 0.300 s [IQR 0.290-0.340], es decir la
# interfaz consume 0.200 s por tic. El gemelo consumia 0.000 y ticaba a 10 Hz rigidos, o sea
# TOMABA EL TRIPLE DE DECISIONES que el robot real, y ademas sobre odometria repetida: el
# 68.7% de sus muestras consecutivas llevaban pose IDENTICA (real: 2.7%).
# La tasa de frontera de decision no es cosmetica en este banco, es una meta-variable.
# OJO: esto solo es correcto CON EL FRENO DE RELOJ puesto en el puente. Sin el, bloquear
# 0.2 s de pared adelantaria ~1.5 s de simulacion y el efecto seria el contrario.
IFACE_LAT = float(os.environ.get("G1_SIM_IFACE_LAT", "0.20"))   # s por tic (0 = desactiva)
IFACE_JIT = float(os.environ.get("G1_SIM_IFACE_JIT", "0.035"))  # dispersion medida
SP_DROP = float(os.environ.get("G1_SIM_PERC_DROP", "0.04"))  # dropout de respuesta
SP_BNOISE = float(os.environ.get("G1_SIM_PERC_BDEG", "1.5"))  # ruido de bearing (deg)

# --- ILUMINACION del gemelo y sus dos consecuencias MEDIDAS en el robot real ---
# G1_SIM_LUZ es el brillo de fotograma que declara la escena: ~116 = todas las luces,
# ~85 = poca luz (los dos valores medidos el 21-ago). De el dependen:
#   (a) el canal de OBJETOS, via el emulador calibrado (sim/emulador_deteccion.py): a 1.8 m
#       la confianza de silla pasa de 0.82 con luz a 0.54 a oscuras -> el testigo W2;
#   (b) el canal de PUERTA: con luz plena el robot real produjo 2-4x mas observaciones de
#       vano y SESGADAS +8.8 grados (89 muestras vs 19-41 a oscuras, mediana +8.8 vs +0.9/-6.1),
#       que es lo que empujo el cruce a +0.134 y provoco el golpe. Se reproduce aqui para
#       poder ENSAYAR el gate de iluminacion antes de la sesion real.
SIM_LUZ = float(os.environ.get("G1_SIM_LUZ", "116"))

# --- CANAL DE ESCENIFICACION (24-ago): las doce configuraciones son TRANSICIONES ----------
# Hasta hoy la escena del gemelo eran constantes de modulo leidas del entorno al importar,
# asi que solo sabia correr una condicion FIJA. Ninguna de las T1-T12 es una condicion fija:
# todas son un cambio en un instante. (Efecto lateral util: por eso ninguna configuracion
# reservada ha podido contaminarse durante el desarrollo -- era estructuralmente imposible
# escenificar una transicion, no una precaucion que tomamos.)
#
# El canal hace DOS cosas con un solo mecanismo, y esa union es deliberada:
#   1. aplica el cambio declarado en el instante declarado, y
#   2. escribe el REGISTRO INDEPENDIENTE de cuando ocurrio.
# El §3 de la pre-registracion exige que Omega_t venga del guion del experimento y no de la
# telemetria del sistema bajo prueba. Aqui el guion ES quien mueve la escena, asi que el
# registro no se infiere: se emite en el mismo acto que provoca la transicion. No hay forma
# de que diverjan porque no son dos fuentes.
ESCENA_F = os.environ.get("G1_SIM_ESCENA", "")          # JSON que el guion reescribe en vivo
ESCENA_LOG = os.environ.get("G1_SIM_ESCENA_LOG", "/tmp/g1_escena_registro.jsonl")
_escena_mtime = 0.0
_escena_estado = {}


def _escena_aplica(nuevo_estado, motivo="guion"):
    """Aplica un estado de escena declarado y ANOTA la transicion. Devuelve lo que cambio."""
    global SIM_LUZ, SIM_GLASS, _DUMMY_URI, _escena_estado
    cambios = {}
    if "luz" in nuevo_estado:
        v = float(nuevo_estado["luz"])
        if abs(v - SIM_LUZ) > 1e-6:
            cambios["luz"] = [SIM_LUZ, v]
            SIM_LUZ = v
            try:
                _DUMMY_URI = _dummy_frame() if SIM_PERC else ""   # la luma sigue a la luz
            except Exception:
                pass
    if "cristal" in nuevo_estado:
        v = _rects(nuevo_estado["cristal"] or "")
        if v != SIM_GLASS:
            cambios["cristal"] = [len(SIM_GLASS), len(v)]
            SIM_GLASS = v
    if cambios:
        _escena_estado = dict(nuevo_estado)
        reg = {"t_pared": time.time(), "motivo": motivo, "cambios": cambios,
               "estado": dict(nuevo_estado)}
        try:
            with open(ESCENA_LOG, "a") as fh:
                fh.write(json.dumps(reg) + "\n")
        except Exception:
            pass
        print("  [ESCENA] %s -> %s" % (motivo, cambios), flush=True)
    return cambios


def _escena_vigila():
    """Relee el fichero de escena cuando cambia. Hilo aparte: no toca el lazo de control."""
    global _escena_mtime
    while True:
        try:
            m = os.path.getmtime(ESCENA_F)
            if m > _escena_mtime:
                _escena_mtime = m
                _escena_aplica(json.load(open(ESCENA_F)), motivo="fichero")
        except Exception:
            pass
        time.sleep(0.2)

SIM_DOOR_BIAS = float(os.environ.get("G1_SIM_DOOR_BIAS", "8.8"))   # sesgo con luz plena
SIM_DOOR_P_LUZ = float(os.environ.get("G1_SIM_DOOR_P_LUZ", "0.95"))
SIM_DOOR_P_OSC = float(os.environ.get("G1_SIM_DOOR_P_OSC", "0.30"))
SP_DOOR_RMIN = float(os.environ.get("G1_SIM_DOOR_RMIN", "0.5"))
SP_DOOR_RMAX = float(os.environ.get("G1_SIM_DOOR_RMAX", "4.2"))
SP_DOOR_FOV = float(os.environ.get("G1_SIM_DOOR_FOV", "28"))

_EMU = None
def _emulador():
    """Emulador de deteccion calibrado (carga perezosa; si falta, el canal queda mudo)."""
    global _EMU
    if _EMU is None:
        try:
            sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "sim"))
            from emulador_deteccion import carga_por_defecto
            _EMU = carga_por_defecto(os.path.dirname(os.path.abspath(__file__)))
            print("[sim-perc] emulador calibrado: %d objetos, brillo declarado %.0f"
                  % (len(_EMU.objetos), SIM_LUZ))
        except Exception as e:
            print("[sim-perc] AVISO: sin emulador (%s); solo canal de puerta" % e)
            _EMU = False
    return _EMU or None


def _dummy_frame():
    """JPEG 320x180 gris-moqueta como data URI (frame neutro: todo 'suelo' para el canal de color).
    G1_SIM_PERC_LUMA escala el brillo (validacion del gate de iluminacion del DOOR-VIS: el
    dummy por defecto da luma ~118 = 'toda la luz'; con LUMA=80 simula poca luz)."""
    try:
        from PIL import Image
        import io, base64
        base_rgb = (126, 116, 104)                       # luma ~117.7
        # UNA SOLA FUENTE DE VERDAD PARA LA LUZ DECLARADA (23-ago). Antes G1_SIM_LUZ movia
        # el emulador de deteccion y G1_SIM_PERC_LUMA el brillo del fotograma, por separado:
        # un episodio podia declarar "luces apagadas" y entregar un fotograma iluminado, con
        # lo que el gate de iluminacion y Omega_t discrepaban sin que nadie lo viera. (Me paso
        # al validar el rol: puse LUZ=85 y la luma siguio en 118.) Ahora la luz declarada la
        # fija G1_SIM_LUZ y el fotograma la sigue; G1_SIM_PERC_LUMA queda como anulacion
        # EXPLICITA para escenificar una discrepancia a proposito, que es un testigo util
        # (el detector ve una cosa y el fotograma dice otra), no un descuido.
        try:
            objetivo = float(os.environ.get("G1_SIM_PERC_LUMA", "") or SIM_LUZ)
        except ValueError:
            objetivo = SIM_LUZ
        if objetivo > 0:
            f = objetivo / 117.7
            base_rgb = tuple(max(0, min(255, int(round(c * f)))) for c in base_rgb)
        buf = io.BytesIO()
        Image.new("RGB", (320, 180), base_rgb).save(buf, format="JPEG", quality=50)
        return "data:image/jpeg;base64," + base64.b64encode(buf.getvalue()).decode()
    except Exception:
        return ""


def start_sim_perc(bridge):
    """Servidor de percepcion FALSO (hilo demonio). Bearing de puerta desde la pose verdadera."""
    import http.server, socketserver

    def true_pose():
        od = bridge.odom
        if not od:
            return None
        p = od["pose"]["pose"]["position"]; q = od["pose"]["pose"]["orientation"]
        yaw = math.atan2(2 * (q["w"] * q["z"] + q["x"] * q["y"]),
                         1 - 2 * (q["y"] * q["y"] + q["z"] * q["z"]))
        return p["x"], p["y"], yaw

    class H(http.server.BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, obj):
            body = json.dumps(obj).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            self._send({"ok": True, "mode": "sim", "self_mask": 0.0})

        def do_POST(self):
            n = int(self.headers.get("Content-Length", 0) or 0)
            if n:
                self.rfile.read(n)
            time.sleep(max(0.10, random.gauss(SP_LAT, 0.05)))
            dets = []; door = None
            tp = true_pose()
            if tp and random.random() > SP_DROP:
                dx = SP_DOOR[0] - tp[0]; dy = SP_DOOR[1] - tp[1]
                rng = math.hypot(dx, dy)
                b = (math.degrees(math.atan2(dy, dx) - tp[2]) + 180) % 360 - 180
                _con_luz = SIM_LUZ >= 108.0
                _p_door = SIM_DOOR_P_LUZ if _con_luz else SIM_DOOR_P_OSC
                # VENTANA calibrada contra el conteo real: el robot real registro 89
                # observaciones de vano con luz plena y 19-41 a oscuras dentro de +-1.5 m del
                # umbral; con la ventana estrecha 0.6-2.8 m el gemelo solo daba 13 y el sesgo
                # no llegaba a actuar. Ambas configurables.
                if SP_DOOR_RMIN < rng < SP_DOOR_RMAX and abs(b) < SP_DOOR_FOV and random.random() < _p_door:
                    if _con_luz:
                        b = b + SIM_DOOR_BIAS         # el sesgo medido con todas las luces
                    b = b + random.gauss(0.0, SP_BNOISE)
                    rng = max(0.3, rng * (1.0 + random.gauss(0.0, 0.05)))
                    dets = [{"label": "door", "bearing_deg": round(b, 1), "range_m": round(rng, 2)}]
                    door = {"bearing_deg": round(b, 1), "range_m": round(rng, 2)}
            # --- OBJETOS: emulador calibrado con datos reales ---
            emu = _emulador()
            if emu is not None and tp:
                try:
                    for d in emu.detecta(tp[0], tp[1], math.degrees(tp[2]), SIM_LUZ):
                        dets.append({"label": d[0], "conf": d[1],
                                     "bearing_deg": d[2], "range_m": d[3]})
                except Exception:
                    pass
            out = {"detections": dets, "scan": [], "brillo": SIM_LUZ}
            if door:
                out["door"] = door
            self._send(out)

    class S(socketserver.ThreadingTCPServer):
        allow_reuse_address = True
        daemon_threads = True

    srv = S(("127.0.0.1", SIM_PERC_PORT), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


_DUMMY_URI = _dummy_frame() if SIM_PERC else ""


class SimCDP:
    """Suplanta al CDP del WebView: resuelve los snippets JS de g1_goto/g1_nav_v2 contra ROS."""

    def __init__(self, bridge):
        self.b = bridge
        self._nz_last = None      # ultima pos REAL vista (para la deriva por metro)
        self._nz_dx = 0.0; self._nz_dy = 0.0; self._nz_dyaw = 0.0
        self._nz_bias = 0.0       # sesgo comun del barrido (AR1: reflejos/inclinacion)
        self._nz_burst = 0        # barridos restantes de la rafaga actual
        self._gait_ph = random.uniform(0.0, 2.0 * math.pi)   # fase de zancada (avanza andando)
        self._gait_t = None       # t del ultimo avance de fase
        self._gait_xy = (0.0, 0.0)  # ultima pos para estimar velocidad
        self._gait_v = 0.0        # velocidad estimada (para congelar la fase parado)
        self._cloud = []          # nube plana [x,y,z,...] en frame mapa (z=0: banda de torso)
        self._cloud_t = 0.0

    # --- helpers ---
    def _pose7(self):
        od = self.b.odom
        if not od:
            return None
        p = od["pose"]["pose"]["position"]; q = od["pose"]["pose"]["orientation"]
        if not SIM_NOISE:
            return [p["x"], p["y"], p["z"], q["x"], q["y"], q["z"], q["w"]]
        # deriva random-walk proporcional a la distancia recorrida (odometria imperfecta)
        if self._nz_last is not None:
            dtrav = math.hypot(p["x"] - self._nz_last[0], p["y"] - self._nz_last[1])
            if dtrav > 1e-4:
                self._nz_dx += random.gauss(0.0, NZ_DRIFT * dtrav)
                self._nz_dy += random.gauss(0.0, NZ_DRIFT * dtrav)
                self._nz_dyaw += random.gauss(0.0, NZ_DYAW * dtrav)
        self._nz_last = (p["x"], p["y"])
        yaw = math.atan2(2 * (q["w"] * q["z"] + q["x"] * q["y"]),
                         1 - 2 * (q["y"] * q["y"] + q["z"] * q["z"])) + self._nz_dyaw
        gx = gy = gw = 0.0
        if SIM_GAIT:
            now = time.time()
            if self._gait_t is not None:
                dt = min(0.5, now - self._gait_t)
                if dt > 1e-3:
                    v = math.hypot(p["x"] - self._gait_xy[0], p["y"] - self._gait_xy[1]) / dt
                    self._gait_v = 0.7 * self._gait_v + 0.3 * v
                if self._gait_v > GAIT_VMIN:      # la fase solo avanza ANDANDO
                    self._gait_ph = (self._gait_ph + 2.0 * math.pi * GAIT_F * dt) % (2.0 * math.pi)
            self._gait_t = now; self._gait_xy = (p["x"], p["y"])
            if self._gait_v > GAIT_VMIN:
                s = math.sin(self._gait_ph)
                gw = GAIT_AW * s                  # vaiven de rumbo (zancada)
                gx = -GAIT_AY * s * math.sin(yaw)  # bamboleo lateral, perpendicular al rumbo
                gy = GAIT_AY * s * math.cos(yaw)
        yaw += gw
        return [p["x"] + self._nz_dx + gx, p["y"] + self._nz_dy + gy, p["z"],
                0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]

    def _refresh_cloud(self):
        sc = self.b.scan; po = self._pose7()
        if not sc or not po or self.b.scan_t <= self._cloud_t:
            return
        yaw = math.atan2(2 * (po[6] * po[5] + po[3] * po[4]),
                         1 - 2 * (po[4] * po[4] + po[5] * po[5]))
        a = sc["angle_min"]; inc = sc["angle_increment"]; rmax = sc.get("range_max", 12.0)
        if SIM_NOISE:
            # sesgo comun del barrido (AR1) + rafagas correlacionadas (girando/reflejos:
            # el laser_noise real vive de ESTO, no del ruido blanco por rayo)
            self._nz_bias = 0.9 * self._nz_bias + random.gauss(0.0, NZ_BIAS_S)
            if self._nz_burst > 0:
                self._nz_burst -= 1
            elif random.random() < NZ_BURST_P:
                self._nz_burst = NZ_BURST_LEN
            _burst = self._nz_burst > 0
            _sig = NZ_SIG_R * (NZ_BURST_GAIN if _burst else 1.0)
            _drop = NZ_P_DROP * (8.0 if _burst else 1.0)
        flat = []
        for i, r in enumerate(sc["ranges"]):
            if r is None or not isinstance(r, (int, float)):
                continue
            if SIM_NOISE:
                if random.random() < _drop:
                    continue                       # dropout de rayo
                r = r + self._nz_bias + random.gauss(0.0, _sig)
            if SIM_GAIT and self._gait_v > GAIT_VMIN:
                # coherente con la fase de marcha: cabeceo a frec. de PASO en proa/popa,
                # balanceo a frec. de ZANCADA en babor/estribor (beta = angulo en el cuerpo)
                _beta = a + i * inc
                r = r + (GAIT_AP * math.sin(2.0 * self._gait_ph) * math.cos(_beta)
                         + GAIT_AR * math.sin(self._gait_ph) * math.sin(_beta))
            if not math.isfinite(r) or r <= sc.get("range_min", 0.15) or r >= rmax * 0.98:
                continue
            th = yaw + a + i * inc
            _px = po[0] + r * math.cos(th); _py = po[1] + r * math.sin(th)
            if SIM_GLASS and any(x0 <= _px <= x1 and y0 <= _py <= y1
                                 for x0, y0, x1, y1 in SIM_GLASS):
                continue                           # el vidrio no devuelve: el rayo lo atraviesa
            flat.extend((round(_px, 3), round(_py, 3), 0.0))
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
        if "__camc" in e:                                 # CAM_JS: frame de camara
            return _DUMMY_URI if SIM_PERC else ""
        if "__relocbuf||[]).length" in e:
            self._refresh_cloud()
            return len(self._cloud)
        if "JSON.stringify(window.__relocbuf" in e:
            self._refresh_cloud()
            return json.dumps(self._cloud)
        if "__relocbuf_t" in e:
            return int(self._cloud_t * 1000)
        if "JSON.stringify({pose:" in e:                  # read_pose()
            # el tic real se bloquea aqui esperando a la interfaz; el gemelo no lo hacia
            if IFACE_LAT > 0:
                time.sleep(max(0.02, random.gauss(IFACE_LAT, IFACE_JIT)))
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
    if ESCENA_F:
        threading.Thread(target=_escena_vigila, daemon=True).start()
        print("  [ESCENA] canal activo: %s (registro -> %s)" % (ESCENA_F, ESCENA_LOG))
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
        if SIM_NOISE:
            print("  RUIDO DE SENSORES ON: laser sigma=%.3fm drop=%.1f%% | odom deriva=%.3fm/m yaw=%.2fdeg/m"
                  % (NZ_SIG_R, 100 * NZ_P_DROP, NZ_DRIFT, math.degrees(NZ_DYAW)))
        if SIM_PERC:
            start_sim_perc(bridge)
            print("  CAMARA SINTETICA ON: /perceive en 127.0.0.1:%d (puerta %.2f,%.2f; lat %.2fs drop %.0f%%)"
                  % (SIM_PERC_PORT, SP_DOOR[0], SP_DOOR[1], SP_LAT, 100 * SP_DROP))
            if not _DUMMY_URI:
                print("  !! PIL no disponible: frame dummy vacio -> la vision NO pasara el test de arranque")
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
