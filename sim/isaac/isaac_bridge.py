"""Isaac como GEMELO PILOTABLE: sirve el dialecto rosbridge que ya habla g1_sim_adapter.

Por que asi y no por ROS2: Isaac 5.1 lleva ROS2 Jazzy por dentro y el gemelo de Gazebo usa
Humble; mezclar distros por DDS es fragil. El adaptador solo necesita TRES cosas —suscribir
/odom y /scan, publicar /cmd_vel—, asi que este script implementa ese subconjunto del
protocolo rosbridge sobre websocket. `g1_sim_adapter.py` se conecta SIN TOCARLO, apuntando a
este puerto en vez de al del contenedor de Gazebo.

Cuerpo del robot: CINEMATICO (integra cmd_vel), igual que el gemelo de Gazebo — la marcha
articulada es el peldano 6, no este. El lidar es el RTX ya calibrado (peldano 1) con el
filtro de la app aplicado.

    G1_ISAAC_PORT (8766)   G1_ISAAC_SCENE (/ws/office3d.usd)
"""
import asyncio, json, math, os, threading, time

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": 640, "height": 360})

import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import open_stage
from isaacsim.sensors.rtx import LidarRtx
from pxr import UsdGeom, Gf

PORT = int(os.environ.get("G1_ISAAC_PORT", "8766"))
ESCENA = os.environ.get("G1_ISAAC_SCENE", "/ws/office3d.usd")
X0, Y0, YAW0 = 0.99, 0.57, math.radians(-120.0)
H_LIDAR = 0.55
DT = 0.05
QUIETO = os.environ.get("G1_ISAAC_LIDAR_QUIETO", "") == "1"

ESTADO = {"x": X0, "y": Y0, "yaw": YAW0, "vx": 0.0, "vy": 0.0, "wz": 0.0,
          "t_cmd": 0.0, "scan": [], "seq": 0}
LOCK = threading.Lock()

open_stage(ESCENA)
# sin physics_dt/rendering_dt propios: la campana de calibracion (que SI produce rayos) usa
# el World por defecto, y forzar la cadencia parece dejar sin alimentar al sensor RTX.
world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()

# cuerpo visible del robot (cinematico): un cilindro a la altura del G1
cuerpo = UsdGeom.Cylinder.Define(stage, "/World/Robot")
cuerpo.CreateRadiusAttr(0.16); cuerpo.CreateHeightAttr(1.2)
xf_c = UsdGeom.Xformable(cuerpo.GetPrim()); xf_c.ClearXformOpOrder()
OP_C = xf_c.AddTranslateOp()

lidar = LidarRtx(prim_path="/World/LidarNav", name="lnav",
                 position=np.array([X0, Y0, H_LIDAR]),
                 config_file_name="Example_Rotary_2D")
lidar.attach_annotator("IsaacComputeRTXLidarFlatScan")
world.reset()
# NO se limpian las xformOps del lidar: hacerlo despues de world.reset() rompe su vinculo con
# el producto de render y el anotador devuelve 0 rayos (medido). Se reutilizan las que trae:
# traslacion, y orientacion por CUATERNION (xformOp:orient), que es su tipo nativo.
xf_l = UsdGeom.Xformable(stage.GetPrimAtPath("/World/LidarNav"))
OP_L = OP_LR = None
for _op in xf_l.GetOrderedXformOps():
    _t = _op.GetOpType()
    if _t == UsdGeom.XformOp.TypeTranslate and OP_L is None:
        OP_L = _op
    elif _t == UsdGeom.XformOp.TypeOrient and OP_LR is None:
        OP_LR = _op
if OP_L is None:
    OP_L = xf_l.AddTranslateOp()
print("[lidar] ops: translate=%s orient=%s" % (OP_L is not None, OP_LR is not None), flush=True)
ann = list(lidar.get_annotators().values())[0]

# --- filtro de la app medido en 1444 snapshots reales (peldano 1) ---
CAP, HIST, BUDGET = 3.7, {0: 0.12, 1: 0.45, 2: 0.37, 3: 0.06}, 62
import random as _rnd
_r = _rnd.Random(7)

def filtra(perfil):
    """Empobrece el scan RTX hasta el reparto de rango del lidar real de la app."""
    cand = {b: v for b, v in perfil.items() if v <= CAP}
    porb = {}
    for b, v in cand.items():
        porb.setdefault(min(int(v), 3), []).append(b)
    out = {}
    for k, bs in porb.items():
        p = min(1.0, HIST.get(k, 0) * BUDGET / max(1, len(bs)))
        for b in bs:
            if _r.random() < p:
                out[b] = cand[b]
    return out

def paso_fisico():
    with LOCK:
        e = ESTADO
        # cmd_vel caduca: si el piloto calla, el robot para (como el gemelo)
        if time.time() - e["t_cmd"] > 0.6:
            e["vx"] = e["vy"] = e["wz"] = 0.0
        c, s = math.cos(e["yaw"]), math.sin(e["yaw"])
        e["x"] += (c * e["vx"] - s * e["vy"]) * DT
        e["y"] += (s * e["vx"] + c * e["vy"]) * DT
        e["yaw"] = (e["yaw"] + e["wz"] * DT + math.pi) % (2 * math.pi) - math.pi
        x, y, yaw = e["x"], e["y"], e["yaw"]
    OP_C.Set(Gf.Vec3d(x, y, 0.6))
    # PRUEBA: el lidar rotatorio necesita completar una revolucion; si se le reescribe la pose
    # cada fotograma puede no acumular nunca. Con G1_ISAAC_LIDAR_QUIETO=1 se deja fijo.
    if not QUIETO:
        OP_L.Set(Gf.Vec3d(x, y, H_LIDAR))
        if OP_LR is not None:
            OP_LR.Set(Gf.Quatd(math.cos(yaw / 2.0), Gf.Vec3d(0, 0, math.sin(yaw / 2.0))))

# ---------------------------------------------------------------- barrido GEOMETRICO del laser
# Por que no el lidar RTX aqui: en corridas dedicadas produce (peldanos 1-2), pero dentro de
# este puente el anotador FlatScan devuelve 0 rayos de forma persistente -- probado con lidar
# quieto, sin cadencia propia y con las xformOps originales. Queda anotado como pendiente.
# El barrido geometrico contra el mapa da lo que la navegacion necesita Y aprovecha mejor la
# calibracion: el CRISTAL se modela como paso probabilistico con la firma medida en banco
# (44% de ausencia de frente, 32% a 30 grados), en vez de depender de las tiras.
_OC = 0.2
_MAPA = set()
_CRISTAL = set()
try:
    _pts = json.load(open("/ws/ref_map_g1.json"))["points"]
    _MAPA = {(round(p[0] / _OC), round(p[1] / _OC)) for p in _pts}
    _nav = json.load(open("/ws/nav_map.json")).get("cells", [])
    _MAPA |= {(int(c[0]), int(c[1])) for c in _nav}
    _R0, _R1 = (-3.75, -0.55), (-2.65, 0.75)
    _blob = {c for c in _MAPA if _R0[0] <= c[0]*_OC <= _R1[0] and _R0[1] <= c[1]*_OC <= _R1[1]}
    _pane = {}
    for c in _blob:
        if c[1] not in _pane or c[0] > _pane[c[1]][0]:
            _pane[c[1]] = c
    _CRISTAL = set(_pane.values())
    _MAPA -= (_blob - _CRISTAL)                 # artefactos tras el cristal: fuera
    _DOOR, _DR = (-3.90, 1.25), 0.55
    _MAPA = {c for c in _MAPA
             if math.hypot(c[0]*_OC - _DOOR[0], c[1]*_OC - _DOOR[1]) >= _DR}
    print("[scan] mapa: %d celdas (%d de cristal)" % (len(_MAPA), len(_CRISTAL)), flush=True)
except Exception as _e:
    print("[scan] AVISO: sin mapa (%s)" % _e, flush=True)

NORMAL_CRISTAL = 0.0        # el panel mira al ESTE (+x)

def _p_ausencia(ang_inc):
    """Firma del cristal calibrada en banco: 44% de ausencia de frente, 32% a 30 grados."""
    a_ = min(60.0, abs(ang_inc))
    return max(0.10, 0.44 - (0.44 - 0.32) * (a_ / 30.0))

def barrido(x, y, yaw):
    """Primer retorno por sector de 2 grados, con el cristal como paso probabilistico."""
    if not _MAPA:
        return {}
    out = {}
    for k in range(180):
        b = k * 2
        a_ = math.radians(b)
        ca, sa = math.cos(a_), math.sin(a_)
        r = 0.15
        while r <= 12.0:
            c = (round((x + ca * r) / _OC), round((y + sa * r) / _OC))
            if c in _MAPA:
                if c in _CRISTAL:
                    inc = abs(((math.degrees(a_) - 180.0 - NORMAL_CRISTAL + 180) % 360) - 180)
                    if _r.random() < _p_ausencia(inc):
                        r += _OC                      # el rayo ATRAVIESA el cristal
                        continue
                out[b] = round(r, 3)
                break
            r += 0.05
    return out


def lee_scan():
    try:
        d = ann.get_data()
    except Exception:
        return None
    rg = np.asarray(d.get("linearDepthData", [])).ravel()
    if rg.size < 900:
        return None
    azr = d.get("azimuthRange")
    az = np.linspace(float(azr[0]), float(azr[1]), rg.size, endpoint=False)
    perfil = {}
    for b, v in zip(az % 360.0, rg):
        if v < 0.05 or not np.isfinite(v):
            continue
        k = int(b // 2) * 2
        if k not in perfil or v < perfil[k]:
            perfil[k] = float(v)
    return filtra(perfil)

def msg_odom():
    with LOCK:
        x, y, yaw = ESTADO["x"], ESTADO["y"], ESTADO["yaw"]
        vx, wz = ESTADO["vx"], ESTADO["wz"]
    return {"header": {"frame_id": "odom"},
            "pose": {"pose": {"position": {"x": x, "y": y, "z": 0.0},
                              "orientation": {"x": 0.0, "y": 0.0,
                                              "z": math.sin(yaw / 2), "w": math.cos(yaw / 2)}}},
            "twist": {"twist": {"linear": {"x": vx, "y": 0.0, "z": 0.0},
                                "angular": {"x": 0.0, "y": 0.0, "z": wz}}}}

def msg_scan(perfil):
    """LaserScan de 360 rayos a 1 grado; sin retorno = inf, como el real."""
    n = 360
    rr = []
    for i in range(n):
        b = int((i * 360.0 / n) // 2) * 2
        rr.append(perfil.get(b, float("inf")))
    return {"header": {"frame_id": "base_link"},
            "angle_min": -math.pi, "angle_max": math.pi,
            "angle_increment": 2 * math.pi / n,
            "range_min": 0.05, "range_max": 12.0, "ranges": rr}

# --------------------------------------------------------------------- websocket rosbridge
CLIENTES = {}          # ws -> conjunto de topics suscritos (un set no es hashable en tupla)

async def handler(ws):
    subs = set()
    CLIENTES[ws] = subs
    try:
        async for raw in ws:
            try:
                m = json.loads(raw)
            except Exception:
                continue
            op = m.get("op")
            if op == "subscribe":
                subs.add(m.get("topic"))
            elif op == "unsubscribe":
                subs.discard(m.get("topic"))
            elif op == "publish" and m.get("topic") == "/cmd_vel":
                tw = m.get("msg") or {}
                lin, ang = tw.get("linear") or {}, tw.get("angular") or {}
                with LOCK:
                    ESTADO["vx"] = float(lin.get("x", 0.0))
                    ESTADO["vy"] = float(lin.get("y", 0.0))
                    ESTADO["wz"] = float(ang.get("z", 0.0))
                    ESTADO["t_cmd"] = time.time()
    except Exception:
        pass
    finally:
        CLIENTES.pop(ws, None)

async def emisor():
    import websockets  # noqa
    while True:
        await asyncio.sleep(0.1)
        with LOCK:
            perfil = dict(ESTADO["scan"]) if ESTADO["scan"] else {}
        od = json.dumps({"op": "publish", "topic": "/odom", "msg": msg_odom()})
        sc = json.dumps({"op": "publish", "topic": "/scan", "msg": msg_scan(perfil)})
        for ws, subs in list(CLIENTES.items()):
            try:
                if "/odom" in subs:
                    await ws.send(od)
                if "/scan" in subs:
                    await ws.send(sc)
            except Exception:
                CLIENTES.pop(ws, None)

def hilo_ws():
    import websockets
    async def main():
        async with websockets.serve(handler, "0.0.0.0", PORT, ping_interval=None):
            print("=== PUENTE ISAAC EN ws://0.0.0.0:%d ===" % PORT, flush=True)
            await emisor()
    asyncio.run(main())

threading.Thread(target=hilo_ws, daemon=True).start()

print("=== SIMULACION ISAAC LISTA ===", flush=True)
n = 0
while True:
    paso_fisico()
    world.step(render=True)
    n += 1
    if n % 4 == 0:
        with LOCK:
            _x, _y, _yaw = ESTADO["x"], ESTADO["y"], ESTADO["yaw"]
        s = filtra(barrido(_x, _y, _yaw))
        with LOCK:
            ESTADO["scan"] = s
        if n % 400 == 0:
            print("[scan] sectores con retorno: %d de 180" % len(s), flush=True)
