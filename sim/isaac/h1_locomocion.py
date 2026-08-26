"""PELDANO 6, fase 1: locomocion REAL con politica preentrenada, en nuestra oficina.

No hay politica publicada para el G1 (el servidor de NVIDIA solo trae Anymal, Franka, H1 y
Spot), asi que se valida primero la CADENA COMPLETA con el H1 -- el otro humanoide de Unitree:
articulacion + fisica PhysX + politica de red neuronal + lazo de control a 200 Hz. Si esto
anda de verdad en la oficina reconstruida, el unico paso que falta para el G1 es su politica.

Comando de velocidad (vx, vy, wz) fijo por ahora; despues lo dara la navegacion.
"""
import math, os, time

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": 960, "height": 540})

import numpy as np
import omni.usd
import omni.replicator.core as rep
from isaacsim.core.api import World
from isaacsim.core.utils.stage import open_stage
from isaacsim.robot.policy.examples.robots import H1FlatTerrainPolicy
from pxr import UsdGeom, Gf
from PIL import Image

REC = os.environ.get("REC", "1") == "1"
DIR = "/ws/video_h1"
X0, Y0 = 0.99, 0.57
VX = float(os.environ.get("VX", "0.5"))
SEGUNDOS = float(os.environ.get("SEGS", "20"))

_O = open("/ws/h1_result.txt", "w")
def log(*a):
    s = " ".join(str(x) for x in a); print(s, flush=True); _O.write(s + "\n"); _O.flush()

open_stage("/ws/office3d.usd")
world = World(stage_units_in_meters=1.0, physics_dt=1/200, rendering_dt=8/200)
stage = omni.usd.get_context().get_stage()

# SUELO CON COLISION: la escena de inspeccion se construyo sin plano de tierra (se hizo con
# create_new_stage y fisica despojada), asi que el robot caia al vacio -- z bajo a -6786 m.
# Aqui la fisica manda, no la inspeccion.
world.scene.add_default_ground_plane()
log("plano de tierra con colision anadido")

# COLISION EN LAS PAREDES: la escena de inspeccion no la lleva (se despojo la fisica), asi que
# el H1 atravesaba los muros como un fantasma. Para locomocion real hacen falta de verdad.
from pxr import UsdPhysics
_n = 0
for _grupo in ("/World/Estructura", "/World/Cristal"):
    _pr = stage.GetPrimAtPath(_grupo)
    if _pr and _pr.IsValid():
        for _c in _pr.GetChildren():
            if not _c.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(_c)
                _n += 1
log("colision anadida a %d prims de estructura" % _n)

# el G1 estatico de la escena estorba: fuera
g1 = stage.GetPrimAtPath("/World/G1")
if g1 and g1.IsValid():
    UsdGeom.Imageable(g1).MakeInvisible()
    log("G1 estatico de la escena ocultado")

h1 = H1FlatTerrainPolicy(prim_path="/World/H1", name="H1",
                         position=np.array([X0, Y0, 1.05]))
log("H1 creado con su politica de terreno plano")

if REC:
    os.makedirs(DIR, exist_ok=True)
    cam = UsdGeom.Camera.Define(stage, "/World/CamH1")
    cam.CreateFocalLengthAttr(22.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 200.0))
    xc = UsdGeom.Xformable(cam.GetPrim()); xc.ClearXformOpOrder()
    OT = xc.AddTranslateOp(); OR_ = xc.AddRotateXYZOp()
    rp = rep.create.render_product("/World/CamH1", (960, 540))
    ann = rep.AnnotatorRegistry.get_annotator("rgb"); ann.attach([rp])

world.reset()
h1.initialize()
log("mundo listo; empieza la marcha")

cmd = np.array([VX, 0.0, 0.0])
n_fis = 0
n_rec = 0
t0 = time.time()
pasos = int(SEGUNDOS * 200)
caidas = 0
while n_fis < pasos:
    h1.forward(1/200, cmd)
    world.step(render=(n_fis % 8 == 0))
    n_fis += 1
    if n_fis % 8 == 0:
        p, _ = h1.robot.get_world_pose()
        if p[2] < 0.55:
            caidas += 1
        if REC and n_fis % 24 == 0:
            # camara que SIGUE al robot: se coloca detras de su avance y le APUNTA. La version
            # anterior miraba siempre hacia +X desde un punto fijo y el robot no salia ni una vez.
            globals().setdefault("_prev", (float(p[0]), float(p[1])))
            vx_, vy_ = float(p[0]) - _prev[0], float(p[1]) - _prev[1]
            globals()["_prev"] = (float(p[0]), float(p[1]))
            if math.hypot(vx_, vy_) < 1e-4:
                vx_, vy_ = 1.0, 0.0
            th = math.atan2(vy_, vx_)
            D, H = 3.2, 2.3
            cx = float(p[0]) - D * math.cos(th)
            cy = float(p[1]) - D * math.sin(th)
            OT.Set(Gf.Vec3d(cx, cy, H))
            dx_, dy_ = float(p[0]) - cx, float(p[1]) - cy
            dz = float(p[2]) + 0.3 - H
            OR_.Set(Gf.Vec3f(90.0 + math.degrees(math.atan2(dz, math.hypot(dx_, dy_))),
                             0.0, math.degrees(math.atan2(dy_, dx_)) - 90.0))
            d = ann.get_data()
            if d is not None and len(d):
                Image.fromarray(np.asarray(d)[:, :, :3].astype("uint8")).save(
                    os.path.join(DIR, "h%05d.png" % n_rec))
                n_rec += 1
        if n_fis % 400 == 0:
            log("t=%.1fs pose=(%.2f, %.2f, %.2f) fotogramas=%d" % (
                n_fis/200.0, p[0], p[1], p[2], n_rec))

p, _ = h1.robot.get_world_pose()
log("FIN: %d pasos de fisica en %.0fs de reloj" % (n_fis, time.time()-t0))
log("pose final (%.2f, %.2f, %.2f) | desplazamiento %.2f m | muestras caido: %d" % (
    p[0], p[1], p[2], math.hypot(p[0]-X0, p[1]-Y0), caidas))
log("=== H1 CAMINA OK ===" if p[2] > 0.7 and math.hypot(p[0]-X0, p[1]-Y0) > 0.5
    else "=== NO CAMINA (caido o sin avance) ===")
app.close()
