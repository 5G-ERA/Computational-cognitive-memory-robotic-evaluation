"""El G1 andando con NUESTRA politica, usando la configuracion de Isaac Lab.

Por que asi: cargar el G1 "a mano" con el controlador estilo H1 fallaba con segfault, porque
la politica se entreno sobre el activo y los actuadores que declara Isaac Lab (G1_MINIMAL_CFG:
otro USD, ganancias de articulacion propias, pose inicial concreta). Aqui se usa esa misma
configuracion, asi que el robot es EXACTAMENTE el que la politica conoce.

    ESCENA=oficina  -> la oficina reconstruida    (por defecto: suelo plano)
"""
import math, os, time

import argparse
from isaaclab.app import AppLauncher
_p = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(_p)
args, _ = _p.parse_known_args()
args.headless = True
app_launcher = AppLauncher(args)
sim_app = app_launcher.app

import numpy as np
import torch
import omni.usd
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.sim import SimulationContext
from isaaclab_assets.robots.unitree import G1_MINIMAL_CFG
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import UsdGeom, Gf, UsdLux, UsdPhysics
from PIL import Image

CKPT = os.environ.get("CKPT", "/ws/IsaacLab/logs/rsl_rl/g1_flat/2026-08-22_21-31-53_g1_noche")
VX = float(os.environ.get("VX", "0.6"))
SEGS = float(os.environ.get("SEGS", "20"))
ESCENA = os.environ.get("ESCENA", "plano")
DIR = "/ws/video_g1lab"

_O = open("/ws/g1lab_result.txt", "w")
def log(*a):
    s = " ".join(str(x) for x in a); print(s, flush=True); _O.write(s + "\n"); _O.flush()

sim = SimulationContext(sim_utils.SimulationCfg(dt=1/200, device="cuda:0"))
stage = omni.usd.get_context().get_stage()

# suelo y luz
sim_utils.GroundPlaneCfg().func("/World/Suelo", sim_utils.GroundPlaneCfg())
sim_utils.DomeLightCfg(intensity=2500.0).func("/World/Luz", sim_utils.DomeLightCfg(intensity=2500.0))

X0, Y0 = (0.99, 0.57) if ESCENA == "oficina" else (0.0, 0.0)
if ESCENA == "oficina":
    add_reference_to_stage(usd_path="/ws/office3d.usd", prim_path="/Oficina")
    n = 0
    for grupo in ("/Oficina/World/Estructura", "/Oficina/World/Cristal"):
        pr = stage.GetPrimAtPath(grupo)
        if pr and pr.IsValid():
            for c in pr.GetChildren():
                if not c.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI.Apply(c); n += 1
    g1v = stage.GetPrimAtPath("/Oficina/World/G1")
    if g1v and g1v.IsValid():
        UsdGeom.Imageable(g1v).MakeInvisible()
    log("oficina cargada, colision en %d prims" % n)

cfg = G1_MINIMAL_CFG.replace(prim_path="/World/Robot")
cfg.init_state.pos = (X0, Y0, 0.74)
robot = Articulation(cfg)
log("G1 creado con la configuracion de Isaac Lab")

# GRABACION opcional (REC=1). Primero interesa la respuesta: ¿camina o no?
REC = os.environ.get("REC", "0") == "1"
ann = None
if REC:
    from isaacsim.core.utils.extensions import enable_extension
    enable_extension("omni.replicator.core")
    sim_app.update()
    import omni.replicator.core as rep
    os.makedirs(DIR, exist_ok=True)
    cam = UsdGeom.Camera.Define(stage, "/World/CamG1")
    cam.CreateFocalLengthAttr(22.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 200.0))
    xc = UsdGeom.Xformable(cam.GetPrim()); xc.ClearXformOpOrder()
    OT = xc.AddTranslateOp(); OR_ = xc.AddRotateXYZOp()
    rp = rep.create.render_product("/World/CamG1", (960, 540))
    ann = rep.AnnotatorRegistry.get_annotator("rgb"); ann.attach([rp])

sim.reset()
log("simulacion lista")

politica = torch.jit.load(CKPT + "/exported/policy.pt").to("cuda:0").eval()
log("politica cargada: %s" % CKPT.split("/")[-1])

dev = robot.device
n_j = robot.num_joints
log("juntas: %d" % n_j)
accion_prev = torch.zeros((1, n_j), device=dev)
cmd = torch.tensor([[VX, 0.0, 0.0]], device=dev)
GRAV = torch.tensor([[0.0, 0.0, -1.0]], device=dev)

def obs():
    d = robot.data
    return torch.cat([d.root_lin_vel_b, d.root_ang_vel_b, d.projected_gravity_b, cmd,
                      d.joint_pos - d.default_joint_pos, d.joint_vel, accion_prev], dim=-1)

pasos = int(SEGS * 200)
nrec = 0; caidas = 0
prev = (X0, Y0)
t0 = time.time()
for k in range(pasos):
    if k % 4 == 0:
        with torch.no_grad():
            accion = politica(obs())
        accion_prev = accion.clone()
        robot.set_joint_position_target(robot.data.default_joint_pos + accion * 0.5)
    robot.write_data_to_sim()
    sim.step(render=(k % 8 == 0))
    robot.update(1/200)
    if k % 8 == 0:
        p = robot.data.root_pos_w[0].cpu().numpy()
        if p[2] < 0.45:
            caidas += 1
        if REC and k % 24 == 0:
            vx_, vy_ = float(p[0])-prev[0], float(p[1])-prev[1]
            prev = (float(p[0]), float(p[1]))
            if math.hypot(vx_, vy_) < 1e-4: vx_, vy_ = 1.0, 0.0
            th = math.atan2(vy_, vx_)
            D, H = 2.6, 1.9
            cx, cy = float(p[0])-D*math.cos(th), float(p[1])-D*math.sin(th)
            OT.Set(Gf.Vec3d(cx, cy, H))
            dx_, dy_ = float(p[0])-cx, float(p[1])-cy
            OR_.Set(Gf.Vec3f(90.0+math.degrees(math.atan2(float(p[2])+0.2-H, math.hypot(dx_, dy_))),
                             0.0, math.degrees(math.atan2(dy_, dx_))-90.0))
            d_ = ann.get_data()
            if d_ is not None and len(d_):
                Image.fromarray(np.asarray(d_)[:, :, :3].astype("uint8")).save(
                    os.path.join(DIR, "g%05d.png" % nrec)); nrec += 1
        if k % 800 == 0:
            log("t=%.1fs pose=(%.2f, %.2f, %.2f)" % (k/200.0, p[0], p[1], p[2]))

p = robot.data.root_pos_w[0].cpu().numpy()
dist = math.hypot(float(p[0])-X0, float(p[1])-Y0)
log("FIN en %.0fs de reloj | pose (%.2f, %.2f, %.2f)" % (time.time()-t0, p[0], p[1], p[2]))
log("desplazamiento %.2f m | caido %d muestras | fotogramas %d" % (dist, caidas, nrec))
log("=== EL G1 CAMINA ===" if p[2] > 0.5 and dist > 0.5 else "=== NO CAMINA ===")
sim_app.close()
