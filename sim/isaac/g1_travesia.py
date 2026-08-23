"""El G1 hace la travesia A -> vano -> B -> vuelta, andando de verdad.

Fisica real (PhysX), piernas movidas por NUESTRA politica entrenada, y la ruta
tomada de la geometria real del mapa: pose A y eje del vano son los mismos
numeros que usa g1_goto.py contra el robot fisico.

    REC=1        graba fotogramas (exige enable_cameras: sin el, Kit carga la
                 experiencia sin canal de render y Replicator no tiene grafo)
    SEGS=90      tope de tiempo simulado
"""
import math, os, time

REC = os.environ.get("REC", "1") == "1"
DEV = os.environ.get("DEV", "cuda:0")

import argparse
from isaaclab.app import AppLauncher
_p = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(_p)
args, _ = _p.parse_known_args()
args.headless = True
args.enable_cameras = REC
app_launcher = AppLauncher(args)
sim_app = app_launcher.app

import numpy as np
import torch
import omni.usd
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.sim import SimulationContext
from isaaclab_assets.robots.unitree import G1_MINIMAL_CFG
from pxr import UsdGeom, Gf, UsdPhysics
from PIL import Image

CKPT = os.environ.get("CKPT", "/ws/IsaacLab/logs/rsl_rl/g1_flat/2026-08-22_21-31-53_g1_noche")
SEGS = float(os.environ.get("SEGS", "90"))
DIR = os.environ.get("DIR", "/ws/video_travesia")
CAM_D = float(os.environ.get("CAM_D", "2.7"))
CAM_H = float(os.environ.get("CAM_H", "3.05"))

# --- geometria REAL, la misma que ve el robot fisico -------------------------
AX, AY, AYAW = 0.99, 0.57, math.radians(-120.0)   # pose A (isaac_bridge.py)
DOOR_X, DOOR_Y = -3.90, 1.25                      # centro del vano (g1_goto.py)
DOOR_AXIS = math.radians(135.0)                   # cruce lado A -> lado B
_ux, _uy = math.cos(DOOR_AXIS), math.sin(DOOR_AXIS)
APROX = (DOOR_X + 1.10 * _ux * -1, DOOR_Y + 1.10 * _uy * -1)   # lado A del vano
SALIDA = (DOOR_X + 1.30 * _ux, DOOR_Y + 1.30 * _uy)            # lado B del vano

RUTA = [APROX, (DOOR_X, DOOR_Y), SALIDA,          # ida
        (DOOR_X, DOOR_Y), APROX, (AX, AY)]        # vuelta
TOL = 0.40

_O = open("/ws/travesia_result.txt", "w")
def log(*a):
    s = " ".join(str(x) for x in a); print(s, flush=True); _O.write(s + "\n"); _O.flush()

log("ruta: A(%.2f,%.2f) -> aprox(%.2f,%.2f) -> vano(%.2f,%.2f) -> salida(%.2f,%.2f) -> y vuelta"
    % (AX, AY, APROX[0], APROX[1], DOOR_X, DOOR_Y, SALIDA[0], SALIDA[1]))

sim = SimulationContext(sim_utils.SimulationCfg(dt=1/200, device=DEV))
stage = omni.usd.get_context().get_stage()

sim_utils.GroundPlaneCfg().func("/World/Suelo", sim_utils.GroundPlaneCfg())
# La oficina trae su propio domo a 320. Un domo extra a 2500 lavaba la escena:
# aqui solo se rellenan sombras.
sim_utils.DomeLightCfg(intensity=400.0).func("/World/Luz", sim_utils.DomeLightCfg(intensity=400.0))

OFI = os.environ.get("OFI", "1") == "1"   # OFI=0 -> misma ruta sobre suelo plano
if OFI:
    _ofi = stage.DefinePrim("/Oficina", "Xform")
    _ofi.GetReferences().AddReference(assetPath="/ws/office3d.usd", primPath="/World")
    n = 0
    for grupo in ("/Oficina/Estructura", "/Oficina/Cristal", "/Oficina/Moqueta"):
        pr = stage.GetPrimAtPath(grupo)
        if pr and pr.IsValid():
            for c in pr.GetChildren():
                if not c.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI.Apply(c); n += 1
    # El G1 decorativo del USD conserva sus ~40 joints aunque se le quitaron los
    # cuerpos rigidos: PhysX intenta crearlos, falla en cada uno
    # ("CreateJoint - no bodies defined at body0 and body1") y la escena de
    # fisica queda rota -> el robot bueno se congela con velocidad no nula y
    # posicion inmovil. MakeInvisible() lo oculta pero NO lo saca de la fisica.
    # Hay que desactivar el prim para que salga de la composicion.
    g1v = stage.GetPrimAtPath("/Oficina/G1")
    if g1v and g1v.IsValid():
        g1v.SetActive(False)
        log("G1 decorativo desactivado (sus joints huerfanos congelaban la fisica)")
    log("oficina cargada, colision en %d prims" % n)
else:
    log("SIN oficina: misma ruta sobre suelo plano")

cfg = G1_MINIMAL_CFG.replace(prim_path="/World/Robot")
cfg.init_state.pos = (AX, AY, 0.74)
cfg.init_state.rot = (math.cos(AYAW / 2), 0.0, 0.0, math.sin(AYAW / 2))
robot = Articulation(cfg)

ann = None
if REC:
    from isaacsim.core.utils.extensions import enable_extension
    enable_extension("omni.replicator.core")
    sim_app.update()
    import omni.replicator.core as rep
    os.makedirs(DIR, exist_ok=True)
    cam = UsdGeom.Camera.Define(stage, "/World/CamG1")
    cam.CreateFocalLengthAttr(20.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 200.0))
    xc = UsdGeom.Xformable(cam.GetPrim()); xc.ClearXformOpOrder()
    OT = xc.AddTranslateOp(); OR_ = xc.AddRotateXYZOp()
    rp = rep.create.render_product("/World/CamG1", (960, 540))
    ann = rep.AnnotatorRegistry.get_annotator("rgb"); ann.attach([rp])

sim.reset()
politica = torch.jit.load(CKPT + "/exported/policy.pt").to(DEV).eval()
log("politica: %s" % CKPT.split("/")[-1])

dev = robot.device
accion_prev = torch.zeros((1, robot.num_joints), device=dev)
cmd = torch.zeros((1, 3), device=dev)

def obs():
    d = robot.data
    return torch.cat([d.root_lin_vel_b, d.root_ang_vel_b, d.projected_gravity_b, cmd,
                      d.joint_pos - d.default_joint_pos, d.joint_vel, accion_prev], dim=-1)

def rumbo():
    q = robot.data.root_quat_w[0]
    w, x, y, z = (float(v) for v in q)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

pasos = int(SEGS * 200)
idx = 0; nrec = 0; caidas = 0; t0 = time.time()
cam_th = AYAW
for k in range(pasos):
    p = robot.data.root_pos_w[0]
    px, py, pz = float(p[0]), float(p[1]), float(p[2])

    if idx < len(RUTA):
        gx, gy = RUTA[idx]
        d = math.hypot(gx - px, gy - py)
        if d < TOL:
            log("t=%5.1fs  punto %d/%d alcanzado  (%.2f, %.2f)" % (k / 200.0, idx + 1, len(RUTA), px, py))
            idx += 1
        else:
            th = rumbo()
            err = (math.atan2(gy - py, gx - px) - th + math.pi) % (2 * math.pi) - math.pi
            # La politica se entreno con heading_command=true y rel_heading_envs=1.0:
            # NUNCA vio una velocidad de giro arbitraria, solo 0.5*error_de_rumbo
            # acotada a [-1,1] (heading_control_stiffness). Y lin_vel_x se muestreo
            # en (0,1], nunca 0. Con ganancia 1.6 y vx=0 el robot se quedaba
            # clavado: fuera de distribucion por los dos lados. Aqui se reproduce
            # exactamente el convenio de entrenamiento.
            wz = max(-1.0, min(1.0, 0.5 * err))
            vx = 0.70 if abs(err) < 0.35 else (0.45 if abs(err) < 0.90 else 0.30)
            cmd[0, 0] = vx; cmd[0, 1] = 0.0; cmd[0, 2] = wz
    else:
        cmd[0, 0] = 0.0; cmd[0, 2] = 0.0

    if k % 4 == 0:
        with torch.no_grad():
            accion = politica(obs())
        accion_prev = accion.clone()
        robot.set_joint_position_target(robot.data.default_joint_pos + accion * 0.5)
    robot.write_data_to_sim()
    sim.step(render=(k % 8 == 0))
    robot.update(1 / 200)

    if k % 8 == 0 and pz < 0.45:
        caidas += 1
    if k % 400 == 0:                                   # traza cada 2 s simulados
        gx_, gy_ = RUTA[idx] if idx < len(RUTA) else (px, py)
        _jv = float(torch.abs(robot.data.joint_vel).max())
        _ac = float(torch.abs(accion_prev).max())
        _lv = float(torch.norm(robot.data.root_lin_vel_b))
        log("t=%5.1f pose=(%6.2f,%6.2f,%5.2f) rumbo=%7.1f -> punto %d d=%5.2f cmd=(%.2f,%.2f) |jvel|=%.3f |acc|=%.3f |v|=%.3f"
            % (k / 200.0, px, py, pz, math.degrees(rumbo()), idx,
               math.hypot(gx_ - px, gy_ - py), float(cmd[0, 0]), float(cmd[0, 2]), _jv, _ac, _lv))
    if REC and k % 8 == 0:
        # camara persecutora: sigue el RUMBO del robot, no su velocidad, que da
        # tirones. Suavizada para que el giro no maree.
        th = rumbo()
        dth = (th - cam_th + math.pi) % (2 * math.pi) - math.pi
        # ganancia ADAPTATIVA: 0.06 en recta (suave), hasta 0.30 en giros. Con ganancia fija
        # la camara se quedaba mirando la pared durante el giro de 180 en B (medido: ~3 s de
        # fotogramas contra la particion en el video v19a).
        cam_th += (0.06 + min(0.24, 1.8 * abs(dth))) * dth
        # Los muros de la oficina miden hasta 2.7 m: a 1.75 m la camara se metia
        # DENTRO de ellos en los tramos estrechos y salian fotogramas planos
        # (10 de 67 muestreados). Por encima de 2.7 no ocluye nunca.
        # en giro cerrado la orbita a radio fijo barre A TRAVES de las paredes (medido
        # en el giro de B): mejor acercarse al robot mientras dura el giro.
        D = CAM_D * (1.0 - 0.55 * min(1.0, abs(dth) / 0.5))
        H = CAM_H
        cx, cy = px - D * math.cos(cam_th), py - D * math.sin(cam_th)
        OT.Set(Gf.Vec3d(cx, cy, H))
        OR_.Set(Gf.Vec3f(90.0 + math.degrees(math.atan2(pz + 0.45 - H, D)), 0.0,
                         math.degrees(cam_th) - 90.0))
        d_ = ann.get_data()
        if d_ is not None and len(d_):
            Image.fromarray(np.asarray(d_)[:, :, :3].astype("uint8")).save(
                os.path.join(DIR, "t%05d.png" % nrec)); nrec += 1
    if idx >= len(RUTA):
        log("RUTA COMPLETA en %.1fs simulados" % (k / 200.0)); break

p = robot.data.root_pos_w[0]
log("FIN | puntos %d/%d | pose (%.2f, %.2f, %.2f) | caido %d | fotogramas %d | reloj %.0fs"
    % (idx, len(RUTA), float(p[0]), float(p[1]), float(p[2]), caidas, nrec, time.time() - t0))
sim_app.close()
