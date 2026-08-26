"""P1: el G1 dentro de Isaac — carga del activo, articulacion y fisica (headless, sin render).

Compatible 4.x (omni.isaac.*) y 5.x (isaacsim.*). Busca el G1 en los assets cloud
(Robots/Unitree*); carga, cuenta juntas, 2 s de fisica y reporta si la gravedad actua.
"""
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

_OUT = open("/ws/p1_result.txt", "w")
def log(*a):
    s = " ".join(str(x) for x in a)
    print(s, flush=True)
    _OUT.write(s + "\n"); _OUT.flush()

import omni.client
import omni.usd

try:
    from isaacsim.storage.native import get_assets_root_path        # 5.x
except ImportError:
    from omni.isaac.nucleus import get_assets_root_path             # 4.x

try:                                                                # 5.x
    from isaacsim.core.api import World
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.prims import SingleArticulation as Articulation
except ImportError:                                                 # 4.x
    from omni.isaac.core import World
    from omni.isaac.core.utils.stage import add_reference_to_stage
    from omni.isaac.core.articulations import Articulation

root = get_assets_root_path()
log("ASSETS_ROOT:", root)

candidatos = []
for carpeta in ("/Isaac/Robots/Unitree", "/Isaac/Robots/UnitreeRobotics", "/Isaac/Robots/Unitree Robotics"):
    res, entries = omni.client.list(root + carpeta)
    if res == omni.client.Result.OK:
        log("carpeta %s:" % carpeta, [e.relative_path for e in entries])
        for e in entries:
            if "g1" in e.relative_path.lower():
                sub = root + carpeta + "/" + e.relative_path
                r2, e2 = omni.client.list(sub)
                if r2 == omni.client.Result.OK:
                    for f in e2:
                        if f.relative_path.lower().endswith(".usd"):
                            candidatos.append(sub + "/" + f.relative_path)
                if e.relative_path.lower().endswith(".usd"):
                    candidatos.append(sub)
log("candidatos G1:", candidatos)

if candidatos:
    usd = sorted(candidatos, key=len)[0]
    log("cargando:", usd)
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    add_reference_to_stage(usd_path=usd, prim_path="/World/G1")
    from pxr import UsdGeom, Gf
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath("/World/G1")
    xf = UsdGeom.Xformable(prim)
    try:
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, 1.05))
    except Exception as e:
        log("(xform:", e, ")")
    robot = Articulation("/World/G1")
    world.reset()
    robot.initialize()
    log("JUNTAS:", robot.num_dof)
    try:
        log("nombres:", list(robot.dof_names)[:12], "...")
    except Exception:
        pass
    p0 = robot.get_world_pose()[0]
    for _ in range(120):
        world.step(render=False)
    p1 = robot.get_world_pose()[0]
    log("pose z inicial %.3f -> final %.3f (fisica actua: %s)" % (
        float(p0[2]), float(p1[2]), "SI" if abs(float(p1[2]) - float(p0[2])) > 0.02 else "no"))
    log("=== P1 SANITY OK ===")
else:
    log("=== SIN ACTIVO G1 EN CLOUD: fallback MJCF /ws/g1_mjcf ===")
app.close()
