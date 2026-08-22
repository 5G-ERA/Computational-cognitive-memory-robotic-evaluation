"""Cristal v2: ¿respeta el lidar RTX el material OmniGlass (MDL con transmision)?"""
import json, math

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": 640, "height": 360})

import numpy as np
import omni.usd, omni.kit.commands
from isaacsim.core.api import World
from isaacsim.core.utils.stage import open_stage
from isaacsim.sensors.rtx import LidarRtx
from pxr import UsdShade, Sdf

open_stage("/ws/office3d.usd")
stage = omni.usd.get_context().get_stage()

# material OmniGlass y re-vinculo de los prims de cristal
n = 0
est = stage.GetPrimAtPath("/World/Estructura")
for prim in est.GetChildren():
    rel = UsdShade.MaterialBindingAPI(prim).GetDirectBinding().GetMaterialPath()
    if rel and "Cristal" in str(rel):
        attr = prim.CreateAttribute("primvars:doNotCastRays", Sdf.ValueTypeNames.Bool)
        attr.Set(True)
        n += 1
print("prims de cristal con doNotCastRays:", n, flush=True)

world = World(stage_units_in_meters=1.0)
lidar = LidarRtx(prim_path="/World/LidarC", name="lidarC",
                 position=np.array([-1.2, 0.1, 0.55]),
                 config_file_name="Example_Rotary_2D")
lidar.attach_annotator("IsaacComputeRTXLidarFlatScan")
world.reset()
anns = lidar.get_annotators()
flat = {}
for i in range(300):
    world.step(render=True)
    if i % 20 == 19:
        for nombre, ann in anns.items():
            try:
                d = ann.get_data()
            except Exception:
                continue
            if hasattr(d, "keys") and np.asarray(d.get("linearDepthData", [])).ravel().size:
                flat = d
        if flat:
            break
rg = np.asarray(flat.get("linearDepthData", [])).ravel()
azr = flat.get("azimuthRange")
az = np.linspace(float(azr[0]), float(azr[1]), rg.size, endpoint=False) if rg.size else np.array([])
perfil = {}
for b, r in zip(az % 360.0, rg):
    if r < 0.05 or not np.isfinite(r):
        continue
    k = int(b // 2) * 2
    if k not in perfil or r < perfil[k]:
        perfil[k] = float(r)
json.dump({"perfil": {str(k): round(v, 3) for k, v in sorted(perfil.items())}},
          open("/ws/sim_scan_NORAYS.json", "w"))
print("sectores con retorno:", len(perfil), flush=True)
print("=== NORAYS SCAN OK ===", flush=True)
app.close()
