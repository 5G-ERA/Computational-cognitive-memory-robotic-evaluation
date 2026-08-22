"""Test AISLADO del mecanismo de tiras: un panel solo frente al lidar, nada mas.

Panel a 1.5 m, tiras verticales de 3.33 cm a lo largo de y, patron 3 visibles / 5 invisibles
(MakeInvisible). Prediccion geometrica: ausencia por sector = (16.7 - 5.2) / 26.7 = 43%.
Si la medida se aleja mucho, el mecanismo de invisibilidad no funciona para el sensor.
"""
import json, math

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": 640, "height": 360})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.stage import create_new_stage
from isaacsim.sensors.rtx import LidarRtx
from pxr import UsdGeom, UsdShade, Sdf, Gf, UsdLux
import omni.usd

create_new_stage()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.Xform.Define(stage, "/World")

ANCHO = 0.0333
n = 0
for i in range(-60, 61):                       # panel de 4 m de largo en y
    yc = i * ANCHO
    invisible = (i % 8) >= 3
    cb = UsdGeom.Cube.Define(stage, "/World/t%d" % n); n += 1
    cb.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(cb.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(1.5, yc, 1.1))
    xf.AddScaleOp().Set(Gf.Vec3f(0.05, ANCHO, 2.2))
    if invisible:
        UsdGeom.Imageable(cb.GetPrim()).MakeInvisible()
print("tiras:", n, flush=True)

world = World(stage_units_in_meters=1.0)
lidar = LidarRtx(prim_path="/World/L", name="L", position=np.array([0.0, 0.0, 0.55]),
                 config_file_name="Example_Rotary_2D")
lidar.attach_annotator("IsaacComputeRTXLidarFlatScan")
world.reset()
ann = list(lidar.get_annotators().values())[0]

lect = []
for i in range(400):
    world.step(render=True)
    if i % 10 == 9:
        d = ann.get_data()
        rg = np.asarray(d.get("linearDepthData", [])).ravel()
        if rg.size >= 900:
            azr = d.get("azimuthRange")
            az = np.linspace(float(azr[0]), float(azr[1]), rg.size, endpoint=False)
            lect.append((az, rg))
            if len(lect) >= 5:
                break
agg = {}
for az, rg in lect:
    for b, r in zip(az % 360, rg):
        if r < 0.05 or not np.isfinite(r):
            continue
        k = int(b // 2) * 2
        if k not in agg or r < agg[k]:
            agg[k] = float(r)

# ventana del panel: de -53 a +53 grados (4m de panel a 1.5m)... restringimos a +-40
total = con = 0
for b in list(range(0, 42, 2)) + list(range(320, 360, 2)):
    total += 1
    r = agg.get(b)
    if r is not None and r <= 2.5:
        con += 1
print("VENTANA DEL PANEL: %d sectores | con retorno %d | SIN retorno %d = %.0f%%" % (
    total, con, total - con, 100 * (total - con) / total), flush=True)
print("prediccion geometrica del patron: ~43%%", flush=True)
print("=== TEST AISLADO OK ===", flush=True)
app.close()
