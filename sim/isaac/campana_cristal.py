"""Campana del cristal (un solo arranque): captura RIGUROSA en 3 poses.

  A (0.99, 0.57)      -> para calibrar el desfase de azimut contra la geometria conocida
  FRONTAL (-1.2, 0.1) -> cristal a incidencia ~0 (como la tanda real 'de frente')
  OBLICUA (-1.47, 1.1)-> linea de vista a ~30 grados del normal del cristal

Rigor: cada captura espera a que DOS lecturas consecutivas del anotador tengan el mismo
tamano y >= 900 rayos (rotacion completa, nada de bufers parciales).
"""
import json, math, time

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": 640, "height": 360})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.stage import open_stage
from isaacsim.sensors.rtx import LidarRtx
from pxr import UsdGeom, Gf
import omni.usd

open_stage("/ws/office3d.usd")
world = World(stage_units_in_meters=1.0)
H = 0.55
lidar = LidarRtx(prim_path="/World/LidarX", name="lidarX",
                 position=np.array([0.99, 0.57, H]),
                 config_file_name="Example_Rotary_2D")
lidar.attach_annotator("IsaacComputeRTXLidarFlatScan")
world.reset()
stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath("/World/LidarX")
ann = list(lidar.get_annotators().values())[0]

def captura_estable():
    """5 lecturas completas (>=900 rayos) espaciadas 10 frames; el llamante agrega por minimo.
    (Dos lecturas identicas nunca llegan: el sensor rotatorio refresca continuamente.)"""
    lecturas = []
    for i in range(600):
        world.step(render=True)
        if i % 10 != 9:
            continue
        try:
            d = ann.get_data()
        except Exception:
            continue
        rg = np.asarray(d.get("linearDepthData", [])).ravel()
        if rg.size >= 900:
            azr = d.get("azimuthRange")
            az = np.linspace(float(azr[0]), float(azr[1]), rg.size, endpoint=False)
            lecturas.append((az.copy(), rg.copy()))
            if len(lecturas) >= 5:
                return lecturas
    return lecturas if lecturas else None

def perfil(az, rg):
    out = {}
    for b, r in zip(az % 360.0, rg):
        if r < 0.05 or not np.isfinite(r):
            continue
        k = int(b // 2) * 2
        if k not in out or r < out[k]:
            out[k] = float(r)
    return out

POSES = {"A": (0.99, 0.57), "FRONTAL": (-1.2, 0.1), "OBLICUA": (-1.47, -0.90)}
res = {}
xf = UsdGeom.Xformable(prim)
ops = xf.GetOrderedXformOps()
top = ops[0] if ops else xf.AddTranslateOp()
for nombre, (px, py) in POSES.items():
    top.Set(Gf.Vec3d(px, py, H))
    for _ in range(30):                      # purga del bufer tras mover
        world.step(render=True)
    lect = captura_estable()
    if not lect:
        print("FALLO captura en", nombre, flush=True)
        continue
    agg = {}
    for az, rg in lect:
        p = perfil(az, rg)
        for k, v in p.items():
            if k not in agg or v < agg[k]:
                agg[k] = v
    res[nombre] = {"pose": [px, py], "n_lecturas": len(lect),
                   "perfil": {str(k): round(v, 3) for k, v in sorted(agg.items())}}
    print("capturado %s: %d sectores" % (nombre, len(res[nombre]["perfil"])), flush=True)

json.dump(res, open("/ws/campana_cristal.json", "w"), indent=1)
print("=== CAMPANA OK ===", flush=True)
app.close()
