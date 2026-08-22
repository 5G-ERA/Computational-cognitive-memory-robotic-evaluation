"""BANCO de calibracion del cristal: panel recto, linea limpia, barrido de patrones.

Reproduce la escenificacion de la tanda real del 21-ago (nada entre el sensor y el especimen)
y busca el patron de tiras cuya firma case con las DOS medidas reales:
       incidencia  0 grados -> 44% de rumbos SIN retorno
       incidencia 30 grados -> 32%
Un solo arranque de Isaac: el panel se queda fijo y entre capturas solo se conmuta la
VISIBILIDAD de cada tira (el mecanismo probado en aislado).
"""
import json, math

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": 640, "height": 360})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.stage import create_new_stage
from isaacsim.sensors.rtx import LidarRtx
from pxr import UsdGeom, Gf
import omni.usd

ANCHO = 0.0333          # ancho de tira (m)
DIST = 1.45             # distancia real de la tanda frontal
LARGO = 60              # tiras a cada lado del centro
H = 0.55

create_new_stage()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.Xform.Define(stage, "/World")

# panel RECTO en x=0, a lo largo de y
tiras = []
for i in range(-LARGO, LARGO + 1):
    cb = UsdGeom.Cube.Define(stage, "/World/t%d" % (i + LARGO))
    cb.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(cb.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(0.0, i * ANCHO, 1.1))
    xf.AddScaleOp().Set(Gf.Vec3f(0.04, ANCHO, 2.2))
    tiras.append((i, cb.GetPrim()))
print("tiras del panel:", len(tiras), flush=True)

world = World(stage_units_in_meters=1.0)
lidar = LidarRtx(prim_path="/World/L", name="L", position=np.array([DIST, 0.0, H]),
                 config_file_name="Example_Rotary_2D")
lidar.attach_annotator("IsaacComputeRTXLidarFlatScan")
world.reset()
ann = list(lidar.get_annotators().values())[0]
prim_l = stage.GetPrimAtPath("/World/L")
op_t = UsdGeom.Xformable(prim_l).GetOrderedXformOps()[0]

def aplica(vis, inv):
    per = vis + inv
    for i, p in tiras:
        invisible = (i % per) >= vis
        im = UsdGeom.Imageable(p)
        if invisible:
            im.MakeInvisible()
        else:
            im.MakeVisible()

def captura():
    lect = []
    for i in range(400):
        world.step(render=True)
        if i % 10 != 9:
            continue
        d = ann.get_data()
        rg = np.asarray(d.get("linearDepthData", [])).ravel()
        if rg.size >= 900:
            azr = d.get("azimuthRange")
            az = np.linspace(float(azr[0]), float(azr[1]), rg.size, endpoint=False)
            lect.append((az, rg))
            if len(lect) >= 4:
                break
    agg = {}
    for az, rg in lect:
        for b, r in zip(az % 360, rg):
            if r < 0.05 or not np.isfinite(r):
                continue
            k = int(b // 2) * 2
            if k not in agg or r < agg[k]:
                agg[k] = float(r)
    return agg

def firma(px, py, agg):
    """% de rumbos SIN retorno en la ventana que apunta al panel (+-25 grados del normal)."""
    total = con = 0
    for off in range(-24, 26, 2):
        # rumbo desde el sensor hacia un punto del panel a esa desviacion
        b = (math.degrees(math.atan2(-py, -px)) + off) % 360
        k = int(b // 2) * 2
        total += 1
        r = agg.get(k)
        if r is not None and r <= DIST * 1.6:
            con += 1
    return 100.0 * (total - con) / total

POSES = {0: (DIST, 0.0), 30: (DIST * math.cos(math.radians(30)), DIST * math.sin(math.radians(30)))}
PATRONES = [(3, 3), (3, 5), (2, 4), (4, 4), (5, 3), (2, 6), (4, 6), (3, 4), (5, 5), (2, 3)]

print("\n%-10s %10s %10s   %s" % ("patron", "0 grados", "30 grados", "objetivo 44 / 32"))
res = []
for vis, inv in PATRONES:
    aplica(vis, inv)
    f = {}
    for ang, (px, py) in POSES.items():
        op_t.Set(Gf.Vec3d(px, py, H))
        for _ in range(20):
            world.step(render=True)
        f[ang] = firma(px, py, captura())
    err = abs(f[0] - 44) + abs(f[30] - 32)
    res.append((err, vis, inv, f[0], f[30]))
    print("%-10s %9.0f%% %9.0f%%   err %.0f" % ("%dv/%di" % (vis, inv), f[0], f[30], err), flush=True)

res.sort()
print("\n=== MEJOR PATRON: %dv/%di -> %.0f%% / %.0f%% (objetivo 44/32) ===" % (
    res[0][1], res[0][2], res[0][3], res[0][4]), flush=True)
json.dump({"vis": res[0][1], "inv": res[0][2], "firma_0": res[0][3], "firma_30": res[0][4],
           "ancho_tira": ANCHO},
          open("/ws/patron_cristal.json", "w"), indent=1)
print("=== CALIBRACION OK ===", flush=True)
app.close()
