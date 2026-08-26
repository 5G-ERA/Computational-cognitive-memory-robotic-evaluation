"""P3 peldaño 1: lidar RTX en el waypoint A de la oficina reconstruida.

Escaneo desde la pose real A (0.99, 0.57) a la altura del lidar del G1, y volcado a
(rumbo, rango) como los laser_snapshots reales — para comparar perfil contra perfil.
"""
import json, math

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": 640, "height": 360})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.stage import open_stage
from isaacsim.sensors.rtx import LidarRtx
import omni.usd

open_stage("/ws/office3d.usd")
world = World(stage_units_in_meters=1.0)

H_LIDAR = 0.55                      # altura aprox del utlidar del G1 (torso)
lidar = LidarRtx(prim_path="/World/LidarA", name="lidarA",
                 position=np.array([0.99, 0.57, H_LIDAR]),
                 config_file_name="Example_Rotary_2D")
lidar.attach_annotator("IsaacComputeRTXLidarFlatScan")
lidar.attach_annotator("IsaacCreateRTXLidarScanBuffer")    # respaldo: nube de puntos
world.reset()

anns = lidar.get_annotators()
flat, nube = {}, None
for i in range(300):                       # espera ACTIVA hasta que el sensor produzca
    world.step(render=True)
    if i % 20 != 19:
        continue
    for nombre, ann in anns.items():
        try:
            d = ann.get_data()
        except Exception:
            continue
        if not hasattr(d, "keys"):
            continue
        ld = np.asarray(d.get("linearDepthData", [])).ravel()
        if ld.size:
            flat = d
        dd = d.get("data", None)
        if dd is not None and np.asarray(dd).size:
            nube = np.asarray(dd)
    if flat or nube is not None:
        print("datos a los %d pasos" % (i+1), flush=True)
        break
print("flat keys:", list(flat.keys()) if flat else "vacio",
      " nube:", None if nube is None else nube.shape, flush=True)
if not flat and nube is not None and nube.size:
    # aplanar la nube: banda horizontal
    rel = nube.reshape(-1, 3)
    d2 = np.hypot(rel[:, 0], rel[:, 1])
    band = np.abs(rel[:, 2]) < 0.12
    rel, d2 = rel[band], d2[band]
    flat = {"linearDepthData": d2,
            "azimuthRange": None,
            "_brg": (np.degrees(np.arctan2(rel[:, 1], rel[:, 0]))) % 360.0}
# claves reales del FlatScan 5.1: linearDepthData + azimuthRange/horizontalResolution
rg = np.asarray(flat.get("linearDepthData", [])).ravel()
azr = flat.get("azimuthRange", None)
if "_brg" in flat:
    az = flat["_brg"]
elif rg.size and azr is not None:
    az = np.linspace(float(azr[0]), float(azr[1]), rg.size, endpoint=False)
else:
    az = np.asarray([])
print("flatscan: %d azimuts, %d rangos" % (len(az), len(rg)), flush=True)
salida = {}
if len(az) and len(rg):
    brg = az % 360.0
    d2 = rg
    # por sector de 2 grados: rango minimo (primer retorno), como el snapshot real
    perfil = {}
    for b, r in zip(brg, d2):
        if r < 0.05 or not np.isfinite(r):
            continue
        k = int(b // 2) * 2
        if k not in perfil or r < perfil[k]:
            perfil[k] = float(r)
    salida = {"h_lidar": H_LIDAR, "pose": [0.99, 0.57], "sector_deg": 2,
              "perfil": {str(k): round(v, 3) for k, v in sorted(perfil.items())}}
    json.dump(salida, open("/ws/sim_scan_A.json", "w"), indent=1)
    print("sectores con retorno:", len(perfil), "de 180", flush=True)
    print("=== SCAN SIM OK ===", flush=True)
else:
    print("SIN nube del lidar", flush=True)
app.close()
