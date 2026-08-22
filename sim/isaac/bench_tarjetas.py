"""Banco de fidelidad: renderiza cada tarjeta DESDE LA MISMA POSE que la vio el robot real.

Camara sim con los intrinsecos reales (fx=600*W/640, centro del fotograma, altura 1.10 m,
cabeceo -10) colocada en el punto de observacion registrado. Si el modelo es fiel, YOLO debe
devolver la MISMA etiqueta con confianza parecida a la real.
"""
import json, math, os

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": 320, "height": 180})

import numpy as np
import omni.usd
import omni.replicator.core as rep
from isaacsim.core.utils.stage import open_stage
from pxr import UsdGeom, Gf
from PIL import Image

os.makedirs("/ws/bench_tarjetas", exist_ok=True)
open_stage("/ws/office3d.usd")
stage = omni.usd.get_context().get_stage()

CAM = "/World/CamBench"
c = UsdGeom.Camera.Define(stage, CAM)
# FOV horizontal real: 2*atan(160/300) = 56.1 grados -> con apertura 20.955mm, focal:
c.CreateFocalLengthAttr(18.9)          # 2*atan(aperture/2/f) = 56.1 con aperture 20.955
c.CreateHorizontalApertureAttr(20.955)
c.CreateClippingRangeAttr(Gf.Vec2f(0.05, 60.0))
xf = UsdGeom.Xformable(c.GetPrim()); xf.ClearXformOpOrder()
OT = xf.AddTranslateOp(); OR_ = xf.AddRotateXYZOp()

rp = rep.create.render_product(CAM, (320, 180))
ann = rep.AnnotatorRegistry.get_annotator("rgb")
ann.attach([rp])

fichas = json.load(open("/ws/recortes/fichas.json"))
salida = []
for f_ in fichas:
    if f_["ancho_m"] < 0.30 or f_["alto_m"] < 0.30:
        continue
    ox, oy = f_["observador"]
    tx, ty = f_.get("pos_obs") or f_["pos"]
    # POSE EXACTA del robot en ese instante (no "mirar a la tarjeta"): asi el objeto cae en el
    # mismo lugar del encuadre que en el fotograma real, con su mismo contexto.
    yaw = f_.get("obs_yaw")
    if yaw is None:
        yaw = math.degrees(math.atan2(ty - oy, tx - ox))
    OT.Set(Gf.Vec3d(ox, oy, 1.10))
    OR_.Set(Gf.Vec3f(90.0 - 10.0, 0.0, float(yaw) - 90.0))
    for _ in range(12):
        rep.orchestrator.step(rt_subframes=2)
    d = ann.get_data()
    img = np.asarray(d)[:, :, :3].astype("uint8")
    nom = "%s_%.2f_%.2f.png" % (f_["lab"], tx, ty)
    Image.fromarray(img).save("/ws/bench_tarjetas/" + nom)
    salida.append({"png": nom, "lab": f_["lab"], "conf_real": f_["conf"],
                   "rango": f_["rango"], "pos": f_["pos"], "frame": f_["frame"]})
    print("render:", nom, flush=True)
json.dump(salida, open("/ws/bench_tarjetas/lista.json", "w"), indent=1)
print("=== BENCH TARJETAS RENDER OK ===", flush=True)
app.close()
