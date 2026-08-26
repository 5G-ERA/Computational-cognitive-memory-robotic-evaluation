"""Isaac: SOLO renderiza y deja el ultimo fotograma en disco. Nada de HTTP aqui.

Por que: servir HTTP desde un hilo del mismo proceso competia por el GIL con el bucle de
render de Isaac, y la imagen llegaba a rafagas (sintoma: 'se atasca'). El servidor web es
ahora un proceso aparte que solo lee ficheros.
    escribe:  /ws/live.jpg      (ultimo fotograma, escritura atomica)
    lee:      /ws/cam.json      (posicion de camara que pide la web)
"""
import io, json, math, os, time

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": 960, "height": 540})

import numpy as np
import omni.usd
import omni.replicator.core as rep
from isaacsim.core.utils.stage import open_stage
from pxr import UsdGeom, Gf
from PIL import Image

open_stage("/ws/office3d.usd")
# SIN World y SIN fisica: escena de inspeccion. La v1 creaba World y el G1 (con articulacion
# activa y sin controlador) se desplomaba como un muñeco. Aqui solo se renderiza.
stage = omni.usd.get_context().get_stage()

CAM = "/World/CamWeb"
c = UsdGeom.Camera.Define(stage, CAM)
c.CreateFocalLengthAttr(18.0)
c.CreateClippingRangeAttr(Gf.Vec2f(0.05, 300.0))
xf = UsdGeom.Xformable(c.GetPrim()); xf.ClearXformOpOrder()
OP_T = xf.AddTranslateOp(); OP_R = xf.AddRotateXYZOp()

rp = rep.create.render_product(CAM, (960, 540))
ann = rep.AnnotatorRegistry.get_annotator("rgb")
ann.attach([rp])

DEF = {"x": 3.0, "y": -3.0, "z": 3.0, "tx": -3.5, "ty": 2.0, "tz": 0.9}
print("=== RENDER LISTO ===", flush=True)

n = 0
while True:
    try:
        e = json.load(open("/ws/cam.json"))
    except Exception:
        e = DEF
    dx, dy, dz = e["tx"]-e["x"], e["ty"]-e["y"], e["tz"]-e["z"]
    OP_T.Set(Gf.Vec3d(e["x"], e["y"], e["z"]))
    OP_R.Set(Gf.Vec3f(90.0 + math.degrees(math.atan2(dz, math.hypot(dx, dy))),
                      0.0, math.degrees(math.atan2(dy, dx)) - 90.0))
    rep.orchestrator.step(rt_subframes=1)
    d = ann.get_data()
    if d is not None and len(d):
        Image.fromarray(np.asarray(d)[:, :, :3].astype("uint8")).save("/ws/.tmp.jpg", "JPEG", quality=68)
        os.replace("/ws/.tmp.jpg", "/ws/live.jpg")     # atomico: la web nunca lee a medias
        n += 1
    time.sleep(0.06)                                    # cede CPU; ~12 fps basta
