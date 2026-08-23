"""Renders fijos de la oficina v19: interior desde A, la puerta, la sala de Renisa
y un cenital bajo techo. Un JPG por vista en /ws/v19_*.jpg"""
import math
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": 1280, "height": 720})

import numpy as np
import omni.usd
import omni.replicator.core as rep
from isaacsim.core.utils.stage import open_stage
from pxr import UsdGeom, Gf
from PIL import Image

open_stage("/ws/office3d.usd")
stage = omni.usd.get_context().get_stage()
c = UsdGeom.Camera.Define(stage, "/World/CamShot")
c.CreateFocalLengthAttr(16.0)
c.CreateClippingRangeAttr(Gf.Vec2f(0.05, 300.0))
xf = UsdGeom.Xformable(c.GetPrim()); xf.ClearXformOpOrder()
OT = xf.AddTranslateOp(); OR_ = xf.AddRotateXYZOp()
rp = rep.create.render_product("/World/CamShot", (1280, 720))
ann = rep.AnnotatorRegistry.get_annotator("rgb"); ann.attach([rp])

def mira(cx, cy, cz, tx, ty, tz):
    OT.Set(Gf.Vec3d(cx, cy, cz))
    dx, dy, dz = tx-cx, ty-cy, tz-cz
    yaw = math.degrees(math.atan2(dy, dx)) - 90.0
    pitch = 90.0 + math.degrees(math.atan2(dz, math.hypot(dx, dy)))
    OR_.Set(Gf.Vec3f(pitch, 0.0, yaw))

VISTAS = [
    ("v19_desdeA",   (0.99, 0.57, 1.45),  (-3.9, 1.25, 1.0)),   # desde A hacia la puerta
    ("v19_puerta",   (-2.2, 0.2, 1.5),    (-4.8, 2.2, 1.0)),    # el vano de cerca
    ("v19_renisa",   (-4.6, 2.4, 1.5),    (-6.5, 1.0, 0.8)),    # dentro de la oficina de Renisa
    ("v19_esquina",  (3.8, -1.5, 1.6),    (4.6, 0.2, 1.2)),     # la caja del Summit
    ("v19_general",  (-1.5, -2.6, 2.45),  (-1.0, 2.5, 0.6)),    # general bajo techo
]
for i in range(24):
    app.update()
for nom, (cx, cy, cz), (tx, ty, tz) in VISTAS:
    mira(cx, cy, cz, tx, ty, tz)
    for i in range(12):
        app.update()
    d = ann.get_data()
    if d is not None and len(d):
        Image.fromarray(np.asarray(d)[:, :, :3].astype("uint8")).save("/ws/%s.jpg" % nom, quality=90)
        print(nom, "ok")
app.close()
