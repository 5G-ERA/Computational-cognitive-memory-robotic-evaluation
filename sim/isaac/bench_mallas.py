"""Banco malla->detector: ¿que ve NUESTRO YOLO11x en cada malla candidata?

Escena neutra (moqueta y pared con los colores reales medidos), cada malla sola, camara a la
altura de la del G1 (1.15 m) a 4 distancias. Tambien un BLOQUE GRIS de control: es lo que el
gemelo ensena hoy al detector. Los JPEG salen a /ws/bench/ para evaluarlos con el mismo
modelo que el servidor de percepcion (yolo11x).
"""
import json, math, os

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": 960, "height": 540})

import numpy as np
import omni.usd
import omni.replicator.core as rep
from isaacsim.core.utils.stage import add_reference_to_stage, create_new_stage
from isaacsim.storage.native import get_assets_root_path
from pxr import Usd, UsdGeom, UsdShade, UsdPhysics, Sdf, Gf, UsdLux
from PIL import Image

os.makedirs("/ws/bench", exist_ok=True)
root = get_assets_root_path(); PR = root + "/Isaac/Environments/Office/Props/"
col = json.load(open("/ws/colores_reales.json"))

CANDIDATAS = {
    "SM_ChairOffice_A": PR + "SM_ChairOffice_A.usd",
    "SM_Chair_01a":     PR + "SM_Chair_01a.usd",
    "SM_Armchair":      PR + "SM_Armchair.usd",
    "SM_BoxBigA":       PR + "SM_BoxBigA.usd",
    "SM_BoxA":          PR + "SM_BoxA.usd",
    "BLOQUE_GRIS":      None,                    # control: lo que el gemelo enseña hoy
}
DISTS = (1.0, 1.5, 1.8, 2.5)
H_CAM = 1.15

create_new_stage()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.Xform.Define(stage, "/World")

def material(path, color, rug=0.8):
    m = UsdShade.Material.Define(stage, path)
    s = UsdShade.Shader.Define(stage, path + "/S")
    s.CreateIdAttr("UsdPreviewSurface")
    s.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    s.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rug)
    m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(), "surface")
    return m

suelo = UsdGeom.Cube.Define(stage, "/World/Suelo"); suelo.GetSizeAttr().Set(1.0)
xs = UsdGeom.Xformable(suelo.GetPrim())
xs.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.006)); xs.AddScaleOp().Set(Gf.Vec3f(20, 20, 0.012))
UsdShade.MaterialBindingAPI(suelo.GetPrim()).Bind(material("/World/Looks/Moq", tuple(col["suelo"]), 0.95))
fondo = UsdGeom.Cube.Define(stage, "/World/Fondo"); fondo.GetSizeAttr().Set(1.0)
xf = UsdGeom.Xformable(fondo.GetPrim())
xf.AddTranslateOp().Set(Gf.Vec3d(-2.5, 0, 1.2)); xf.AddScaleOp().Set(Gf.Vec3f(0.2, 12, 2.4))
UsdShade.MaterialBindingAPI(fondo.GetPrim()).Bind(material("/World/Looks/Par", tuple(col["pared"]), 0.9))

key = UsdLux.DistantLight.Define(stage, "/World/Key"); key.CreateIntensityAttr(1000.0)
UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-45, 20, 0))
UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr(400.0)

CAM = "/World/Cam"
c = UsdGeom.Camera.Define(stage, CAM)
c.CreateFocalLengthAttr(16.0)
xc = UsdGeom.Xformable(c.GetPrim()); xc.ClearXformOpOrder()
OT = xc.AddTranslateOp(); OR_ = xc.AddRotateXYZOp()
rp = rep.create.render_product(CAM, (960, 540))
ann = rep.AnnotatorRegistry.get_annotator("rgb")
ann.attach([rp])

def captura(nombre):
    for _ in range(10):
        rep.orchestrator.step(rt_subframes=2)
    d = ann.get_data()
    Image.fromarray(np.asarray(d)[:, :, :3].astype("uint8")).save("/ws/bench/%s.jpg" % nombre, quality=90)

for nombre, usd in CANDIDATAS.items():
    path = "/World/Obj"
    if usd:
        add_reference_to_stage(usd_path=usd, prim_path=path)
    else:
        cb = UsdGeom.Cube.Define(stage, path); cb.GetSizeAttr().Set(1.0)
        xb = UsdGeom.Xformable(cb.GetPrim())
        xb.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.4)); xb.AddScaleOp().Set(Gf.Vec3f(0.6, 0.6, 0.8))
        UsdShade.MaterialBindingAPI(cb.GetPrim()).Bind(material("/World/Looks/Blq", (0.46, 0.40, 0.34), 0.8))
    pr = stage.GetPrimAtPath(path)
    if usd:
        xo = UsdGeom.Xformable(pr); xo.ClearXformOpOrder()
        xo.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.0))
        xo.AddRotateZOp().Set(35.0)
    for d_ in DISTS:
        OT.Set(Gf.Vec3d(d_, 0, H_CAM))
        dz = 0.45 - H_CAM
        pitch = math.degrees(math.atan2(dz, d_))
        OR_.Set(Gf.Vec3f(90.0 + pitch, 0.0, 90.0))
        captura("%s_%.1f" % (nombre, d_))
    stage.RemovePrim(path)
    print("hecho:", nombre, flush=True)

print("=== BENCH RENDER OK ===")
app.close()
