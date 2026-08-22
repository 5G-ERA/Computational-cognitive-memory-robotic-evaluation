"""La oficina real en USD con MALLAS de la biblioteca NVIDIA (v6).

Mejoras sobre la version de cajas:
  - PAREDES: fusion codiciosa 2D (rectangulos) en vez de tiras por fila -> muchos menos prims
    y sin el escalonado feo. Material con el color MEDIDO en 301 fotogramas del G1.
  - MUEBLES: mallas reales de /Isaac/Environments/Office/Props colocadas en las posiciones
    MEDIDAS por la camara del robot (133 runs, proyeccion rumbo+rango desde la pose):
        sofa   -> SM_Armchair        silla -> SM_ChairOffice_A
        caja   -> SM_BoxBigA / SM_BoxA (las "refrigerator" que COCO confunde)
  - CRISTAL tintado en su tramo declarado, material transmisivo (testigo W1).
  - Suelo con el color real de la moqueta.
Declarado: posiciones y colores MEDIDOS; la eleccion de malla por clase es plausible.
"""
import json, math, random

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": 1600, "height": 900})

from pxr import UsdGeom, UsdShade, UsdPhysics, Sdf, Gf, UsdLux
import omni.usd
import omni.replicator.core as rep
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path

OC = 0.2
ALTO = 2.4
DOOR = (-3.90, 1.25)
DOOR_R = 0.55
CRISTAL = (-3.75, -0.55, -2.65, 0.75)

_OUT = open("/ws/p3_result.txt", "w")
def log(*a):
    s = " ".join(str(x) for x in a)
    print(s, flush=True); _OUT.write(s + "\n"); _OUT.flush()

root = get_assets_root_path()
PROPS = root + "/Isaac/Environments/Office/Props/"

col = json.load(open("/ws/colores_reales.json"))
COL_SUELO, COL_PARED = tuple(col["suelo"]), tuple(col["pared"])
log("colores medidos -> suelo", COL_SUELO, " pared", COL_PARED)

celdas = {(round(p[0]/OC), round(p[1]/OC))
          for p in json.load(open("/ws/ref_map_g1.json"))["points"]}
celdas = {c for c in celdas
          if math.hypot(c[0]*OC - DOOR[0], c[1]*OC - DOOR[1]) >= DOOR_R}

def en_cristal(c):
    x, y = c[0]*OC, c[1]*OC
    return CRISTAL[0] <= x <= CRISTAL[2] and CRISTAL[1] <= y <= CRISTAL[3]

pared_cells = {c for c in celdas if not en_cristal(c)}
vidrio_cells = {c for c in celdas if en_cristal(c)}

def rectangulos(cs):
    """Fusion codiciosa 2D: cubre el conjunto con los rectangulos mas grandes posibles."""
    libres = set(cs); rects = []
    while libres:
        x0, y0 = min(libres, key=lambda c: (c[1], c[0]))
        w = 1
        while (x0 + w, y0) in libres:
            w += 1
        h = 1
        while all((x0 + i, y0 + h) in libres for i in range(w)):
            h += 1
        for i in range(w):
            for j in range(h):
                libres.discard((x0 + i, y0 + j))
        rects.append((x0, y0, w, h))
    return rects

r_pared, r_vidrio = rectangulos(pared_cells), rectangulos(vidrio_cells)
log("paredes: %d celdas -> %d rectangulos (antes 298 tiras)" % (len(pared_cells), len(r_pared)))

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()

def material(path, color, opac=1.0, ior=1.0, rug=0.6, met=0.0):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/S")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rug)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(met)
    if opac < 1.0:
        sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opac)
        sh.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(ior)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat

mat_pared = material("/World/Looks/Pared", COL_PARED, rug=0.85)
mat_suelo = material("/World/Looks/Moqueta", COL_SUELO, rug=0.95)
mat_vidrio = material("/World/Looks/Cristal", (0.16, 0.20, 0.24), opac=0.22, ior=1.52, rug=0.04)

UsdGeom.Xform.Define(stage, "/World/Oficina")
piso = UsdGeom.Cube.Define(stage, "/World/Oficina/Moqueta")
piso.GetSizeAttr().Set(1.0)
pf = UsdGeom.Xformable(piso.GetPrim())
pf.AddTranslateOp().Set(Gf.Vec3d(-1.0, 1.5, 0.008)); pf.AddScaleOp().Set(Gf.Vec3f(28, 28, 0.012))
UsdShade.MaterialBindingAPI(piso.GetPrim()).Bind(mat_suelo)

def emite_rects(rects, padre, mat, alto):
    UsdGeom.Xform.Define(stage, padre)
    for i, (x0, y0, w, h) in enumerate(rects):
        cb = UsdGeom.Cube.Define(stage, "%s/r_%d" % (padre, i))
        cb.GetSizeAttr().Set(1.0)
        xf = UsdGeom.Xformable(cb.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d((x0 + w/2.0 - 0.5)*OC, (y0 + h/2.0 - 0.5)*OC, alto/2.0))
        xf.AddScaleOp().Set(Gf.Vec3f(w*OC, h*OC, alto))
        UsdShade.MaterialBindingAPI(cb.GetPrim()).Bind(mat)
        UsdPhysics.CollisionAPI.Apply(cb.GetPrim())
    return len(rects)

n1 = emite_rects(r_pared, "/World/Oficina/Paredes", mat_pared, ALTO)
n2 = emite_rects(r_vidrio, "/World/Oficina/Cristal", mat_vidrio, ALTO)
log("prims pared:", n1, " cristal:", n2)

# --- MALLAS REALES donde la camara vio los objetos ---
MESH = {
    "couch":        (["SM_Armchair.usd"], 1.0),
    "chair":        (["SM_ChairOffice_A.usd", "SM_Chair_01a.usd", "SM_Chair_02a.usd"], 1.0),
    "refrigerator": (["SM_BoxBigA.usd", "SM_BoxA.usd", "SM_BoxOpen.usd"], 1.0),
}
objetos = json.load(open("/ws/objetos_vistos.json"))
UsdGeom.Xform.Define(stage, "/World/Oficina/Muebles")
rng = random.Random(7)
n = 0
for lab, lista in objetos.items():
    if lab not in MESH:
        continue
    mallas, esc = MESH[lab]
    for o in lista:
        if o["n"] < 8:
            continue
        usdp = PROPS + rng.choice(mallas)
        path = "/World/Oficina/Muebles/%s_%d" % (lab, n)
        add_reference_to_stage(usd_path=usdp, prim_path=path)
        pr = stage.GetPrimAtPath(path)
        xf = UsdGeom.Xformable(pr); xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(o["x"], o["y"], 0.0))
        xf.AddRotateZOp().Set(rng.uniform(0, 360))
        xf.AddScaleOp().Set(Gf.Vec3f(esc, esc, esc))
        n += 1
log("mallas reales colocadas:", n)

key = UsdLux.DistantLight.Define(stage, "/World/Key"); key.CreateIntensityAttr(1200.0)
UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-45, 30, 0))
UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr(350.0)

add_reference_to_stage(usd_path=root + "/Isaac/Robots/Unitree/G1/g1.usd", prim_path="/World/G1")
g1 = stage.GetPrimAtPath("/World/G1")
xg = UsdGeom.Xformable(g1); xg.ClearXformOpOrder()
xg.AddTranslateOp().Set(Gf.Vec3d(0.99, 0.57, 0.80)); xg.AddRotateZOp().Set(-120.0)

world.reset()
stage.GetRootLayer().Export("/ws/office_v2.usd")
log("USD escrito: /ws/office_v2.usd")

cam = rep.create.camera(position=(2.5, -4.0, 5.0), look_at=(-4.0, 2.4, 0.8))
rp = rep.create.render_product(cam, (1600, 900))
w = rep.WriterRegistry.get("BasicWriter")
w.initialize(output_dir="/ws/shots_meshes", rgb=True); w.attach([rp])
for _ in range(30):
    world.step(render=True)
for _ in range(14):
    rep.orchestrator.step()
log("=== OFICINA CON MALLAS OK ===")
app.close()
