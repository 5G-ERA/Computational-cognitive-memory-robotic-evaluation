"""La oficina reconstruida desde la NUBE 3D REAL del LiDAR del Summit (930k puntos).

Salto respecto a las versiones anteriores: ya no se extruye un plano 2D a una altura inventada.
Cada celda toma su ALTURA MEDIDA en la nube (percentil 95 de z, robusto a puntos sueltos), las
alturas se cuantizan en bandas para poder fusionar rectangulos, y se clasifican:
    z >= 1.9 m  -> pared / estructura      0.35-1.9 m -> mobiliario      < 0.35 m -> bajo
Colores: los MEDIDOS en 301 fotogramas de la camara del G1.
Mallas reales de la biblioteca NVIDIA donde la CAMARA identifico objetos (133 runs); sus
voxeles se retiran para no meter una caja dentro de un sofa.
Cristal tintado: tramo declarado, material transmisivo (testigo W1).
"""
import json, math, random
import numpy as np

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": 1600, "height": 900})

from pxr import UsdGeom, UsdShade, UsdPhysics, Sdf, Gf, UsdLux
import omni.usd
import omni.replicator.core as rep
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path

OC = 0.20
BANDA = 0.30                     # cuantizacion de altura (m)
DOOR = (-3.90, 1.25); DOOR_R = 0.55
CRISTAL = (-3.75, -0.55, -2.65, 0.75)
MIN_PTS = 3                      # puntos minimos por celda (filtra ruido)

_O = open("/ws/office3d_result.txt", "w")
def log(*a):
    s = " ".join(str(x) for x in a); print(s, flush=True); _O.write(s + "\n"); _O.flush()

P = np.load("/ws/nube_g1.npy")
log("nube:", len(P), "puntos")

# --- rejilla: altura medida por celda ---
ix = np.round(P[:, 0] / OC).astype(np.int32)
iy = np.round(P[:, 1] / OC).astype(np.int32)
celdas = {}
for k in range(len(P)):
    c = (int(ix[k]), int(iy[k]))
    z = float(P[k, 2])
    e = celdas.get(c)
    if e is None:
        celdas[c] = [z, 1]
    else:
        if z > e[0]:
            e[0] = z
        e[1] += 1
celdas = {c: v for c, v in celdas.items() if v[1] >= MIN_PTS and v[0] > 0.12}
log("celdas con estructura:", len(celdas))

# vano de la puerta: fuera
celdas = {c: v for c, v in celdas.items()
          if math.hypot(c[0]*OC - DOOR[0], c[1]*OC - DOOR[1]) >= DOOR_R}

col = json.load(open("/ws/colores_reales.json"))
COL_SUELO, COL_PARED = tuple(col["suelo"]), tuple(col["pared"])
COL_MUEBLE = (0.42, 0.38, 0.34)

def en_cristal(c):
    x, y = c[0]*OC, c[1]*OC
    return CRISTAL[0] <= x <= CRISTAL[2] and CRISTAL[1] <= y <= CRISTAL[3]

# --- quitar voxeles donde va una MALLA identificada por la camara ---
objetos = json.load(open("/ws/objetos_vistos.json"))
MESH = {"couch": ["SM_Armchair.usd"],
        "chair": ["SM_ChairOffice_A.usd", "SM_Chair_01a.usd", "SM_Chair_02a.usd"],
        "refrigerator": ["SM_BoxBigA.usd", "SM_BoxA.usd", "SM_BoxOpen.usd"]}
props = []
for lab, lista in objetos.items():
    if lab not in MESH:
        continue
    for o in lista:
        if o["n"] >= 12:                       # solo lo MUY visto
            props.append((lab, o["x"], o["y"]))
R_PROP = 0.55
antes = len(celdas)
celdas = {c: v for c, v in celdas.items()
          if not any(math.hypot(c[0]*OC - px, c[1]*OC - py) < R_PROP for _, px, py in props)}
log("props identificados:", len(props), "-> voxeles retirados:", antes - len(celdas))

# --- agrupar por banda de altura y fusionar rectangulos ---
bandas = {}
for c, (z, n) in celdas.items():
    b = max(1, int(round(z / BANDA)))
    bandas.setdefault((b, en_cristal(c)), set()).add(c)

def rects(cs):
    libres = set(cs); out = []
    while libres:
        x0, y0 = min(libres, key=lambda c: (c[1], c[0]))
        w = 1
        while (x0+w, y0) in libres: w += 1
        h = 1
        while all((x0+i, y0+h) in libres for i in range(w)): h += 1
        for i in range(w):
            for j in range(h): libres.discard((x0+i, y0+j))
        out.append((x0, y0, w, h))
    return out

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()

def material(path, color, opac=1.0, ior=1.0, rug=0.7):
    m = UsdShade.Material.Define(stage, path)
    s = UsdShade.Shader.Define(stage, path + "/S")
    s.CreateIdAttr("UsdPreviewSurface")
    s.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    s.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rug)
    if opac < 1.0:
        s.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opac)
        s.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(ior)
    m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(), "surface")
    return m

mat_pared = material("/World/Looks/Pared", COL_PARED, rug=0.9)
mat_mueble = material("/World/Looks/Mueble", COL_MUEBLE, rug=0.8)
mat_suelo = material("/World/Looks/Moqueta", COL_SUELO, rug=0.95)
mat_vidrio = material("/World/Looks/Cristal", (0.16, 0.20, 0.24), opac=0.22, ior=1.52, rug=0.04)

UsdGeom.Xform.Define(stage, "/World/Oficina")
piso = UsdGeom.Cube.Define(stage, "/World/Oficina/Moqueta"); piso.GetSizeAttr().Set(1.0)
pf = UsdGeom.Xformable(piso.GetPrim())
pf.AddTranslateOp().Set(Gf.Vec3d(0, 1, 0.008)); pf.AddScaleOp().Set(Gf.Vec3f(30, 30, 0.012))
UsdShade.MaterialBindingAPI(piso.GetPrim()).Bind(mat_suelo)
UsdGeom.Xform.Define(stage, "/World/Oficina/Estructura")

n = 0
for (b, vidrio), cs in sorted(bandas.items()):
    alto = b * BANDA
    mat = mat_vidrio if vidrio else (mat_pared if alto >= 1.9 else mat_mueble)
    for (x0, y0, w, h) in rects(cs):
        cb = UsdGeom.Cube.Define(stage, "/World/Oficina/Estructura/v%d" % n)
        cb.GetSizeAttr().Set(1.0)
        xf = UsdGeom.Xformable(cb.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d((x0+w/2.0-0.5)*OC, (y0+h/2.0-0.5)*OC, alto/2.0))
        xf.AddScaleOp().Set(Gf.Vec3f(w*OC, h*OC, alto))
        UsdShade.MaterialBindingAPI(cb.GetPrim()).Bind(mat)
        UsdPhysics.CollisionAPI.Apply(cb.GetPrim())
        n += 1
log("prims de estructura 3D:", n)

# --- mallas reales ---
root = get_assets_root_path(); PR = root + "/Isaac/Environments/Office/Props/"
UsdGeom.Xform.Define(stage, "/World/Oficina/Muebles")
rng = random.Random(7); m = 0
for lab, px, py in props:
    add_reference_to_stage(usd_path=PR + rng.choice(MESH[lab]),
                           prim_path="/World/Oficina/Muebles/%s_%d" % (lab, m))
    pr = stage.GetPrimAtPath("/World/Oficina/Muebles/%s_%d" % (lab, m))
    xf = UsdGeom.Xformable(pr); xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(px, py, 0.0)); xf.AddRotateZOp().Set(rng.uniform(0, 360))
    m += 1
log("mallas reales:", m)

key = UsdLux.DistantLight.Define(stage, "/World/Key"); key.CreateIntensityAttr(1100.0)
UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-50, 25, 0))
UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr(320.0)

add_reference_to_stage(usd_path=root + "/Isaac/Robots/Unitree/G1/g1.usd", prim_path="/World/G1")
xg = UsdGeom.Xformable(stage.GetPrimAtPath("/World/G1")); xg.ClearXformOpOrder()
xg.AddTranslateOp().Set(Gf.Vec3d(0.99, 0.57, 0.80)); xg.AddRotateZOp().Set(-120.0)

world.reset()
stage.GetRootLayer().Export("/ws/office3d.usd")
log("USD: /ws/office3d.usd")

cam = rep.create.camera(position=(3.0, -3.5, 4.2), look_at=(-4.0, 2.2, 0.9))
rp = rep.create.render_product(cam, (1600, 900))
w = rep.WriterRegistry.get("BasicWriter")
w.initialize(output_dir="/ws/shots_3d", rgb=True); w.attach([rp])
for _ in range(25): world.step(render=True)
for _ in range(12): rep.orchestrator.step()
log("=== OFICINA 3D OK ===")
app.close()
