"""Oficina 3D v3 — fusion correcta de las TRES fuentes (22-ago, noche).

LECCION que motiva la v3 (medida, no supuesta): la nube 3D cruda ACUMULA todos los barridos
del mapeo y no borra lo que se mueve. En celdas de la propia ruta A<->B (que el robot recorre
fisicamente, garantizadas libres) hay mediana 83% de puntos entre 0.2 y 1.8 m: FANTASMAS de
personas andando durante la sesion de mapeo del Summit. Por eso la v1/v2 sembraban la sala
de tacos ("suelo profundamente desigual", y no: el suelo del lab es plano con moqueta, y el
ajuste del plano lo confirma con 0.37 cm/m).

El mapa 2D de ocupacion (slam_toolbox) SI hace ray-clearing y borra a los que se mueven.
Reparto correcto:
    - ref_map_g1 (2D, limpio)  -> DONDE hay pared            (961 celdas)
    - nav_map    (2D, limpio)  -> DONDE hay mobiliario       (615 celdas)
    - nube 3D                  -> QUE ALTURA tiene cada celda (p95 relativo al plano del suelo)
    - fotos de la camara       -> QUE es cada cosa y de que color (mallas + colores medidos)
Celdas fuera de los mapas 2D: NADA, aunque la nube tenga puntos (son fantasmas).
Robot: fisica despojada (escena de inspeccion) -> DE PIE.
"""
import json, math, random
import numpy as np

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": 960, "height": 540})

from pxr import Usd, UsdGeom, UsdShade, UsdPhysics, Sdf, Gf, UsdLux
import omni.usd
from isaacsim.core.utils.stage import add_reference_to_stage, create_new_stage
from isaacsim.storage.native import get_assets_root_path

OC = 0.20
BANDA = 0.30
DOOR = (-3.90, 1.25); DOOR_R = 0.55
CRISTAL = (-3.75, -0.55, -2.65, 0.75)

_O = open("/ws/office3d_v3_result.txt", "w")
def log(*a):
    s = " ".join(str(x) for x in a); print(s, flush=True); _O.write(s + "\n"); _O.flush()

# --- fuentes 2D LIMPIAS ---
pared_cells = {(round(p[0]/OC), round(p[1]/OC))
               for p in json.load(open("/ws/ref_map_g1.json"))["points"]}
nav = json.load(open("/ws/nav_map.json"))
mueble_cells = {(int(c[0]), int(c[1])) for c in nav.get("cells", [])} - pared_cells
log("2D limpio: pared", len(pared_cells), " mueble", len(mueble_cells))

# --- nube 3D: SOLO para alturas ---
P = np.load("/ws/nube_g1.npy").astype(np.float64)
fa, fb, fc = -0.0008, 0.0036, -0.0305          # plano del suelo (ajuste robusto, 0.37 cm/m)
ix = np.round(P[:, 0]/OC).astype(int); iy = np.round(P[:, 1]/OC).astype(int)
from collections import defaultdict
zpor = defaultdict(list)
for k in range(len(P)):
    zpor[(ix[k], iy[k])].append(P[k, 2])

def altura(c, defecto):
    zs = []
    for dx_ in (-1, 0, 1):
        for dy_ in (-1, 0, 1):
            zs.extend(zpor.get((c[0]+dx_, c[1]+dy_), ()))
    if len(zs) < 8:
        return defecto
    f = fa*c[0]*OC + fb*c[1]*OC + fc
    h = float(np.percentile(np.array(zs), 95) - f)
    return max(0.3, min(2.7, h))

def en_puerta(c):
    return math.hypot(c[0]*OC - DOOR[0], c[1]*OC - DOOR[1]) < DOOR_R

pared_cells = {c for c in pared_cells if not en_puerta(c)}
mueble_cells = {c for c in mueble_cells if not en_puerta(c)}

# --- mallas de la camara: sus celdas se ceden ---
objetos = json.load(open("/ws/objetos_vistos.json"))
MESH = {"couch": ["SM_Armchair.usd"],
        "chair": ["SM_ChairOffice_A.usd", "SM_Chair_01a.usd"],
        "refrigerator": ["SM_BoxBigA.usd", "SM_BoxA.usd"]}
props = [(lab, o["x"], o["y"]) for lab, ol in objetos.items() if lab in MESH
         for o in ol if o["n"] >= 12]
def cerca_prop(c):
    return any(math.hypot(c[0]*OC-px, c[1]*OC-py) < 0.55 for _, px, py in props)
antes = len(pared_cells) + len(mueble_cells)
# OJO: las celdas de PARED no se ceden (una pared es una pared); solo mobiliario
mueble_cells = {c for c in mueble_cells if not cerca_prop(c)}
log("props camara:", len(props), " celdas de mueble cedidas:", antes - len(pared_cells) - len(mueble_cells))

def en_cristal(c):
    x, y = c[0]*OC, c[1]*OC
    return CRISTAL[0] <= x <= CRISTAL[2] and CRISTAL[1] <= y <= CRISTAL[3]

# --- bandas de altura por clase ---
bandas = {}
for c in pared_cells:
    h = altura(c, 2.2)
    if h < 1.5:
        h = max(h, 1.8)               # una pared del ref_map nunca es un zocalo: fantasma de altura no
    b = int(round(h / BANDA))
    bandas.setdefault(("P", b, en_cristal(c)), set()).add(c)
for c in mueble_cells:
    h = altura(c, 0.8)
    h = min(h, 1.4)                   # mobiliario: los puntos por encima son fantasmas/techo
    b = max(1, int(round(h / BANDA)))
    bandas.setdefault(("M", b, False), set()).add(c)

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

create_new_stage()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.Xform.Define(stage, "/World")

col = json.load(open("/ws/colores_reales.json"))
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

mat_pared = material("/World/Looks/Pared", tuple(col["pared"]), rug=0.9)
mat_mueble = material("/World/Looks/Mobiliario", (0.46, 0.40, 0.34), rug=0.8)
mat_suelo = material("/World/Looks/Moqueta", tuple(col["suelo"]), rug=0.95)
mat_vidrio = material("/World/Looks/Cristal", (0.16, 0.20, 0.24), opac=0.22, ior=1.52, rug=0.04)

piso = UsdGeom.Cube.Define(stage, "/World/Moqueta"); piso.GetSizeAttr().Set(1.0)
pf = UsdGeom.Xformable(piso.GetPrim())
pf.AddTranslateOp().Set(Gf.Vec3d(0, 1, -0.006)); pf.AddScaleOp().Set(Gf.Vec3f(34, 34, 0.012))
UsdShade.MaterialBindingAPI(piso.GetPrim()).Bind(mat_suelo)

UsdGeom.Xform.Define(stage, "/World/Estructura")
n = 0
for (cls, b, vid), cs in sorted(bandas.items()):
    alto = b * BANDA
    mat = mat_vidrio if vid else (mat_pared if cls == "P" else mat_mueble)
    for (x0, y0, w, h) in rects(cs):
        cb = UsdGeom.Cube.Define(stage, "/World/Estructura/%s%d" % (cls.lower(), n))
        cb.GetSizeAttr().Set(1.0)
        xf = UsdGeom.Xformable(cb.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d((x0+w/2.0-0.5)*OC, (y0+h/2.0-0.5)*OC, alto/2.0))
        xf.AddScaleOp().Set(Gf.Vec3f(w*OC, h*OC, alto))
        UsdShade.MaterialBindingAPI(cb.GetPrim()).Bind(mat)
        n += 1
log("prims de estructura:", n)

root = get_assets_root_path(); PR = root + "/Isaac/Environments/Office/Props/"
rng = random.Random(7); m = 0
UsdGeom.Xform.Define(stage, "/World/Muebles")
for lab, px, py in props:
    p_ = "/World/Muebles/%s_%d" % (lab, m)
    add_reference_to_stage(usd_path=PR + rng.choice(MESH[lab]), prim_path=p_)
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(p_)); xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(px, py, 0.0)); xf.AddRotateZOp().Set(rng.uniform(0, 360))
    m += 1
log("mallas reales:", m)

add_reference_to_stage(usd_path=root + "/Isaac/Robots/Unitree/G1/g1.usd", prim_path="/World/G1")
g1 = stage.GetPrimAtPath("/World/G1")
xg = UsdGeom.Xformable(g1); xg.ClearXformOpOrder()
xg.AddTranslateOp().Set(Gf.Vec3d(0.99, 0.57, 0.793)); xg.AddRotateZOp().Set(-120.0)
q = 0
for prim in Usd.PrimRange(g1):
    for api in (UsdPhysics.ArticulationRootAPI, UsdPhysics.RigidBodyAPI, UsdPhysics.CollisionAPI):
        if prim.HasAPI(api):
            prim.RemoveAPI(api); q += 1
log("APIs de fisica despojadas del G1:", q, "(de pie, escena de inspeccion)")

key = UsdLux.DistantLight.Define(stage, "/World/Key"); key.CreateIntensityAttr(1100.0)
UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-50, 25, 0))
UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr(320.0)

stage.GetRootLayer().Export("/ws/office3d.usd")
log("USD: /ws/office3d.usd (v3)")
log("=== OFICINA v3 OK ===")
app.close()
