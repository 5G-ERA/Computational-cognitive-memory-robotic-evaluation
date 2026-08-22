"""P2: la oficina real en USD, generada de la MISMA fuente que lab.world.

Frame: G1 (summit/ref_map_g1.json). Por construccion los waypoints reales valen sin traducir:
A(0.99,0.57) B(-4.73,3.04) C(-0.03,-1.49); puerta en (-3.90,1.25) eje 135 deg.

Estrategia: agrupar las celdas del mapa de referencia en SEGMENTOS de pared (barrido por filas
y columnas), tallar el vano de la puerta, y emitir cada segmento como un Cube USD con material
de pared. El tramo ACRISTALADO se emite aparte con material transmisivo (el testigo W1: el
lidar RTX deberia atravesarlo mientras el mapa lo predice).

Uso (dentro del contenedor):  ./python.sh /ws/p2_office.py
Salida: /ws/office.usd  +  /ws/shots_office/rgb_*.png
"""
import json
import math
import os

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": 1600, "height": 900})

from pxr import Usd, UsdGeom, UsdShade, UsdPhysics, Sdf, Gf, UsdLux
import omni.usd
import omni.replicator.core as rep
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path

OC = 0.2
ALTO = 2.2
DOOR = (-3.90, 1.25)
DOOR_R = 0.5
# tramo acristalado declarado (rectangulo en frame del mapa) — el testigo W1
CRISTAL = (-3.75, -0.55, -2.65, 0.75)

_OUT = open("/ws/p2_result.txt", "w")
def log(*a):
    s = " ".join(str(x) for x in a)
    print(s, flush=True); _OUT.write(s + "\n"); _OUT.flush()

pts = json.load(open("/ws/ref_map_g1.json"))["points"]
celdas = {(round(p[0] / OC), round(p[1] / OC)) for p in pts}
log("celdas PARED (ref_map):", len(celdas))
# muebles: nav_map, extruidos BAJOS (0.8 m) — misma distincion que lab.world de Gazebo
_nav = json.load(open("/ws/nav_map.json"))
muebles = {(int(c[0]), int(c[1])) for c in _nav.get("cells", [])}
muebles -= celdas                                  # lo que ya es pared manda
log("celdas MUEBLE (nav_map):", len(muebles))

# quitar el vano de la puerta (como hace lab.world)
def en_puerta(c):
    return math.hypot(c[0] * OC - DOOR[0], c[1] * OC - DOOR[1]) < DOOR_R
celdas = {c for c in celdas if not en_puerta(c)}
log("tras tallar el vano:", len(celdas))

def en_cristal(c):
    x, y = c[0] * OC, c[1] * OC
    return CRISTAL[0] <= x <= CRISTAL[2] and CRISTAL[1] <= y <= CRISTAL[3]

# --- agrupar celdas en segmentos horizontales (barrido por fila) ---
def segmentos(cs):
    segs = []
    porfila = {}
    for (cx, cy) in cs:
        porfila.setdefault(cy, []).append(cx)
    for cy, xs in porfila.items():
        xs.sort()
        ini = prev = xs[0]
        for x in xs[1:]:
            if x == prev + 1:
                prev = x
            else:
                segs.append((ini, prev, cy)); ini = prev = x
        segs.append((ini, prev, cy))
    return segs

muro = segmentos({c for c in celdas if not en_cristal(c)})
vidrio = segmentos({c for c in celdas if en_cristal(c)})
log("segmentos de pared:", len(muro), " segmentos de cristal:", len(vidrio))

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()

def material(path, color, opacidad=1.0, ior=1.0, rugosidad=0.5, metalico=0.0):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rugosidad)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metalico)
    if opacidad < 1.0:
        sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacidad)
        sh.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(ior)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat

# COLORES MEDIDOS en 301 fotogramas de la camara del G1 (dataset/colores_reales.json)
_col = json.load(open("/ws/colores_reales.json"))
COL_SUELO = tuple(_col["suelo"])       # moqueta azul-gris real
COL_PARED = tuple(_col["pared"])
log("colores reales -> suelo %s  pared %s" % (COL_SUELO, COL_PARED))

mat_pared = material("/World/Looks/Pared", COL_PARED)
mat_suelo = material("/World/Looks/Moqueta", COL_SUELO, rugosidad=0.95)
# el suelo por defecto de Isaac es una rejilla azul: lo cubrimos con la moqueta real
_piso = UsdGeom.Cube.Define(stage, "/World/Oficina/Moqueta")
_piso.GetSizeAttr().Set(1.0)
_pf = UsdGeom.Xformable(_piso.GetPrim())
_pf.AddTranslateOp().Set(Gf.Vec3d(-1.0, 1.5, 0.008))   # JUSTO ENCIMA del plano por defecto
_pf.AddScaleOp().Set(Gf.Vec3f(26.0, 26.0, 0.012))
UsdShade.MaterialBindingAPI(_piso.GetPrim()).Bind(mat_suelo)
mat_vidrio = material("/World/Looks/CristalTintado", (0.18, 0.22, 0.25),
                      opacidad=0.25, ior=1.52, rugosidad=0.05)

UsdGeom.Xform.Define(stage, "/World/Oficina")

def emite(segs, prefijo, mat, padre, alto=ALTO):
    n = 0
    for (x0, x1, cy) in segs:
        largo = (x1 - x0 + 1) * OC
        cx = (x0 + x1) / 2.0 * OC
        cyv = cy * OC
        path = "%s/%s_%d" % (padre, prefijo, n)
        cube = UsdGeom.Cube.Define(stage, path)
        cube.GetSizeAttr().Set(1.0)
        xf = UsdGeom.Xformable(cube.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(cx, cyv, alto / 2.0))
        xf.AddScaleOp().Set(Gf.Vec3f(largo, OC, alto))
        UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(mat)
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        n += 1
    return n

mat_mueble = material("/World/Looks/Mueble", (0.55, 0.42, 0.30), rugosidad=0.7)
UsdGeom.Xform.Define(stage, "/World/Oficina/Paredes")
UsdGeom.Xform.Define(stage, "/World/Oficina/Cristal")
UsdGeom.Xform.Define(stage, "/World/Oficina/Muebles")
np_ = emite(muro, "pared", mat_pared, "/World/Oficina/Paredes")
nv = emite(vidrio, "cristal", mat_vidrio, "/World/Oficina/Cristal")
nm = emite(segmentos(muebles), "mueble", mat_mueble, "/World/Oficina/Muebles", alto=0.8)
# (los props semanticos se anaden despues; se solapan a proposito con las celdas del laser,
#  que son la verdad de OCUPACION, mientras el prop aporta la IDENTIDAD)
log("prims: pared", np_, "cristal", nv, "mueble", nm)

# --- MUEBLES SEMANTICOS: donde la CAMARA del G1 los vio (133 runs reales) ---
# Posiciones MEDIDAS (proyeccion rumbo+rango desde la pose); dimensiones y color son
# plausibles por clase, declarado: la camara da DONDE y QUE, no la geometria exacta.
PROPS = {                       # (ancho, fondo, alto, color)
    "chair":        (0.60, 0.60, 0.90, (0.22, 0.24, 0.28)),
    "couch":        (1.80, 0.85, 0.75, (0.86, 0.83, 0.76)),   # los sofas claros de la oficina
    "refrigerator": (0.60, 0.50, 0.70, (0.62, 0.48, 0.32)),   # en realidad CAJAS de carton
}
objetos = json.load(open("/ws/objetos_vistos.json"))
UsdGeom.Xform.Define(stage, "/World/Oficina/Objetos")
nobj = 0
_ocupado = []
for lab, lista in objetos.items():
    if lab not in PROPS:
        continue
    w_, d_, h_, col = PROPS[lab]
    mat_o = material("/World/Looks/Obj_%s" % lab, col, rugosidad=0.8)
    for o in lista:
        if o["n"] < 8:                       # solo lo visto MUCHAS veces
            continue
        path = "/World/Oficina/Objetos/%s_%d" % (lab, nobj)
        cb = UsdGeom.Cube.Define(stage, path)
        cb.GetSizeAttr().Set(1.0)
        xo = UsdGeom.Xformable(cb.GetPrim())
        xo.AddTranslateOp().Set(Gf.Vec3d(o["x"], o["y"], h_ / 2.0))
        xo.AddScaleOp().Set(Gf.Vec3f(w_, d_, h_))
        UsdShade.MaterialBindingAPI(cb.GetPrim()).Bind(mat_o)
        UsdPhysics.CollisionAPI.Apply(cb.GetPrim())
        _ocupado.append((o["x"], o["y"]))
        nobj += 1
log("muebles semanticos colocados:", nobj)

# luces
key = UsdLux.DistantLight.Define(stage, "/World/Key")
key.CreateIntensityAttr(900.0)
UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-40, 25, 0))
UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr(250.0)

# el G1 en el waypoint A real
root = get_assets_root_path()
add_reference_to_stage(usd_path=root + "/Isaac/Robots/Unitree/G1/g1.usd", prim_path="/World/G1")
g1 = stage.GetPrimAtPath("/World/G1")
xg = UsdGeom.Xformable(g1); xg.ClearXformOpOrder()
xg.AddTranslateOp().Set(Gf.Vec3d(0.99, 0.57, 0.80))
xg.AddRotateZOp().Set(-120.0)
log("G1 colocado en el waypoint A real (0.99, 0.57)")

world.reset()

# guardar el USD
stage.GetRootLayer().Export("/ws/office.usd")
log("USD escrito: /ws/office.usd")

# VISTA 1: cenital de toda la planta (para ver el trazado real de la oficina)
cam1 = rep.create.camera(position=(0.0, 0.0, 30.0), look_at=(0.0, 0.0, 0.0))
rp1 = rep.create.render_product(cam1, (1600, 900))
w1 = rep.WriterRegistry.get("BasicWriter")
w1.initialize(output_dir="/ws/shots_planta", rgb=True)
w1.attach([rp1])
for _ in range(40):
    world.step(render=True)
for _ in range(10):
    rep.orchestrator.step()
w1.detach()

# VISTA 2: oblicua alta hacia la puerta y el cristal, con el G1 a la vista
cam2 = rep.create.camera(position=(4.5, -5.5, 9.5), look_at=(-3.5, 2.2, 0.8))
rp2 = rep.create.render_product(cam2, (1600, 900))
w2 = rep.WriterRegistry.get("BasicWriter")
w2.initialize(output_dir="/ws/shots_puerta", rgb=True)
w2.attach([rp2])
for _ in range(10):
    world.step(render=True)
for _ in range(10):
    rep.orchestrator.step()
log("=== P2 OK ===")
app.close()
