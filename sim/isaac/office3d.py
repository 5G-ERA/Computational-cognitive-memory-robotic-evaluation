"""Oficina 3D v8 — cristal a TIRAS para el lidar RTX (23-ago).

El FlatScan RTX ignora la transparencia de cualquier material (PreviewSurface y OmniGlass
devuelven 31/31); lo unico que respeta es `primvars:doNotCastRays`. Modelo efectivo del
cristal tintado: cada celda de cristal se subdivide en TIRAS verticales de ~3.3 cm a lo largo
de la pared, con patron determinista 5 visibles / 4 invisibles (44.4% de ausencia frontal =
el 0.44 medido en el cristal real el 21-ago). La tendencia angular (mas retornos en oblicuo,
0.32@30 grados real) debe EMERGER de la geometria: un rayo oblicuo cruza mas tiras.
OJO declarado: doNotCastRays puede afectar tambien al render de camara (tiras en la imagen);
se verificara en el peldano 3 y, si molesta, se usaran dos geometrias solapadas (una solo
para sensores, otra solo visual).

Pregunta de Adrian: ¿impacta poner mallas sobre nuestro object detector? MEDIDO con el banco
(bench_mallas.py + eval_bench.py, el mismo yolo11x del perception_server):
    SM_ChairOffice_A -> chair 0.92-0.93  = CLAVA la firma real (chair 0.82-0.92)   APROBADA
    SM_Chair_01a     -> NADA             (silla invisible al canal de vision)      SUSPENSA
    SM_Armchair      -> tv 0.61-0.74 (!) (donde la realidad da couch 0.95)         SUSPENSA
    SM_BoxBigA/BoxA  -> toaster/truck    (la realidad da refrigerator ~0.9)        SUSPENSAS
    BLOQUE GRIS      -> nada             (ciego pero honesto: no inyecta mentiras)
DOCTRINA: una malla entra en escena solo si provoca en NUESTRO detector la misma firma que
el objeto real. Mientras no haya malla aprobada para sofa/caja, se quedan como bloques.

Queja de Adrian sobre la v3, correcta: al poner una silla/sofa de malla, su bloque gris
seguia alli debajo/alrededor. Dos causas medibles:
  - el mapa 2D de "paredes" (ref_map) tambien contiene los MUEBLES ALTOS (el lidar 2D a
    altura de torso ve el respaldo del sofa como pared), y la v3 se negaba a ceder celdas
    de pared;
  - el radio fijo de 0.55 m era corto para sofas/mesas.

Regla v4: cada malla se carga PRIMERO, se mide su BOUNDING BOX real en el mundo (+0.12 m de
margen), y se cede TODO bloque cuyo centro caiga dentro - venga de nav_map o de ref_map -
SALVO que la nube 3D diga que esa celda mide >= 1.9 m (pared de verdad: se queda, para no
abrir agujeros al lidar). El reparto de fuentes sigue siendo el de la v3 (2D=donde,
nube=altura, fotos=que/color).
"""
import json, math, os, random
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
H_PARED_INTOCABLE = 1.9      # una celda asi de alta nunca se cede: es pared real
MARGEN_BBOX = 0.35   # v6: generoso - la auditoria vio anillos de mobiliario a 0.5-0.6 m del centro

_O = open("/ws/office3d_v18_result.txt", "w")
def log(*a):
    s = " ".join(str(x) for x in a); print(s, flush=True); _O.write(s + "\n"); _O.flush()

pared_cells = {(round(p[0]/OC), round(p[1]/OC))
               for p in json.load(open("/ws/ref_map_g1.json"))["points"]}
nav = json.load(open("/ws/nav_map.json"))
mueble_cells = {(int(c[0]), int(c[1])) for c in nav.get("cells", [])} - pared_cells
log("2D limpio: pared", len(pared_cells), " mueble", len(mueble_cells))

P = np.load("/ws/nube_g1.npy").astype(np.float64)
fa, fb, fc = -0.0008, 0.0036, -0.0305
ix = np.round(P[:, 0]/OC).astype(int); iy = np.round(P[:, 1]/OC).astype(int)
from collections import defaultdict
zpor = defaultdict(list)
for k in range(len(P)):
    zpor[(ix[k], iy[k])].append(P[k, 2])

def altura(c, defecto):
    """Altura ROBUSTA a fantasmas (v18): banda de 0.3 m mas alta con densidad REAL.

    El p95 simple se dejaba enganar por los rastros de gente: una celda de sofa con puntos
    ralos de torsos por encima "media" 2 m y la regla la protegia como pared. Una superficie
    real acumula MUCHOS retornos en su banda; el rastro de un transeunte, pocos. Se exige a
    cada banda al menos max(6, 8% de los puntos de la celda) para contar."""
    zs = []
    for dx_ in (-1, 0, 1):
        for dy_ in (-1, 0, 1):
            zs.extend(zpor.get((c[0]+dx_, c[1]+dy_), ()))
    if len(zs) < 8:
        return defecto
    f = fa*c[0]*OC + fb*c[1]*OC + fc
    rel = np.array(zs) - f
    rel = rel[(rel > 0.05) & (rel < 2.8)]
    if len(rel) < 6:
        return defecto
    need = max(6, int(0.08 * len(rel)))
    tope = defecto
    for b_ in range(9):                       # bandas 0.0-0.3 ... 2.4-2.7
        lo, hi = b_ * 0.3, (b_ + 1) * 0.3
        if int(((rel >= lo) & (rel < hi)).sum()) >= need:
            tope = hi
    return max(0.3, min(2.7, float(tope)))

# v16: el vano se talla como CORREDOR, no como circulo. Medido en el mapa reconstruido: con
# un circulo de r=0.55 el pasaje se ESTRANGULA a 0.40 m medio metro antes del umbral, asi que
# el robot no aproximaba alineado -- enhebraba una ranura (llegaba 31 grados fuera de eje
# contra los 10 del robot real, y pasaba 4-5x mas tiempo maniobrando cerca). La puerta real es
# un PASAJE de ~1.13 m de boca y ~0.5 m de fondo (nota de lab.world), asi que se talla eso:
# ancho constante a lo largo del eje de cruce.
DOOR_W = float(os.environ.get("G1_DOOR_W", "1.13"))     # boca real
DOOR_L = float(os.environ.get("G1_DOOR_L", "1.10"))     # fondo del pasaje + margen de aproximacion
_AXR = math.radians(135.0)
_UX, _UY = math.cos(_AXR), math.sin(_AXR)

def en_puerta(c):
    dx, dy = c[0]*OC - DOOR[0], c[1]*OC - DOOR[1]
    s = dx*_UX + dy*_UY                 # a lo largo del eje de cruce
    p = -dx*_UY + dy*_UX                # perpendicular
    return abs(s) <= DOOR_L/2.0 and abs(p) <= DOOR_W/2.0
pared_cells = {c for c in pared_cells if not en_puerta(c)}
mueble_cells = {c for c in mueble_cells if not en_puerta(c)}

# --- LIMPIEZA POR TRAYECTORIA REAL (v18) ---
# Arbitro incontestable: por donde el robot paso fisicamente, no hay pared. De 38779 poses en
# 133 runs salen 928 celdas transitadas, y resulta que 122 celdas de "pared" y 100 de "mueble"
# estan entre ellas -- ocupacion espuria del mapa 2D y de la nube acumulada. Junto a la puerta
# eran 55 de 72 (76%), que es lo que estrangulaba el pasaje a 0.40 m y hacia que el robot
# llegase 31 grados fuera de eje en vez de los 10 del robot real.
try:
    _lib = {(int(c[0]), int(c[1]))
            for c in json.load(open("/ws/celdas_libres.json"))["cells"]}
    _np, _nm = len(pared_cells), len(mueble_cells)
    pared_cells -= _lib
    mueble_cells -= _lib
    log("limpieza por trayectoria: -%d celdas de pared, -%d de mueble (%d transitadas)" % (
        _np - len(pared_cells), _nm - len(mueble_cells), len(_lib)))
except Exception as _e:
    log("AVISO: sin limpieza por trayectoria (%s)" % _e)

# --- SEGUNDA LIMPIEZA (v18): la CREENCIA DEL PROPIO G1 ---
# Una celda que el robot tuvo cerca >=60 veces, desde >=3 octantes distintos, y en la que su
# laser nunca puso obstaculo (<=2% de las veces), no esta ocupada -- por mucho que lo diga el
# mapa del Summit. Misma logica que cov_missing, aplicada al reves para depurar el mapa.
# OJO (error propio, corregido): un primer intento talló rayos desde la pose hasta cada punto
# del snapshot. Es INVALIDO -- 'pts' es el mapa de obstaculos ACUMULADO del robot, no el
# barrido instantaneo, asi que esos rayos nunca existieron; abria los flancos del vano.
try:
    _esp = {(int(c[0]), int(c[1]))
            for c in json.load(open("/ws/celdas_espurias.json"))["cells"]}
    _np, _nm = len(pared_cells), len(mueble_cells)
    pared_cells -= _esp
    mueble_cells -= _esp
    log("limpieza por creencia del G1: -%d celdas de pared, -%d de mueble" % (
        _np - len(pared_cells), _nm - len(mueble_cells)))
except Exception as _e:
    log("AVISO: sin limpieza por creencia (%s)" % _e)

# --- escena ---
create_new_stage()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.Xform.Define(stage, "/World")

# --- 1) MALLAS PRIMERO, con su bbox real ---
objetos = json.load(open("/ws/objetos_vistos.json"))
# v13: FUERA LAS MALLAS. Adrian no se fia de ellas y el banco le da la razon (SM_Armchair ->
# "tv" 0.74 en nuestro detector). En su lugar, TARJETAS con los PIXELES REALES: para cada
# objeto se recorta del fotograma real la region que produjo su deteccion y se coloca como
# quad texturizado en su posicion medida, con el ANCHO y ALTO fisicos derivados de la caja y
# el rango, apoyado en el suelo (todos son objetos de suelo). El detector no ve una
# aproximacion del sofa: ve EL SOFA, asi que la etiqueta y la confianza salen por construccion.
MESH = {}
props = [(lab, o["x"], o["y"]) for lab, ol in objetos.items() if lab in MESH
         for o in ol if o["n"] >= 12]
root = get_assets_root_path()
UsdGeom.Xform.Define(stage, "/World/Objetos")

def tarjeta(path, png, cx, cy, ancho, alto, mirar_x, mirar_y):
    """Quad texturizado con los pixeles reales, apoyado en el suelo y encarado al observador."""
    from pxr import UsdShade as _S, Vt
    ang = math.atan2(mirar_y - cy, mirar_x - cx)          # normal hacia el observador
    ux, uy = -math.sin(ang), math.cos(ang)                 # eje ancho (perpendicular a la normal)
    h = ancho / 2.0
    pts = [Gf.Vec3f(cx - ux*h, cy - uy*h, 0.0), Gf.Vec3f(cx + ux*h, cy + uy*h, 0.0),
           Gf.Vec3f(cx + ux*h, cy + uy*h, alto), Gf.Vec3f(cx - ux*h, cy - uy*h, alto)]
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateDoubleSidedAttr(True)
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                                 UsdGeom.Tokens.varying)
    st.Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
    mat = _S.Material.Define(stage, path + "/Mat")
    sh = _S.Shader.Define(stage, path + "/Mat/S")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    tex = _S.Shader.Define(stage, path + "/Mat/Tex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(png)
    tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
    lector = _S.Shader.Define(stage, path + "/Mat/St")
    lector.CreateIdAttr("UsdPrimvarReader_float2")
    lector.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        lector.ConnectableAPI(), "result")
    # EMISIVA, no difusa: el recorte YA lleva la iluminacion real de la oficina. Si ademas se
    # ilumina con la luz de la escena se cuenta dos veces y la tarjeta sale negra (medido).
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0, 0, 0))
    sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        tex.ConnectableAPI(), "rgb")
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    _S.MaterialBindingAPI(mesh.GetPrim()).Bind(mat)
    return mesh

fichas = json.load(open("/ws/recortes/fichas.json"))
# PUERTA DEL BANCO: solo entran las tarjetas que, renderizadas desde la pose original, hacen
# que NUESTRO detector diga la misma etiqueta con confianza cercana a la real (<=0.25).
# Medido: funciona con sillas (0.78-0.82 sim vs 0.83-0.93 real); sofas y cajas NO pasan -- sus
# recortes son parciales y "refrigerator" era ya una confusion de COCO guiada por contexto de
# escena, que una tarjeta plana no reproduce. Esos objetos se quedan como bloques.
try:
    _ap = json.load(open("/ws/recortes/aprobadas.json"))
    _ok = {(a_["lab"], round(a_["pos"][0], 2), round(a_["pos"][1], 2)) for a_ in _ap}
except Exception:
    _ok = None
cajas = []
corredores = []
m = 0
for f_ in fichas:
    px, py = f_.get("pos_obs") or f_["pos"]
    an, al = f_["ancho_m"], f_["alto_m"]
    if an < 0.30 or al < 0.30:                      # recortes parciales: no valen como tarjeta
        continue
    if _ok is not None and (f_["lab"], round(f_["pos"][0], 2), round(f_["pos"][1], 2)) not in _ok:
        continue                                     # no paso el banco detector
    tarjeta("/World/Objetos/%s_%d" % (f_["lab"], m), "/ws/recortes/" + f_["png"],
            px, py, an, al, f_["observador"][0], f_["observador"][1])
    r_ = max(an, 0.5) / 2.0 + MARGEN_BBOX
    cajas.append((px - r_, py - r_, px + r_, py + r_))
    corredores.append((f_["observador"][0], f_["observador"][1], px, py))
    m += 1
log("tarjetas con pixeles reales:", m, "de", len(fichas), "recortes")
props = [(f_["lab"], f_["pos"][0], f_["pos"][1]) for f_ in fichas]

# --- 2) CESION por bbox real, protegiendo pared alta ---
def dentro(c):
    x, y = c[0]*OC, c[1]*OC
    if any(x0 <= x <= x1 and y0 <= y <= y1 for (x0, y0, x1, y1) in cajas):
        return True
    # y el CORREDOR de vision entre el observador y su tarjeta: si un bloque se cruza ahi, la
    # tarjeta queda tapada y el detector no ve nada (medido en el primer banco).
    for (ox, oy, tx_, ty_) in corredores:
        dx_, dy_ = tx_-ox, ty_-oy
        L2 = dx_*dx_ + dy_*dy_
        if L2 < 1e-6:
            continue
        t = max(0.0, min(1.0, ((x-ox)*dx_ + (y-oy)*dy_) / L2))
        if math.hypot(x - (ox+t*dx_), y - (oy+t*dy_)) < 0.22:
            return True
    return False

ced_p = {c for c in pared_cells if dentro(c) and altura(c, 2.2) < H_PARED_INTOCABLE}
ced_m = {c for c in mueble_cells if dentro(c)}
pared_cells -= ced_p
mueble_cells -= ced_m
log("cedidas a las mallas: %d de pared(2D) + %d de mueble (protegidas %d de pared real >=%.1fm)" % (
    len(ced_p), len(ced_m),
    sum(1 for c in pared_cells if dentro(c)), H_PARED_INTOCABLE))

# v6: RECOLOCAR cada malla al centroide de las celdas que desaloja. La posicion de camara
# trae ~0.3-0.5 m de error (cuantizacion de rumbo + rango); la ocupacion del laser dice
# donde estaba el objeto DE VERDAD. Si no desalojo nada, se queda donde la vio la camara.
# (las tarjetas se colocan en la posicion medida por la camara; no se recolocan)

# --- 3) bloques restantes ---
def en_cristal(c):
    x, y = c[0]*OC, c[1]*OC
    return CRISTAL[0] <= x <= CRISTAL[2] and CRISTAL[1] <= y <= CRISTAL[3]

cristal_blob = {c for c in pared_cells if en_cristal(c)}
pared_cells = pared_cells - cristal_blob
# v10: el cristal es un PANEL FINO, no el pegote entero. El mapa 2D dentro del rect mezcla el
# panel con artefactos vistos DETRAS/A TRAVES del vidrio (1.1 m de fondo). La sala esta al
# ESTE (x mayores): el panel = la celda mas oriental de cada fila y; el resto del pegote se
# BORRA (tras el cristal real no hay retorno lidar).
_pane = {}
for c in cristal_blob:
    y = c[1]
    if y not in _pane or c[0] > _pane[y][0]:
        _pane[y] = c
cristal_cells = set(_pane.values())
log("cristal: pegote de %d celdas -> panel fino de %d (el resto, artefactos tras-cristal, borrado)" % (
    len(cristal_blob), len(cristal_cells)))
bandas = {}
for c in pared_cells:
    h = altura(c, 2.2)
    if h < 1.5:
        h = max(h, 1.8)
    bandas.setdefault(("P", int(round(h/BANDA)), False), set()).add(c)
for c in mueble_cells:
    h = min(altura(c, 0.8), 1.4)
    bandas.setdefault(("M", max(1, int(round(h/BANDA))), False), set()).add(c)

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

col = json.load(open("/ws/colores_reales.json"))
def material(path, color, opac=1.0, ior=1.0, rug=0.7):
    mt = UsdShade.Material.Define(stage, path)
    s = UsdShade.Shader.Define(stage, path + "/S")
    s.CreateIdAttr("UsdPreviewSurface")
    s.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    s.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rug)
    if opac < 1.0:
        s.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opac)
        s.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(ior)
    mt.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(), "surface")
    return mt

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
    mt = mat_vidrio if vid else (mat_pared if cls == "P" else mat_mueble)
    for (x0, y0, w, h) in rects(cs):
        cb = UsdGeom.Cube.Define(stage, "/World/Estructura/%s%d" % (cls.lower(), n))
        cb.GetSizeAttr().Set(1.0)
        xf = UsdGeom.Xformable(cb.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d((x0+w/2.0-0.5)*OC, (y0+h/2.0-0.5)*OC, alto/2.0))
        xf.AddScaleOp().Set(Gf.Vec3f(w*OC, h*OC, alto))
        UsdShade.MaterialBindingAPI(cb.GetPrim()).Bind(mt)
        n += 1
log("prims de estructura:", n)

# --- CRISTAL A TIRAS (patron 5 visibles / 4 doNotCastRays = 44.4% ausencia frontal) ---
ANCHO_TIRA = 0.0333
UsdGeom.Xform.Define(stage, "/World/Cristal")
nt = 0
for c in sorted(cristal_cells):
    h = max(altura(c, 2.2), 1.8)
    x0 = c[0]*OC - OC/2.0
    y0 = c[1]*OC - OC/2.0
    ntiras = int(round(OC / ANCHO_TIRA))
    for i in range(ntiras):
        yc = y0 + (i + 0.5) * ANCHO_TIRA
        idx = int(round(yc / ANCHO_TIRA))          # indice GLOBAL: patron continuo entre celdas
        # PATRON CALIBRADO EN BANCO (sim/isaac/calibra_cristal.py, 23-ago): panel recto, linea
        # limpia, barrido de 20 patrones a incidencia 0 y 30 grados. Ganador 3 visibles /
        # 5 invisibles, reproducible en 3 corridas: 36-40% de ausencia frontal y 32% en
        # oblicuo, contra las firmas reales 44% y 32%. Las medidas reales tienen solo 25
        # rumbos informativos (IC95 [25,63]% y [14,50]%), asi que el patron es
        # ESTADISTICAMENTE INDISTINGUIBLE del cristal real; afinar mas seria ajustar ruido.
        # La tendencia angular (menos ausencia en oblicuo) EMERGE de la geometria, no se impone.
        invisible = (idx % 8) >= 3                  # 3 visibles / 5 invisibles
        cb = UsdGeom.Cube.Define(stage, "/World/Cristal/t%d" % nt)
        cb.GetSizeAttr().Set(1.0)
        xf = UsdGeom.Xformable(cb.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(c[0]*OC, yc, h/2.0))
        xf.AddScaleOp().Set(Gf.Vec3f(OC, ANCHO_TIRA, h))
        UsdShade.MaterialBindingAPI(cb.GetPrim()).Bind(mat_vidrio)
        if invisible:
            # v11: doNotCastRays autorado NO afecta a este lidar (medido: firma identica con
            # patrones distintos; el "exito" del test en runtime era el sensor rompiendose).
            # Visibilidad USD si que lo quita de TODO el render, sensores incluidos.
            # Coste declarado: la camara vera el cristal a franjas; el arreglo fino
            # (materiales de sensor con reflectancia fisica) queda para el peldano 3.
            UsdGeom.Imageable(cb.GetPrim()).MakeInvisible()
        nt += 1
log("tiras de cristal:", nt, "(patron calibrado 3v/5i)")

add_reference_to_stage(usd_path=root + "/Isaac/Robots/Unitree/G1/g1.usd", prim_path="/World/G1")
g1 = stage.GetPrimAtPath("/World/G1")
xg = UsdGeom.Xformable(g1); xg.ClearXformOpOrder()
xg.AddTranslateOp().Set(Gf.Vec3d(0.99, 0.57, 0.793)); xg.AddRotateZOp().Set(-120.0)
q = 0
for prim in Usd.PrimRange(g1):
    for api in (UsdPhysics.ArticulationRootAPI, UsdPhysics.RigidBodyAPI, UsdPhysics.CollisionAPI):
        if prim.HasAPI(api):
            prim.RemoveAPI(api); q += 1
log("fisica despojada del G1:", q)

key = UsdLux.DistantLight.Define(stage, "/World/Key"); key.CreateIntensityAttr(1100.0)
UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-50, 25, 0))
UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr(320.0)

# defaultPrim: sin el, quien referencie este USD (en vez de abrirlo) se queda
# con la referencia sin resolver y la oficina no aparece.
stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
stage.GetRootLayer().Export("/ws/office3d.usd")
log("USD: /ws/office3d.usd (v18)")
log("=== OFICINA v18 OK ===")
app.close()
