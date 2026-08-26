"""Pase 2: paredes por habitacion, con extension y perfil de altura por muro.
Produce /tmp/paredes_v19.json con lineas medidas en (u,v) y de vuelta en (x,y)."""
import json, math
import numpy as np

P = np.load("/home/ros/isaac_ws/nube_g1.npy").astype(np.float64)
fa, fb, fc = -0.0008, 0.0036, -0.0305
zrel = P[:,2] - (fa*P[:,0] + fb*P[:,1] + fc)
A = math.radians(135.0)
ux, uy = math.cos(A), math.sin(A)
u = P[:,0]*ux + P[:,1]*uy
v = -P[:,0]*uy + P[:,1]*ux
alto = (zrel > 1.75) & (zrel < 2.7)

def muro(nombre, eje, banda, otro, otro_rng, delta=0.28):
    """Afina el pico en 'banda' del eje dado, restringido al rango del otro eje."""
    c = u if eje == "u" else v
    o = v if eje == "u" else u
    m = alto & (c > banda[0]) & (c < banda[1]) & (o > otro_rng[0]) & (o < otro_rng[1])
    if m.sum() < 200:
        print("%s: SOLO %d puntos" % (nombre, m.sum())); return None
    pos = float(np.median(c[m]))
    m2 = alto & (np.abs(c - pos) < delta) & (o > otro_rng[0]) & (o < otro_rng[1])
    ext = (float(np.percentile(o[m2], 2)), float(np.percentile(o[m2], 98)))
    # perfil de altura del muro: ¿hasta donde llega macizo?
    mz = (np.abs(c - pos) < delta) & (o > otro_rng[0]) & (o < otro_rng[1]) & (zrel > 0.1)
    zs = zrel[mz]
    ztope = float(np.percentile(zs, 98))
    # ¿banda baja hueca? (ventana: pocos retornos 0.3-1.0)
    n_bajo = int(((zs > 0.3) & (zs < 1.0)).sum()); n_alto2 = int(((zs > 1.75)).sum())
    print("%-14s %s=%6.2f  extension %s: %6.2f..%6.2f (%.1f m)  z98=%.2f  bajo/alto=%d/%d"
          % (nombre, eje, pos, "v" if eje=="u" else "u", ext[0], ext[1], ext[1]-ext[0], ztope, n_bajo, n_alto2))
    return {"eje": eje, "pos": round(pos,3), "ext": [round(ext[0],2), round(ext[1],2)],
            "z_tope": round(ztope,2), "n": int(m2.sum())}

R = {}
print("== SALA A (u -4..4.4, v -4.5..3.5) ==")
R["muroE_A"]    = muro("muroE (tras A)", "u", (-4.2,-3.0), "v", (-4.0, 3.0))
R["particion"]  = muro("particion pta", "u", (3.2, 4.5), "v", (-4.0, 3.1))
R["muroS"]      = muro("muroS (caja)", "v", (-4.5,-3.5), "u", (-3.6, 3.9))
R["muroN_vent"] = muro("muroN ventanas", "v", (2.6, 3.6), "u", (-3.6, 3.6))
print("== SALA B (u 4.3..7.6) ==")
R["fondoB"]     = muro("fondo B", "u", (6.3, 7.6), "v", (-3.5, 3.5))
R["muroS_B"]    = muro("muroS de B", "v", (-4.6,-3.3), "u", (4.3, 7.2))
R["muroN_B"]    = muro("muroN de B", "v", (2.4, 3.8), "u", (4.3, 7.2))

# la CAJA del Summit: mancha alta pegada al muroS dentro de la sala A
if R.get("muroS"):
    vs = R["muroS"]["pos"]
    m = alto & (v > vs) & (v < vs + 1.4) & (u > -3.5) & (u < 3.8)
    if m.sum() > 100:
        h, edges = np.histogram(u[m], bins=np.arange(-3.5, 3.8, 0.15))
        i = int(np.argmax(h))
        cu = edges[i] + 0.075
        mc = m & (np.abs(u - cu) < 1.2)
        print("\ncandidato CAJA: centro u=%.2f  puntos=%d  u_ext=%.2f..%.2f  v_ext=%.2f..%.2f  z98=%.2f"
              % (cu, mc.sum(), np.percentile(u[mc],2), np.percentile(u[mc],98),
                 np.percentile(v[mc],2), np.percentile(v[mc],98), np.percentile(zrel[mc],98)))

def uv2xy(uu, vv):
    return (round(uu*ux - vv*uy, 2), round(uu*uy + vv*ux, 2))
for k, w in R.items():
    if not w: continue
    if w["eje"] == "u":
        w["xy0"] = uv2xy(w["pos"], w["ext"][0]); w["xy1"] = uv2xy(w["pos"], w["ext"][1])
    else:
        w["xy0"] = uv2xy(w["ext"][0], w["pos"]); w["xy1"] = uv2xy(w["ext"][1], w["pos"])
json.dump(R, open("/tmp/paredes_v19.json","w"), indent=1)
print("\nJSON -> /tmp/paredes_v19.json")
