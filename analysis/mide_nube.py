"""Que da de si la nube 3D del Summit MAS ALLA de alturas por celda de 20 cm.

Preguntas medibles:
  1. ¿Cubre paredes de forma CONTINUA donde nuestros bloques van a saltos?
  2. ¿Hay techo real (z alto)?
  3. ¿Los muebles salen como grupos compactos (una caja orientada por mueble)?
  4. ¿Cuanto entorno hay fuera del recinto que ya modelamos?
"""
import json, math
import numpy as np
from collections import defaultdict

P = np.load("/home/ros/isaac_ws/nube_g1.npy").astype(np.float64)
print("puntos en frame G1:", len(P))
print("extension x: %.2f .. %.2f | y: %.2f .. %.2f | z: %.2f .. %.2f"
      % (P[:,0].min(), P[:,0].max(), P[:,1].min(), P[:,1].max(), P[:,2].min(), P[:,2].max()))

# suelo corregido como en office3d v18
fa, fb, fc = -0.0008, 0.0036, -0.0305
zrel = P[:,2] - (fa*P[:,0] + fb*P[:,1] + fc)

print("\n--- reparto vertical (z relativo al suelo) ---")
bandas = [(-1,0.05,"suelo"), (0.05,0.5,"bajo (patas, zocalos)"), (0.5,1.2,"mueble (mesas, sillas)"),
          (1.2,1.8,"alto (estantes, monitores)"), (1.8,2.4,"pared alta"), (2.4,3.0,"techo bajo?"),
          (3.0,5.0,"techo/estructura"), (5.0,99,"fuera de rango")]
for lo,hi,e in bandas:
    m = ((zrel>=lo)&(zrel<hi)).sum()
    print("  %-24s %7d  (%.1f%%)" % (e, m, 100.0*m/len(P)))

# ¿continuidad de pared? rejilla FINA de 5 cm solo con puntos de altura de pared
F = 0.05
wall = P[(zrel>1.6)&(zrel<2.6)]
cw = set(zip(np.round(wall[:,0]/F).astype(int), np.round(wall[:,1]/F).astype(int)))
print("\nceldas de 5 cm con retorno a altura de pared (1.6-2.6):", len(cw))

# nuestras celdas de pared actuales (20 cm) del nav_map
nav = json.load(open("/home/ros/isaac_ws/nav_map.json"))
pared20 = {(int(c[0]),int(c[1])) for c in nav.get("walls", nav.get("pared", []))} if isinstance(nav, dict) else set()
print("claves del nav_map:", list(nav.keys()) if isinstance(nav, dict) else type(nav))

# techo: densidad de z>2.6
techo = P[zrel>2.6]
if len(techo):
    ct = set(zip(np.round(techo[:,0]/0.2).astype(int), np.round(techo[:,1]/0.2).astype(int)))
    print("techo: %d puntos sobre %d celdas de 20cm (z medio %.2f)" % (len(techo), len(ct), techo[:,2].mean()))

# muebles: componentes conexas de celdas 10cm con altura dominante 0.4-1.5
F2 = 0.10
mid = P[(zrel>0.35)&(zrel<1.55)]
cm = defaultdict(int)
for x,y in zip(np.round(mid[:,0]/F2).astype(int), np.round(mid[:,1]/F2).astype(int)):
    cm[(x,y)] += 1
celdas_m = {c for c,n in cm.items() if n>=4}
vis = set(); comps = []
for c in celdas_m:
    if c in vis: continue
    pila=[c]; vis.add(c); comp=[]
    while pila:
        a=pila.pop(); comp.append(a)
        for dx in (-1,0,1):
            for dy in (-1,0,1):
                b=(a[0]+dx,a[1]+dy)
                if b in celdas_m and b not in vis:
                    vis.add(b); pila.append(b)
    comps.append(comp)
comps.sort(key=len, reverse=True)
print("\nmuebles: %d componentes (celdas 10cm, >=4 puntos); las 12 mayores:" % len(comps))
for comp in comps[:12]:
    xs=[c[0]*F2 for c in comp]; ys=[c[1]*F2 for c in comp]
    print("  centro (%6.2f,%6.2f)  caja %4.1f x %4.1f m  celdas %d"
          % (np.mean(xs), np.mean(ys), max(xs)-min(xs)+F2, max(ys)-min(ys)+F2, len(comp)))

# extension util fuera del recinto modelado (recinto aprox x -8..3, y -3..6 en frame G1)
fuera = P[(P[:,0]<-8.5)|(P[:,0]>3.5)|(P[:,1]<-3.5)|(P[:,1]>6.5)]
print("\npuntos FUERA del recinto ya modelado: %d (%.1f%%)" % (len(fuera), 100.0*len(fuera)/len(P)))
