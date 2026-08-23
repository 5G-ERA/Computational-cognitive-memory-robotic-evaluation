"""Extrae las LINEAS de pared de la nube Summit (frame G1).

La sala esta girada ~45: el eje de cruce de la puerta es 135, asi que las
paredes corren a 45 (constantes en u) y a 135 (constantes en v), con
  u = proyeccion sobre el eje de cruce (135)
  v = proyeccion perpendicular (45)
Picos de densidad de puntos ALTOS (z>1.75) en cada eje = caras de pared.
El sector del artefacto de la ventana (Z6, tras el muro N) se EXCLUYE."""
import math
import numpy as np

P = np.load("/home/ros/isaac_ws/nube_g1.npy").astype(np.float64)
fa, fb, fc = -0.0008, 0.0036, -0.0305
zrel = P[:,2] - (fa*P[:,0] + fb*P[:,1] + fc)

A = math.radians(135.0)
ux, uy = math.cos(A), math.sin(A)          # eje de cruce A->B
u = P[:,0]*ux + P[:,1]*uy
v = -P[:,0]*uy + P[:,1]*ux

alto = (zrel > 1.75) & (zrel < 2.7)

def perfil(coord, m, e, lo, hi, paso=0.05):
    print("\n--- picos en %s (solo z>1.75) ---" % e)
    h, edges = np.histogram(coord[m], bins=np.arange(lo, hi, paso))
    # picos locales con soporte
    for i in range(2, len(h)-2):
        if h[i] > 400 and h[i] >= h[i-1] and h[i] >= h[i+1] and h[i] > h[i-2] and h[i] > h[i+2]:
            print("  %s = %6.2f   n=%5d" % (e, edges[i]+paso/2, h[i]))

# pose A en (u,v): para orientarse
for nom,(x,y) in [("A",(0.99,0.57)), ("puerta",(-3.90,1.25)), ("B",(-4.71,2.84))]:
    print("%s: u=%.2f v=%.2f" % (nom, x*ux+y*uy, -x*uy+y*ux))

# excluir el sector del artefacto Z6: tras el muro N == v grande (medir primero sin excluir)
perfil(u, alto, "u", -10, 10)
perfil(v, alto, "v", -10, 10)
