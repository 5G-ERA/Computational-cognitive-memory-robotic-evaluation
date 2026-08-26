"""¿Las bandas gruesas de 'pared' son deriva o estructura real con fondo?

Corte perpendicular a dos muros: perfil de densidad y altura maxima frente a la
distancia perpendicular. Capas separadas con alturas distintas = estanterias
reales. Mancha continua sin capas = deriva del registro.
Ademas: el mismo corte con SOLO los puntos del techo, que un scan no puede
derivar igual que la pared (geometria distinta)."""
import numpy as np

P = np.load("/home/ros/isaac_ws/nube_g1.npy").astype(np.float64)
fa, fb, fc = -0.0008, 0.0036, -0.0305
zrel = P[:,2] - (fa*P[:,0] + fb*P[:,1] + fc)

# muro NE de la sala (el de la derecha en el plano): banda diagonal que pasa por (4.3, 0.9)
# eje del muro ~135 grados => perpendicular ~45
import math
def corte(cx, cy, perp_deg, ancho=2.0, largo_half=3.0, nombre=""):
    a = math.radians(perp_deg)
    ux, uy = math.cos(a), math.sin(a)          # direccion perpendicular al muro
    vx, vy = -uy, ux                            # a lo largo del muro
    dx, dy = P[:,0]-cx, P[:,1]-cy
    s = dx*ux + dy*uy                           # distancia perpendicular
    t = dx*vx + dy*vy
    m = (np.abs(t) < ancho/2) & (np.abs(s) < largo_half)
    ss, zz = s[m], zrel[m]
    print("\n=== corte %s en (%.1f, %.1f) perp=%d ===  puntos %d" % (nombre, cx, cy, perp_deg, m.sum()))
    print("  dist   n     z_max  z_p90  bandas(z) 0-.5|.5-1|1-1.5|1.5-2|2-2.5|2.5+")
    for b in np.arange(-3.0, 3.0, 0.25):
        mb = (ss>=b)&(ss<b+0.25)
        if mb.sum() < 5:
            continue
        zb = zz[mb]
        cnts = [int(((zb>=lo)&(zb<hi)).sum()) for lo,hi in
                [(0,.5),(.5,1),(1,1.5),(1.5,2),(2,2.5),(2.5,9)]]
        print("  %5.2f %5d  %5.2f  %5.2f  %s" % (b+0.125, mb.sum(), zb.max(),
              np.percentile(zb,90), " ".join("%5d"%c for c in cnts)))

# muro derecho (Este de la sala en frame G1): centro de banda ~(4.3, 0.9), muro corre a 135
corte(4.3, 0.9, 45, nombre="muro E (derecha)")
# muro de la puerta: banda por (-3.9, 1.25) tambien a 135 => perpendicular 45
corte(-3.9, 1.25, 45, nombre="muro de la PUERTA")
# muro superior (Norte): banda por (0.0, 4.6), muro corre a ~45 => perp 135
corte(0.0, 4.6, 135, nombre="muro N (arriba)")
