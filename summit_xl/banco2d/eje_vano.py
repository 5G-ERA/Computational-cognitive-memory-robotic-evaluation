#!/usr/bin/env python3
"""¿Por dónde pasa de verdad el centro del vano, y dónde lo ponen las marcas del robot? 17-sep-2026.

Las marcas `vano_antes` y `vano_despues` se tomaron con el robot y AMCL. Con 6,45 cm de margen por lado, que su eje
esté descentrado unos centímetros se come el hueco. Esto mide, sobre el mapa, el centro real del paso libre.
"""
import math, sys, numpy as np, yaml
from PIL import Image

y = yaml.safe_load(open(sys.argv[1]))
import os
o = np.flipud((255.0 - np.array(Image.open(os.path.join(os.path.dirname(sys.argv[1]), y["image"])).convert("L"), dtype=np.float32)) / 255.0 > 0.65)
res = float(y["resolution"]); ox, oy = float(y["origin"][0]), float(y["origin"][1])
A = (1.830, -3.418); D = (1.646, -5.006)
th = math.atan2(D[1] - A[1], D[0] - A[0]); ux, uy = math.cos(th), math.sin(th); vx, vy = -uy, ux
print("eje de las marcas: %.1f°" % math.degrees(th))
print("  s(m)   libre_izq  libre_der   ancho   centro respecto al eje")
desv = []
for s in np.arange(-0.4, 0.45, 0.1):
    cx, cy = (A[0] + D[0]) / 2 + s * ux, (A[1] + D[1]) / 2 + s * uy
    der = izq = 0.0
    for b in np.arange(0, 1.2, 0.005):
        i, j = int((cy + b * vy - oy) / res), int((cx + b * vx - ox) / res)
        if not (0 <= i < o.shape[0] and 0 <= j < o.shape[1]) or o[i, j]: break
        der = b
    for b in np.arange(0, 1.2, 0.005):
        i, j = int((cy - b * vy - oy) / res), int((cx - b * vx - ox) / res)
        if not (0 <= i < o.shape[0] and 0 <= j < o.shape[1]) or o[i, j]: break
        izq = b
    c = (der - izq) / 2
    desv.append(c)
    print("  %+5.2f   %6.3f     %6.3f    %6.3f   %+6.1f cm" % (s, izq, der, izq + der, 100 * c))
print("\ndesviación media del centro real respecto al eje de las marcas: %+.1f cm" % (100 * float(np.mean(desv))))
print("margen por lado si se va por el eje de las marcas: %.1f cm · si se va por el centro real: %.1f cm"
      % (100 * (min(min(desv) + 0, 0) + 0), 0))
