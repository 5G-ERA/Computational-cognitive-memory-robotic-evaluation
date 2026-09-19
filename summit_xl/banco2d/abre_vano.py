#!/usr/bin/env python3
"""Corrige la anchura del vano en un mapa de Nav2, dejándolo en la medida de la cinta métrica. 17-sep-2026.

Por qué: el mapa de SLAM del 17-sep da un paso de 0,640 m donde la cinta mide 0,742 m jamba a jamba — el barrido
engorda las jambas ~5 cm por lado. Con 0,640 el robot (0,613) tiene 1,35 cm por lado y Nav2 no planifica ni con
padding 0; con la anchura real tiene 6,45 cm por lado. Esto **no** inventa espacio: lo devuelve a lo medido.

  python3 abre_vano.py ORIGEN.yaml DESTINO.yaml X Y YAW ANCHO [PROFUNDIDAD]
      X Y      centro del vano (m, marco del mapa)      YAW    eje del vano en grados
      ANCHO    anchura libre a dejar (m)                PROF   longitud del tramo a rehacer (m, por defecto 0,5)
"""
import math, os, sys, numpy as np, yaml
from PIL import Image

src, dst, px, py, yaw, ancho = sys.argv[1], sys.argv[2], *[float(v) for v in sys.argv[3:7]]
prof = float(sys.argv[7]) if len(sys.argv) > 7 else 0.5
y = yaml.safe_load(open(src))
img = Image.open(os.path.join(os.path.dirname(src), y["image"])).convert("L")
a = np.array(img)
res = float(y["resolution"]); ox, oy = float(y["origin"][0]), float(y["origin"][1])
alto = a.shape[0]
LIBRE, OCUP = 254, 0          # en un PGM/PNG de Nav2: claro = libre, oscuro = ocupado
th = math.radians(yaw); ux, uy = math.cos(th), math.sin(th); vx, vy = -uy, ux
n = 0
for s in np.arange(-prof / 2, prof / 2, res / 3):
    for t in np.arange(-1.0, 1.0, res / 3):
        x, yy = px + s * ux + t * vx, py + s * uy + t * vy
        j = int((x - ox) / res); i = alto - 1 - int((yy - oy) / res)
        if not (0 <= i < a.shape[0] and 0 <= j < a.shape[1]):
            continue
        jamba = abs(t) >= ancho / 2 and abs(t) <= ancho / 2 + 0.10 and abs(s) <= 0.06
        nuevo = OCUP if jamba else (LIBRE if abs(t) < ancho / 2 else None)
        if nuevo is not None and a[i, j] != nuevo:
            a[i, j] = nuevo; n += 1
png = os.path.splitext(os.path.basename(dst))[0] + ".png"
Image.fromarray(a).save(os.path.join(os.path.dirname(dst) or ".", png))
y["image"] = png
yaml.safe_dump(y, open(dst, "w"), sort_keys=False)
print("vano reabierto a %.3f m en (%.2f, %.2f) eje %.1f°: %d celdas cambiadas → %s" % (ancho, px, py, yaw, n, png))
