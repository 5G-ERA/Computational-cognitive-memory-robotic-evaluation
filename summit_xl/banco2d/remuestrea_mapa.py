#!/usr/bin/env python3
"""Remuestrea un mapa de Nav2 a celdas más finas SIN cambiar la geometría (vecino más próximo). 17-sep-2026.

Por qué: la capa estática impone al costmap la resolución del MAPA, así que `resolution: 0.02` en los parámetros no
sirve de nada con un mapa de 5 cm. A 5 cm el vano entero cae en la banda inscrita y NavFn no encuentra camino.

  python3 remuestrea_mapa.py ORIGEN.yaml FACTOR DESTINO.yaml
"""
import os, sys, yaml
from PIL import Image

src, factor, dst = sys.argv[1], int(sys.argv[2]), sys.argv[3]
y = yaml.safe_load(open(src))
img = Image.open(os.path.join(os.path.dirname(src), y["image"]))
fino = img.resize((img.width * factor, img.height * factor), Image.NEAREST)
png = os.path.splitext(os.path.basename(dst))[0] + ".png"
fino.save(os.path.join(os.path.dirname(dst) or ".", png))
y["image"] = png; y["resolution"] = round(float(y["resolution"]) / factor, 6)
yaml.safe_dump(y, open(dst, "w"), sort_keys=False)
print("%s → %s: %dx%d a %.4f m (origen %s)" % (os.path.basename(src), png, fino.width, fino.height, y["resolution"], y["origin"][:2]))
