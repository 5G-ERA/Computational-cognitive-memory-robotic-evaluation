#!/usr/bin/env python3
"""Compara el mapa SLAM con su copia _nav y mide el paso más estrecho del vano en cada uno. 17-sep-2026."""
import numpy as np, yaml, os, math, sys
from PIL import Image
E = sys.argv[1] if len(sys.argv) > 1 else "/home/ros/Desktop/extraccion_20260917/mapa"
N = "rbk_2026_09_17_16_23_20"

def carga(n):
    y = yaml.safe_load(open("%s/%s.yaml" % (E, n)))
    img = np.array(Image.open("%s/%s" % (E, y["image"])).convert("L"), dtype=np.float32)
    occ = (255.0 - img) / 255.0
    if y.get("negate", 0):
        occ = 1.0 - occ
    return y, np.flipud(occ > y.get("occupied_thresh", 0.65)), np.flipud(occ < y.get("free_thresh", 0.25))

y0, o0, l0 = carga(N); y1, o1, l1 = carga(N + "_nav")
for nom, y, o, l in (("mapa", y0, o0, l0), ("mapa_nav", y1, o1, l1)):
    print("%-9s %s res %.3f origen %s · ocupadas %d libres %d desconocidas %d"
          % (nom, o.shape, y["resolution"], y["origin"][:2], o.sum(), l.sum(), (~o & ~l).sum()))
print("¿ocupados idénticos? %s · celdas distintas: %d" % (np.array_equal(o0, o1), int((o0 != o1).sum())))

ax, ay, bx, by = 1.830, -3.418, 1.646, -5.006
mx, my = (ax + bx) / 2, (ay + by) / 2
th = math.atan2(by - ay, bx - ax); vx, vy = -math.sin(th), math.cos(th)
for nom, y, o in (("mapa", y0, o0), ("mapa_nav", y1, o1)):
    res = y["resolution"]; ox, oy = y["origin"][0], y["origin"][1]
    mejor = None
    for a in np.arange(-0.6, 0.6, 0.02):
        px, py = mx + a * math.cos(th), my + a * math.sin(th)
        der = izq = 0.0
        for b in np.arange(0, 1.5, 0.01):
            i, j = int((py + b * vy - oy) / res), int((px + b * vx - ox) / res)
            if not (0 <= i < o.shape[0] and 0 <= j < o.shape[1]) or o[i, j]:
                break
            der = b
        for b in np.arange(0, 1.5, 0.01):
            i, j = int((py - b * vy - oy) / res), int((px - b * vx - ox) / res)
            if not (0 <= i < o.shape[0] and 0 <= j < o.shape[1]) or o[i, j]:
                break
            izq = b
        if mejor is None or der + izq < mejor[0]:
            mejor = (der + izq, a, izq, der)
    print("%-9s paso más estrecho %.3f m (a %+.2f m del centro; %.3f izq / %.3f der)" % ((nom,) + mejor)) 
