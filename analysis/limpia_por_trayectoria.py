"""Celdas que el robot REAL atraveso: no pueden estar ocupadas.

El mapa reconstruido arrastra ocupacion espuria (fantasmas de la nube acumulada y ruido del
mapa 2D). Hay un arbitro incontestable: las trayectorias reales. Si el robot paso fisicamente
por una celda, esa celda esta libre. Se marcan todas las celdas a menos de RADIO del centro de
cualquier pose real registrada, en 133 runs.
"""
import json, glob, math, collections

OC = 0.2
# RADIO = medio ancho REAL del cuerpo, sin inflar: con 0.22 se borraban hasta las jambas (el
# robot las rozo de verdad) y el vano quedaba en 1.75 m contra los 1.13 reales.
RADIO = float(__import__("os").environ.get("RADIO", "0.16"))
MIN_RUNS = int(__import__("os").environ.get("MIN_RUNS", "3"))   # varios runs independientes

import collections as _col
por_celda = _col.defaultdict(set)
poses = 0
for rf in sorted(glob.glob("dataset/2026*_ours_[AB].json")):
    try:
        d = json.load(open(rf))
    except Exception:
        continue
    if d.get("sim_id"):
        continue
    for m in d.get("samples") or []:
        if m.get("x") is None:
            continue
        poses += 1
        cx, cy = m["x"] / OC, m["y"] / OC
        r_c = int(RADIO / OC) + 1
        for dx in range(-r_c, r_c + 1):
            for dy in range(-r_c, r_c + 1):
                c = (round(cx) + dx, round(cy) + dy)
                if math.hypot(c[0]*OC - m["x"], c[1]*OC - m["y"]) <= RADIO:
                    por_celda[c].add(rf)

libres = {c for c, runs in por_celda.items() if len(runs) >= MIN_RUNS}
print("poses reales: %d | celdas tocadas %d | transitadas en >=%d runs: %d" % (
    poses, len(por_celda), MIN_RUNS, len(libres)))
json.dump({"cells": [list(c) for c in sorted(libres)], "OCELL": OC, "radio": RADIO,
           "min_runs": MIN_RUNS, "poses": poses},
          open("/home/ros/isaac_ws/celdas_libres.json", "w"))

# cuanto limpia
pared = {(round(p[0]/OC), round(p[1]/OC))
         for p in json.load(open("/home/ros/isaac_ws/ref_map_g1.json"))["points"]}
nav = {(int(c[0]), int(c[1])) for c in json.load(open("/home/ros/isaac_ws/nav_map.json")).get("cells", [])}
print("choques con el mapa: pared %d de %d | mueble %d de %d" % (
    len(pared & libres), len(pared), len(nav & libres), len(nav)))
# y en el vano
DX, DY = -3.90, 1.25
cerca_puerta = {c for c in (pared | nav)
                if math.hypot(c[0]*OC - DX, c[1]*OC - DY) < 1.2}
print("celdas ocupadas a <1.2 m de la puerta: %d, de ellas transitadas: %d" % (
    len(cerca_puerta), len(cerca_puerta & libres)))
