"""Geometria espuria segun la CREENCIA DEL PROPIO G1 (no ray-march: los snapshots son
el mapa de obstaculos ACUMULADO, no el barrido instantaneo -- mi version anterior era invalida).

Regla: una celda que el G1 tuvo cerca muchas veces, desde varias direcciones distintas, y en la
que su laser NUNCA puso obstaculo, no esta ocupada -- por mucho que lo diga el mapa del Summit.
Es la misma logica que cov_missing, aplicada al reves para depurar el mapa.

Guarda contra oclusion: se exige haber observado la celda desde >= MIN_OCT octantes distintos,
para que una pared que la tape desde un lado no la declare libre.
"""
import json, glob, math, os
import collections

OC = 0.2
RADIO = 2.4                    # el mapa acumulado del robot se recorta a +-2.6 m
MIN_OPP = int(os.environ.get("MIN_OPP", "100"))
MIN_OCT = int(os.environ.get("MIN_OCT", "3"))
MAX_HIT = float(os.environ.get("MAX_HIT", "0.01"))   # fraccion maxima de veces con obstaculo

opp = collections.Counter()
hit = collections.Counter()
octantes = collections.defaultdict(set)
n = 0
for rf in sorted(glob.glob("dataset/2026*_ours_[AB].json")):
    try:
        d = json.load(open(rf))
    except Exception:
        continue
    if d.get("sim_id"):
        continue
    for s in d.get("laser_snapshots") or []:
        if s.get("x") is None:
            continue
        n += 1
        sx, sy = s["x"], s["y"]
        vistos = {(round(p[0]/OC), round(p[1]/OC)) for p in (s.get("pts") or [])}
        rc = int(RADIO / OC)
        cx0, cy0 = round(sx/OC), round(sy/OC)
        for dx in range(-rc, rc + 1):
            for dy in range(-rc, rc + 1):
                c = (cx0 + dx, cy0 + dy)
                r = math.hypot(c[0]*OC - sx, c[1]*OC - sy)
                if r < 0.3 or r > RADIO:
                    continue
                opp[c] += 1
                octantes[c].add(int(((math.degrees(math.atan2(c[1]*OC - sy, c[0]*OC - sx)) + 360) % 360) // 45))
                if c in vistos:
                    hit[c] += 1

print("snapshots:", n, "| celdas con oportunidad:", len(opp))
pared = {(round(p[0]/OC), round(p[1]/OC))
         for p in json.load(open("/home/ros/isaac_ws/ref_map_g1.json"))["points"]}
nav = {(int(c[0]), int(c[1]))
       for c in json.load(open("/home/ros/isaac_ws/nav_map.json")).get("cells", [])}

esp = set()
for c in (pared | nav):
    o = opp.get(c, 0)
    if o < MIN_OPP or len(octantes.get(c, ())) < MIN_OCT:
        continue
    if hit.get(c, 0) / o <= MAX_HIT:
        esp.add(c)
print("MIN_OPP=%d MIN_OCT=%d MAX_HIT=%.2f -> espurias: %d de pared, %d de mueble" % (
    MIN_OPP, MIN_OCT, MAX_HIT, len(esp & pared), len(esp & nav)))
json.dump({"cells": [list(c) for c in sorted(esp)], "OCELL": OC, "min_opp": MIN_OPP,
           "min_oct": MIN_OCT, "max_hit": MAX_HIT, "snapshots": n},
          open("/home/ros/isaac_ws/celdas_espurias.json", "w"))
