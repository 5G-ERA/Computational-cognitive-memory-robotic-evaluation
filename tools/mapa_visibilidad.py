#!/usr/bin/env python3
"""Construye el mapa de VISIBILIDAD propio del G1 desde los laser_snapshots del dataset.

POR QUE. El campo cov_def pregunta "veo lo que el mapa predice?", pero el mapa de referencia
(summit/ref_map_g1.json) lo levanto OTRO robot con el laser mejor montado: el solape de celdas
con lo que el G1 ve es ~50%, asi que en el robot real el deficit por muestra sale ~0.9 SIEMPRE
y el campo satura -- C3 y C4 quedan clavadas y el contraste C4-C3 no mide nada. La pregunta
correcta es "veo lo que YO suelo ver desde aqui", y eso exige una referencia del PROPIO G1.

BASE DE EVIDENCIA -- IMPORTANTE. Los snapshots guardan 'op': el mapa de obstaculos ACUMULADO
con filtro de persistencia, recortado a +-2.6 m del robot. NO es el barrido instantaneo que usa
el cov_def online. Por eso este mapa y el campo de replay que se puntua contra el (analysis/
cov_g1.py) se declaran sobre ESA base: creencia acumulada, radio 2.5 m (dentro del recorte en
toda direccion). El campo online instantaneo es otra variante del mismo campo y se calibra
aparte.

COMO. Para cada snapshot se replica la geometria de cov_def (sector +-COV_SECT, radio COV_R):
las celdas del sector son OPORTUNIDADES; las que ademas tienen retorno son ACIERTOS. La
estadistica es POR SNAPSHOT -- la misma unidad que luego puntua --, con dos guardas: la celda
debe verse en >= MIN_RUNS runs distintos (contra artefactos de un dia) y tener >= MIN_OPP
oportunidades. Entra al mapa si aciertos/oportunidades >= MIN_RATIO.

La oclusion no se modela y no hace falta: una celda tras una pared acumula oportunidades y
ningun acierto, y su ratio la excluye sola. El espacio libre tampoco entra.

Uso:
    python3 tools/mapa_visibilidad.py --hasta 20260819              # held-out del dia 20
    python3 tools/mapa_visibilidad.py --hasta 20260819 --barrido    # barre MIN_RATIO y no escribe
"""
import glob
import json
import math
import os
import sys
import collections

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OC = 0.2
COV_SECT = 40.0
COV_R = 2.5                   # < 2.6 del recorte de 'op' en toda direccion
MIN_OPP = 30                  # oportunidades minimas (snapshots) para juzgar una celda
MIN_RUNS = 5                  # ...y vista en al menos estos runs distintos
MIN_RATIO = 0.65              # fraccion de snapshots-con-oportunidad que deben verla


def celdas_sector(x, y, yaw):
    out = set()
    r_c = int(COV_R / OC) + 1
    cx, cy = x / OC, y / OC
    for dx in range(-r_c, r_c + 1):
        for dy in range(-r_c, r_c + 1):
            px, py = (round(cx) + dx) * OC - x, (round(cy) + dy) * OC - y
            r = math.hypot(px, py)
            if r < OC or r > COV_R:
                continue
            da = (math.degrees(math.atan2(py, px)) - yaw + 540.0) % 360.0 - 180.0
            if abs(da) <= COV_SECT:
                out.add((round(cx) + dx, round(cy) + dy))
    return out


def acumula(fs):
    opp = collections.Counter()               # celda -> snapshots con oportunidad
    hit = collections.Counter()               # celda -> snapshots con retorno
    runs_hit = collections.defaultdict(set)   # celda -> runs distintos que la vieron
    usados = 0
    for f in fs:
        try:
            d = json.load(open(f))
        except Exception:
            continue
        snaps = d.get("laser_snapshots") or []
        if not snaps:
            continue
        for s in snaps:
            if s.get("x") is None or s.get("yaw") is None:
                continue
            sector = celdas_sector(s["x"], s["y"], s["yaw"])
            vivos = {(round(p[0] / OC), round(p[1] / OC)) for p in (s.get("pts") or [])}
            for c in sector:
                opp[c] += 1
            for c in sector & vivos:
                hit[c] += 1
                runs_hit[c].add(f)
        usados += 1
    return opp, hit, runs_hit, usados


def main():
    desde, hasta = "20260701", "20269999"
    for i, a in enumerate(sys.argv):
        if a == "--desde" and i + 1 < len(sys.argv):
            desde = sys.argv[i + 1]
        if a == "--hasta" and i + 1 < len(sys.argv):
            hasta = sys.argv[i + 1]
    barrido = "--barrido" in sys.argv

    fs = sorted(glob.glob(os.path.join(RAIZ, "dataset", "2026*_ours_[AB].json")))
    fs = [f for f in fs if desde <= os.path.basename(f)[:8] <= hasta]
    print("runs de entrada: %d  (%s .. %s)" % (len(fs), desde, hasta))
    opp, hit, runs_hit, usados = acumula(fs)
    print("runs con snapshots utilizables: %d" % usados)

    def puntos_con(ratio):
        return [[round(c[0] * OC, 2), round(c[1] * OC, 2)]
                for c, n in opp.items()
                if n >= MIN_OPP and len(runs_hit[c]) >= MIN_RUNS and hit[c] / n >= ratio]

    if barrido:
        for r in (0.5, 0.65, 0.8, 0.9):
            print("  MIN_RATIO=%.2f -> %d celdas" % (r, len(puntos_con(r))))
        return

    puntos = puntos_con(MIN_RATIO)
    juzgadas = sum(1 for c, n in opp.items() if n >= MIN_OPP and len(runs_hit[c]) >= MIN_RUNS)
    print("celdas juzgadas: %d   en el mapa (ratio>=%.2f): %d" % (juzgadas, MIN_RATIO, len(puntos)))
    out = {"frame": "g1 (map, laser_snapshots)",
           "src": "visibilidad empirica G1 sobre 'op' (mapa acumulado, recorte 2.6m): "
                  "%d runs %s..%s, sector +-%g deg r<=%gm, por SNAPSHOT, "
                  "opp>=%d runs>=%d ratio>=%g" % (usados, desde, hasta, COV_SECT, COV_R,
                                                  MIN_OPP, MIN_RUNS, MIN_RATIO),
           "OCELL": OC, "npts": len(puntos), "points": sorted(puntos)}
    dst = os.path.join(RAIZ, "dataset", "visibilidad_g1.json")
    json.dump(out, open(dst, "w"))
    print("escrito %s" % os.path.relpath(dst, RAIZ))


if __name__ == "__main__":
    main()
