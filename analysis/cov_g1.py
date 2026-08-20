#!/usr/bin/env python3
"""Campo de cobertura del G1 recomputado OFFLINE sobre los laser_snapshots, y su version buena.

DOS CAMPOS, UNA HISTORIA (medida el 20/21-ago-2026 sobre 8 runs reales + 366 historicos):

  cov_def   -- fraccion de rumbos predichos por el mapa y ausentes (la definicion online).
               Contra el mapa del Summit SATURA en el robot real (mediana por muestra 0.96):
               el solape de celdas entre lo que ve el G1 y ese mapa es ~50-59%. Contra un mapa
               de visibilidad propio mejora pero sigue ruidoso: la fraccion cambia de golpe al
               girar (el sector apunta a otras superficies) y el mobiliario se mueve entre
               semanas (solape jun->ago 58%). Se conserva como evidencia legada.

  cov_missing -- numero de CELDAS del mapa de visibilidad predichas y AUSENTES en >=2 snapshots
               consecutivos. Localizado (sin denominador que baile con el rumbo) y con
               persistencia (el transitorio de un giro dura un snapshot; una perdida de
               cobertura dura toda la aproximacion). Validado con CRISTAL SINTETICO sobre las
               grabaciones reales del 20-ago (borrando los retornos de un tramo de pared
               declarado, el mecanismo del gemelo): deteccion 8/8 runs con K=3, con ~2-3
               eventos falsos por run atribuibles a la deriva de mobiliario del mapa
               historico. CONSECUENCIA DE PROTOCOLO: en datos confirmatorios la referencia se
               congela POR SESION (vueltas de calibracion con G1_LASER_SNAP=0.5), no de un mapa
               de historia larga.

BASE DE EVIDENCIA. Los snapshots guardan op: el mapa de obstaculos ACUMULADO con filtro de
persistencia, recortado a +-2.6 m del robot -- NO el barrido instantaneo del cov_def online.
Por eso aqui el radio es 2.5 m (dentro del recorte en toda direccion) y la variante online se
calibra aparte.

Como modulo: recompute(run, ref) -> [(t, cov_def, cov_n, cov_blind, cov_missing)] y
aplica(run, ref) -> muestras COPIADAS con esos campos sustituidos por el snapshot mas cercano
(el fichero del dataset no se toca: no-reescritura historica). El parametro opcional
borra=(x0,y0,x1,y1) simula cristal eliminando los retornos de ese rectangulo del mundo.

CLI:  python3 analysis/cov_g1.py dataset/visibilidad_g1.json [dia p.ej 20260820]
"""
import glob
import json
import math
import os
import sys

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OC = 0.2
COV_SECT, COV_R, COV_NRAY = 40.0, 2.5, 25
NEAR_BLIND = 0.6
MAX_EDAD = 6.0                 # s: una muestra sin snapshot a menos de esto queda sin campo
# eje de la puerta (el de bitacora.py) para el desglose por zonas
DX, DY = -3.90, 1.25
AX = math.radians(135.0)
UX, UY = math.cos(AX), math.sin(AX)


def celdas(mapa_json):
    m = json.load(open(mapa_json))
    return {(round(p[0] / OC), round(p[1] / OC)) for p in m.get("points", [])}


def _por_rayos(x, y, yaw, vivos, ref):
    """(deficit, n_predichos, ciego, {celda_predicha: ausente?}) con la geometria de cov_def."""
    paso = OC * 0.5
    pred = falt = ciego = 0
    cel = {}
    for k in range(COV_NRAY):
        off = -COV_SECT + (2.0 * COV_SECT * k / max(1, COV_NRAY - 1))
        a_ = math.radians(yaw + off)
        ca, sa = math.cos(a_), math.sin(a_)
        r_map = r_live = None
        c_map = None
        r = paso
        while r <= COV_R:
            c = (int(round((x + ca * r) / OC)), int(round((y + sa * r) / OC)))
            if r_map is None and c in ref:
                r_map, c_map = r, c
            if r_live is None and c in vivos:
                r_live = r
            if r_map is not None and r_live is not None:
                break
            r += paso
        if r_map is None:
            continue
        pred += 1
        if r_map < NEAR_BLIND:
            ciego += 1
            continue
        falta = r_live is None or r_live > r_map + OC
        if falta:
            falt += 1
        # si varios rayos tocan la misma celda, VERLA una vez basta
        cel[c_map] = cel.get(c_map, True) and falta
    if not pred:
        return (None, 0, None, cel)
    return (round(falt / pred, 3), pred, round(ciego / pred, 3), cel)


def recompute(d, ref, borra=None):
    out = []
    prev = {}
    for s in d.get("laser_snapshots") or []:
        if s.get("x") is None or s.get("yaw") is None:
            continue
        pts = s.get("pts") or []
        if borra:
            x0, y0, x1, y1 = borra
            pts = [p for p in pts if not (x0 <= p[0] <= x1 and y0 <= p[1] <= y1)]
        vivos = {(round(p[0] / OC), round(p[1] / OC)) for p in pts}
        cd, cn, cb, cel = _por_rayos(s["x"], s["y"], s["yaw"], vivos, ref)
        missing = sum(1 for c, falta in cel.items() if falta and prev.get(c) is True)
        prev = cel
        out.append((s["t"], cd, cn, cb, missing))
    return out


def aplica(d, ref, borra=None, max_edad=MAX_EDAD):
    """Muestras del run (COPIAS) con cov_def/cov_n/cov_blind/cov_missing del snapshot
    recomputado mas cercano. El run en disco no se toca."""
    serie = recompute(d, ref, borra)
    out = []
    for m in d.get("samples") or []:
        m2 = dict(m)
        t = m.get("t")
        mejor = None
        if t is not None and serie:
            mejor = min(serie, key=lambda r: abs(r[0] - t))
            if abs(mejor[0] - t) > max_edad:
                mejor = None
        if mejor is not None and mejor[1] is not None:
            m2["cov_def"], m2["cov_n"], m2["cov_blind"] = mejor[1], mejor[2], mejor[3]
            m2["cov_missing"] = mejor[4]
        else:
            m2["cov_def"] = None
            m2["cov_n"] = 0
            m2["cov_blind"] = None
            m2["cov_missing"] = None
        out.append(m2)
    return out


def zona(x, y):
    s = abs((x - DX) * UX + (y - DY) * UY)
    return "vano" if s <= 0.5 else ("cerca" if s <= 1.5 else ("media" if s <= 3.0 else "lejos"))


def _describe(vals):
    vs = sorted(v for v in vals if v is not None)
    if not vs:
        return "sin datos"
    return "n=%d  mediana %.3f  p90 %.3f  max %.3f" % (
        len(vs), vs[len(vs) // 2], vs[int(0.9 * (len(vs) - 1))], vs[-1])


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    ref = celdas(sys.argv[1])
    dia = sys.argv[2] if len(sys.argv) > 2 else "20260820"
    fs = sorted(glob.glob(os.path.join(RAIZ, "dataset", dia + "*_ours_[AB].json")))
    print("mapa: %s (%d celdas)   runs %s: %d" % (sys.argv[1], len(ref), dia, len(fs)))
    defs, miss = [], []
    for f in fs:
        d = json.load(open(f))
        for t, cd, cn, cb, mg in recompute(d, ref):
            if cd is not None:
                defs.append(cd)
            miss.append(mg)
    print("cov_def     %s" % _describe(defs))
    print("cov_missing %s" % _describe(miss))


if __name__ == "__main__":
    main()
