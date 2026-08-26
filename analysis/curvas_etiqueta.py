#!/usr/bin/env python3
"""Curvas de deteccion POR ETIQUETA desde las detecciones reales de navegacion libre.

POR QUE. La CURVA del emulador esta medida solo para 'chair' (tandas escenificadas, unico
denominador limpio). Pero el laboratorio tiene sofa y nevera fijos, y el servidor real los
detecta con otra tasa y otro alcance (68% de los 'couch' reales caen MAS ALLA del RMAX=4.5
del emulador). Para escenificar T5/T6 con esos objetos hace falta su curva.

COMO -- el problema del denominador. En navegacion libre solo se graban los ACIERTOS
(la clave dets no existe si no hubo deteccion): no se sabe cuantos fotogramas tenian el
objeto a la vista. Se estima GEOMETRICAMENTE: los muebles son fijos, asi que
  1. cada acierto se retroproyecta a mapa con la pose (el signo del rumbo no esta
     documentado: se prueban ambos y se queda el de clusters mas prietos);
  2. los aciertos por etiqueta se agrupan (histograma 0.2 m + picos) -> posicion de cada
     mueble;
  3. OPORTUNIDAD = muestra cuya pose tiene el cluster dentro del cono (|brg|<=HFOV,
     r<=11 m), SIN modelo de oclusion (declarado);
  4. tasa(bin de rango) = aciertos asignados al cluster / oportunidades.
La tasa asi medida es la de CANAL CARGADO (navegacion libre), NO el techo escenificado:
comparable a CURVA x ATENUACION, no a CURVA. Por eso lo que se exporta al emulador es el
FACTOR RELATIVO a la silla en el mismo pie: F_lab(r) = tasa_lab(r) / tasa_chair(r), y el
alcance real observado. Los errores sistematicos del denominador (oclusion, cuantizacion
del rango) pegan igual en numerador de las dos etiquetas y se cancelan en el cociente
en primer orden.

'person' NO se ajusta: las personas se mueven y no tienen cluster fijo (declarado).

Ancla de cordura: la fraccion global de muestras con deteccion debe salir ~16% (medida
real conocida) y la tasa de chair debe casar con CURVA x ATENUACION del emulador.
"""
import collections
import glob
import json
import math
import os
import sys

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HFOV = 28.07
RMAX_FIT = 11.0
GRID = 0.2
R_ASIGNA = 1.5          # un acierto pertenece a un cluster si cae a <= esto de su pico
MIN_CLUSTER = 60        # aciertos minimos para aceptar un pico como mueble
BINS = [(0.0, 1.0), (1.0, 2.0), (2.0, 3.0), (3.0, 4.5), (4.5, 6.5), (6.5, 8.5), (8.5, 11.0)]
ETIQUETAS = ("chair", "couch", "refrigerator")


def carga():
    runs = []
    for f in sorted(glob.glob(os.path.join(RAIZ, "dataset", "2026*_ours_[AB].json"))):
        try:
            d = json.load(open(f))
        except Exception:
            continue
        if d.get("sim_id"):
            continue
        ss = d.get("samples", [])
        if not any(m.get("dets") for m in ss):
            continue
        poses, hits = [], []
        for m in ss:
            x, y, yaw = m.get("x"), m.get("y"), m.get("yaw")
            if not all(isinstance(v, (int, float)) for v in (x, y, yaw)):
                continue
            poses.append((x, y, yaw))
            for det in (m.get("dets") or []):
                lab, conf, brg, rng = det[0], det[1], det[2], det[3]
                if lab in ETIQUETAS and isinstance(rng, (int, float)) and \
                   isinstance(brg, (int, float)) and 0.1 <= rng <= RMAX_FIT:
                    hits.append((lab, conf, brg, rng, x, y, yaw))
        runs.append((os.path.basename(f), poses, hits))
    return runs


def proyecta(hits, signo):
    out = collections.defaultdict(list)
    for lab, conf, brg, rng, x, y, yaw in hits:
        a = math.radians(yaw + signo * brg)
        out[lab].append((x + rng * math.cos(a), y + rng * math.sin(a), conf, rng, brg))
    return out


def picos(puntos):
    """histograma en rejilla GRID -> picos locales golosos, cada uno se lleva sus celdas a <R_ASIGNA"""
    h = collections.Counter((round(px / GRID), round(py / GRID)) for px, py, *_ in puntos)
    usadas, out = set(), []
    for celda, n in h.most_common():
        if celda in usadas:
            continue
        cx, cy = celda[0] * GRID, celda[1] * GRID
        propios = [(px, py) for px, py, *_ in puntos if math.hypot(px - cx, py - cy) <= R_ASIGNA]
        if len(propios) < MIN_CLUSTER:
            continue
        mx = sum(p[0] for p in propios) / len(propios)
        my = sum(p[1] for p in propios) / len(propios)
        out.append((mx, my, len(propios)))
        for c2 in list(h):
            if math.hypot(c2[0] * GRID - mx, c2[1] * GRID - my) <= R_ASIGNA * 1.6:
                usadas.add(c2)
    return out


def dispersion(por_lab):
    tot, n = 0.0, 0
    for lab, pts in por_lab.items():
        for mx, my, _ in picos(pts):
            propios = [(px, py) for px, py, *_ in pts if math.hypot(px - mx, py - my) <= R_ASIGNA]
            tot += sum(math.hypot(px - mx, py - my) ** 2 for px, py in propios)
            n += len(propios)
    return tot / max(1, n)


def main():
    runs = carga()
    todos_hits = [h for _, _, hits in runs for h in hits]
    tot_muestras = sum(len(p) for _, p, _ in runs)
    con_det = 0
    for _, poses, hits in runs:
        # muestras con >=1 det: aprox por parejas pose-hit del mismo run (los hits llevan la pose)
        con_det += len({(x, y) for _, _, _, _, x, y, _ in hits})
    print("runs reales con dets: %d | muestras %d | hits usables %d" % (len(runs), tot_muestras, len(todos_hits)))
    print("ancla de cordura (~16%% esperado): %.1f%% de muestras con deteccion" % (100.0 * con_det / tot_muestras))

    # signo del rumbo por compacidad de clusters
    var_pos = dispersion(proyecta(todos_hits, +1.0))
    var_neg = dispersion(proyecta(todos_hits, -1.0))
    signo = +1.0 if var_pos <= var_neg else -1.0
    print("signo del rumbo: %+d (var +1: %.3f, var -1: %.3f)" % (signo, var_pos, var_neg))
    por_lab = proyecta(todos_hits, signo)

    resultado = {"signo_rumbo": signo, "base": "navegacion libre, canal cargado",
                 "denominador": "cono FOV geometrico sin oclusion (declarado)",
                 "muebles": {}, "curvas": {}, "factores": {}, "conf": {}}

    tasas = {}
    for lab in ETIQUETAS:
        pts = por_lab.get(lab, [])
        ps = picos(pts)
        print("\n%s: %d hits proyectados, %d cluster(s):" % (lab, len(pts), len(ps)))
        for mx, my, n in ps:
            print("   (%.2f, %.2f)  n=%d" % (mx, my, n))
        resultado["muebles"][lab] = [[round(mx, 2), round(my, 2), n] for mx, my, n in ps]
        if not ps:
            continue
        # oportunidades y aciertos por bin
        opp = collections.Counter()
        hit = collections.Counter()
        confs = collections.defaultdict(list)
        for _, poses, hits in runs:
            for x, y, yaw in poses:
                for mx, my, _ in ps:
                    r = math.hypot(mx - x, my - y)
                    if r > RMAX_FIT or r < 0.1:
                        continue
                    da = (math.degrees(math.atan2(my - y, mx - x)) - yaw + 540.0) % 360.0 - 180.0
                    if abs(da) <= HFOV:
                        for i, (a, b) in enumerate(BINS):
                            if a <= r < b:
                                opp[i] += 1
                                break
                        break   # una oportunidad por muestra y etiqueta
            for hlab, conf, brg, rng, x, y, yaw in hits:
                if hlab != lab:
                    continue
                a2 = math.radians(yaw + signo * brg)
                px, py = x + rng * math.cos(a2), y + rng * math.sin(a2)
                if min(math.hypot(px - mx, py - my) for mx, my, _ in ps) <= R_ASIGNA:
                    for i, (a, b) in enumerate(BINS):
                        if a <= rng < b:
                            hit[i] += 1
                            confs[i].append(conf)
                            break
        curva = {}
        for i, (a, b) in enumerate(BINS):
            if opp[i] < 30:
                continue
            t = hit[i] / opp[i]
            cs = sorted(confs[i])
            cmed = cs[len(cs) // 2] if cs else None
            curva["%.1f-%.1f" % (a, b)] = {"tasa": round(t, 4), "opp": opp[i], "hits": hit[i],
                                           "conf_med": round(cmed, 2) if cmed else None}
            print("   %4.1f-%4.1f m: tasa %.3f (%d/%d) conf_med %s" %
                  (a, b, t, hit[i], opp[i], cmed and round(cmed, 2)))
        resultado["curvas"][lab] = curva
        tasas[lab] = {i: hit[i] / opp[i] for i, _ in enumerate(BINS) if opp[i] >= 30}

    # factores relativos a chair, mismo pie (canal cargado / canal cargado)
    ch = tasas.get("chair", {})
    for lab in ("couch", "refrigerator"):
        fs = {}
        for i, (a, b) in enumerate(BINS):
            if i in tasas.get(lab, {}) and i in ch and ch[i] > 0.005:
                fs["%.1f-%.1f" % (a, b)] = round(tasas[lab][i] / ch[i], 3)
        resultado["factores"][lab] = fs
        print("\nfactor %s/chair por bin: %s" % (lab, fs))

    dst = os.path.join(RAIZ, "dataset", "curvas_etiqueta.json")
    json.dump(resultado, open(dst, "w"), indent=1)
    print("\nescrito", dst)


if __name__ == "__main__":
    main()
