# -*- coding: utf-8 -*-
"""Desenlaces secundarios del §9.3, sobre la serie de delta derivada.

Todos se definen respecto a las FRONTERAS DE REFERENCIA: los instantes en que la delta
derivada (estado declarado + pose, dcc_omega.delta_muestra) cambia de conjunto aceptable.
Eso incluye tanto las fronteras escenificadas por el guion como las geometricas (entrar en
la envolvente del objeto, salir de la zona del cristal) -- que es como el protocolo define
la unidad: la frontera de decision, no el evento del guion.

  RETARDO DE CONMUTACION   desde la frontera hasta que Z entra en el nuevo conjunto y se
                           SOSTIENE >=2 muestras. Sin adopcion antes de la siguiente
                           frontera (o el fin) = conmutacion PERDIDA, no un retardo grande
                           (§12: los episodios sin conmutacion requerida van al analisis de
                           falsos cambios, no reciben valores arbitrarios).
  PERSISTENCIA FALSA       cuanto sigue Z en el conjunto VIEJO tras la frontera. Es la otra
                           cara del retardo: entre dejar lo viejo y adoptar lo nuevo puede
                           haber terceros roles, y las dos duraciones se miden aparte.
  RETORNO                  una frontera cuyo conjunto nuevo ya habia gobernado antes en la
                           run se marca retorno, y su exactitud se reporta separada (§9.3
                           la nombra como desenlace propio).
  CONMUTACION INNECESARIA  transiciones de Z que SALEN del conjunto aceptable mientras la
                           delta derivada no cambia. El tableteo DENTRO de un conjunto
                           ({defer,review}) no cuenta: ambos son la abstencion correcta.

Se puntua el resolutor PURO por muestra (evalua_todas), sin el estabilizador: asi las
cuatro condiciones se miden con la misma vara. La constante del estabilizador en vuelo
(+0.5 s de confirmacion) y la de la EMA del contrato (~0.9 s) estan declaradas y se restan
en el analisis, no aqui.

Fronteras mas breves que MIN_SEG (1.0 s) se funden: un parpadeo geometrico de una muestra
(el borde de la envolvente del objeto) no es una frontera exigible.
"""
import math

MIN_SEG = 1.0        # s: duracion minima de un tramo de delta para exigir su adopcion
SOSTEN = 2           # muestras seguidas dentro del conjunto nuevo para contar adopcion


def serie_delta(run, segs, delta_muestra):
    """[(t, conjunto_aceptable, muestra)] con tramos breves fundidos al anterior."""
    ss = run.get("samples") or []
    cruda = []
    for m in ss:
        t = m.get("t")
        if t is None:
            continue
        seg = next((s for s in segs if s["desde"] <= t < s["hasta"]), None)
        if seg is None:
            continue
        cruda.append((t, tuple(sorted(delta_muestra(seg, m))), m))
    # fusion de tramos < MIN_SEG
    out = []
    i = 0
    while i < len(cruda):
        j = i
        while j + 1 < len(cruda) and cruda[j + 1][1] == cruda[i][1]:
            j += 1
        dur = cruda[j][0] - cruda[i][0]
        if out and dur < MIN_SEG:
            prev = out[-1][1]
            out.extend((t, prev, m) for t, _, m in cruda[i:j + 1])
        else:
            out.extend(cruda[i:j + 1])
        i = j + 1
    return out


def puntua_secundarios(run, segs, evalua_todas, usa_pose, delta_muestra, condiciones):
    serie = serie_delta(run, segs, delta_muestra)
    if not serie:
        return {}
    Z = {c: [] for c in condiciones}
    for t, dset, m in serie:
        r = evalua_todas(m, usa_pose=usa_pose)
        for c in condiciones:
            Z[c].append(r[c]["Z"])

    # fronteras de referencia
    fronteras = []          # (idx, set_viejo, set_nuevo, es_retorno)
    vistos = [serie[0][1]]
    for i in range(1, len(serie)):
        if serie[i][1] != serie[i - 1][1]:
            fronteras.append((i, serie[i - 1][1], serie[i][1], serie[i][1] in vistos))
            vistos.append(serie[i][1])

    out = {}
    for c in condiciones:
        z = Z[c]
        adopta = []; pierde = 0; persiste = []; ret_ok = 0; ret_n = 0
        for (i, viejo, nuevo, es_ret) in fronteras:
            fin = next((j for j, _ in enumerate(serie[i:], i) if serie[j][1] != nuevo),
                       len(serie))
            t0 = serie[i][0]
            # adopcion sostenida del conjunto nuevo
            t_ad = None
            for j in range(i, min(fin, len(serie)) - SOSTEN + 1):
                if all(z[k] in nuevo for k in range(j, j + SOSTEN)):
                    t_ad = serie[j][0]; break
            if es_ret:
                ret_n += 1
                ret_ok += 1 if t_ad is not None else 0
            if t_ad is None:
                pierde += 1
            else:
                adopta.append(t_ad - t0)
            # persistencia del conjunto viejo
            t_p = 0.0
            for j in range(i, fin):
                if z[j] in viejo:
                    t_p = serie[j][0] - t0
                else:
                    break
            persiste.append(t_p)
        # conmutacion innecesaria: Z sale del conjunto estando la delta estable
        innec = 0
        for j in range(1, len(serie)):
            if serie[j][1] == serie[j - 1][1] and z[j] != z[j - 1] \
                    and z[j - 1] in serie[j][1] and z[j] not in serie[j][1]:
                innec += 1
        dur_total = serie[-1][0] - serie[0][0]
        out[c] = {"n_fronteras": len(fronteras),
                  "retardo": adopta, "perdidas": pierde,
                  "persistencia": persiste,
                  "retorno_ok": ret_ok, "retorno_n": ret_n,
                  "innecesarias_por_min": 60.0 * innec / max(1e-6, dur_total)}
    return out
