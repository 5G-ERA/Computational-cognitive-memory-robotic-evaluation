#!/usr/bin/env python3
"""Puntuacion W2 ESTATICA con material real: la silla en marcas de cinta, dos condiciones de luz.

Demostracion de desarrollo del par W2 (21-ago noche, sobre los lotes de calib_luz):
  - mitad ILUMINADA (20-ago, detecciones CON rango): Omega=object
  - mitad POCA LUZ (21-ago, silla 1.8m, conf 0.47-0.61 inestable): Omega=illumination
La evidencia de iluminacion se adjunta como campo I1 declarado desde la etiqueta del lote
(con correccion por brillo para los dos lotes iluminados que reutilizaron por error la
etiqueta "poca luz": brillo ~115 los identifica). NO es el testigo confirmatorio (estatico,
sin escenificacion del protocolo, sin oscuridad profunda): demuestra la ASIMETRIA de
resolucion con datos reales.

Resultado (21-ago): C4 100/100 (resuelve ambas mitades); C2 100/0 — ciega exactamente donde
vive la distincion; C1/C3 0/0 por diseno (el titular no reconstruye roles).

Uso: python3 analysis/w2_estatico.py
"""
import collections
import json
import os
import sys

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)
import dcc_roles as R                                            # noqa: E402


def lote_a_muestras(filas, illum):
    for f in filas:
        dets = [[d[0], d[1], None, d[2]]
                for d in ((f.get("percepcion") or {}).get("detecciones") or [])]
        yield {"t": 0, "bat": 90, "dets": dets or None, "illum_state": illum}


def puntua(muestras, delta):
    ac = {c: 0 for c in ("C1", "C2", "C3", "C4")}
    fallos = collections.defaultdict(collections.Counter)
    n = 0
    for m in muestras:
        n += 1
        for c in ac:
            out = R.condicion(m, c)["out"]
            ok = (out == delta) and c not in ("C1", "C3")
            ac[c] += ok
            if not ok:
                fallos[c]["%s->%s" % (delta, out)] += 1
    return n, ac, fallos


def main():
    # mitad ILUMINADA: 20-ago, con rango (las del 21 vinieron sin rango: scan virtual vacio)
    f20 = [json.loads(l) for l in open(os.path.join(RAIZ, "calib_luz/2026-08-20/muestras.jsonl"))
           if l.strip()]
    lit = [f for f in f20 if "silla" in f.get("etiqueta", "").lower()
           and (f.get("imagen") or {}).get("brillo_medio", 0) > 95
           and any(d[0] == "chair" and d[2] is not None
                   for d in ((f.get("percepcion") or {}).get("detecciones") or []))]
    # mitad POCA LUZ: 21-ago, silla 1.8m con brillo bajo (condicion VERDADERA por brillo)
    f21 = [json.loads(l) for l in open(os.path.join(RAIZ, "calib_luz/2026-08-21/muestras.jsonl"))
           if l.strip()]
    oscura = [f for f in f21 if "1.8m" in f.get("etiqueta", "")
              and (f.get("imagen") or {}).get("brillo_medio", 100) < 100]

    for nombre, filas, illum, delta in (
            ("ILUMINADA (20-ago, con rango)", lit, "adequate", "object"),
            ("POCA LUZ (21-ago, 1.8m)", oscura, "inadequate", "illumination")):
        n, ac, fallos = puntua(lote_a_muestras(filas, illum), delta)
        print("%-32s n=%2d  Omega=%-12s C1 %3d%%  C2 %3d%%  C3 %3d%%  C4 %3d%%" % (
            nombre, n, delta, *(100 * ac[c] // max(1, n) for c in ("C1", "C2", "C3", "C4"))))
        for c in ("C2", "C4"):
            if fallos[c]:
                print("   fallos %s: %s" % (c, dict(fallos[c].most_common(2))))


if __name__ == "__main__":
    main()
