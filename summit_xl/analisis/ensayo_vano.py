#!/usr/bin/env python3
"""Ensayo sintético offline del vano: ¿tiene margen la huella para el comprobador de Nav2?

Replica la geometría que evalúa `FootprintCollisionChecker` de Nav2 Humble:
rasteriza el PERÍMETRO del polígono (Bresenham entre vértices consecutivos, como
hace `lineCost`) y rechaza si alguna celda del trazo es letal. No comprueba el
interior — eso lo señaló Astra y aquí se respeta.

Barre orientación, descentramiento y fase del vano respecto a la rejilla, que es
la variable que nadie mira y que sola ya se come 4 cm.

Uso:  python3 ensayo_vano.py [--vano 0.742] [--res 0.02] [--padding 0.02]
"""
import argparse, math

LARGO, ANCHO = 0.722, 0.613          # huella medida del Summit XL, sin padding


def celdas_muro(vano, res, fase, t_muro=0.12, alcance=3.0):
    """Celdas letales de las dos jambas. El muro está en x∈[0,t], con hueco en y.

    `fase` desplaza el vano respecto al origen de la rejilla: es la variable que
    decide cuánto se come la discretización.  Una celda se marca letal si el muro
    la toca (es lo que hace la capa de obstáculos con el retorno del láser).
    """
    letales = set()
    y0, y1 = fase - vano / 2.0, fase + vano / 2.0
    i0, i1 = int(math.floor(0.0 / res)), int(math.ceil(t_muro / res))
    j0, j1 = int(math.floor(-alcance / res)), int(math.ceil(alcance / res))
    for i in range(i0, i1):
        for j in range(j0, j1):
            cy0, cy1 = j * res, (j + 1) * res
            # letal si el muro TOCA la celda: basta que no quepa entera en el hueco.
            # Es lo que hace la capa de obstáculos con el retorno del láser sobre
            # la jamba, y es lo contrario de lo que suponía la primera versión.
            if cy0 < y0 or cy1 > y1:
                letales.add((i, j))
    return letales


def traza(a, b, res):
    """Bresenham entre dos puntos del mundo, en índices de celda."""
    x0, y0 = int(math.floor(a[0] / res)), int(math.floor(a[1] / res))
    x1, y1 = int(math.floor(b[0] / res)), int(math.floor(b[1] / res))
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    err = dx - dy
    out = []
    while True:
        out.append((x0, y0))
        if x0 == x1 and y0 == y1:
            return out
        e2 = 2 * err
        if e2 > -dy:
            err -= dy; x0 += sx
        if e2 < dx:
            err += dx; y0 += sy


def huella(cx, cy, th, largo, ancho):
    c, s = math.cos(th), math.sin(th)
    L, W = largo / 2.0, ancho / 2.0
    return [(cx + c * dx - s * dy, cy + s * dx + c * dy)
            for dx, dy in ((L, -W), (L, W), (-L, W), (-L, -W))]


def choca(cx, cy, th, letales, res, largo, ancho):
    v = huella(cx, cy, th, largo, ancho)
    for k in range(4):
        for celda in traza(v[k], v[(k + 1) % 4], res):
            if celda in letales:
                return True
    return False


def cruza(desv, th, letales, res, largo, ancho, fase, paso=0.01):
    """¿Pasa el vano entero avanzando en +x, centrado en `fase+desv`?"""
    x = -1.0
    while x <= 1.2:
        if choca(x, fase + desv, th, letales, res, largo, ancho):
            return False, x
        x += paso
    return True, None


def dilata(letales, n=1):
    """Engorda las celdas letales n celdas en todas direcciones.

    Modela lo que la jamba real deja en el costmap: el retorno del laser dispersa
    y la capa de obstaculos marca celdas mas alla de la superficie. Sin esto el
    ensayo supone una jamba perfecta, que es la hipotesis optimista."""
    out = set(letales)
    for (i, j) in letales:
        for di in range(-n, n + 1):
            for dj in range(-n, n + 1):
                out.add((i + di, j + dj))
    return out


def frontera(letales, res, largo, ancho, fase, desvs):
    """Maximo angulo (grados, paso 0.25) que aun cruza, para cada descentramiento."""
    import math as _m
    out = []
    for d in desvs:
        top = None
        g = 0.0
        while g <= 10.0:
            ok, _ = cruza(d, _m.radians(g), letales, res, largo, ancho, fase)
            if not ok:
                break
            top = g
            g += 0.25
        out.append(top)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vano", type=float, default=0.742)
    ap.add_argument("--res", type=float, default=0.02)
    ap.add_argument("--padding", type=float, default=0.02)
    a = ap.parse_args()

    largo, ancho = LARGO + 2 * a.padding, ANCHO + 2 * a.padding
    fases = [0.0, a.res / 4, a.res / 2, 3 * a.res / 4]

    print("vano %.3f m · rejilla %.0f cm · huella con padding %.3f x %.3f m"
          % (a.vano, a.res * 100, largo, ancho))
    print("margen nominal por lado: %.1f mm\n" % ((a.vano - ancho) / 2 * 1000))

    desvs = [0.000, 0.005, 0.010, 0.015, 0.020]

    for etiqueta, n in (("jamba limpia", 0), ("jamba + 1 celda de dispersion", 1)):
        print("=" * 68)
        print(etiqueta.upper())
        print("=" * 68)
        print("  fase   libre   " + "".join("%9s" % ("%+.0fmm" % (d * 1000)) for d in desvs))
        for fase in fases:
            letales = dilata(celdas_muro(a.vano, a.res, fase), n)
            js = sorted({j for (i, j) in letales if i == 0})
            hueco = [j for j in range(min(js), max(js) + 1) if (0, j) not in letales]
            libre = len(hueco) * a.res
            fr = frontera(letales, a.res, largo, ancho, fase, desvs)
            fila = "  %.3f  %.3f  " % (fase, libre)
            for t in fr:
                fila += "%9s" % ("NO CRUZA" if t is None else "<= %.2f deg" % t)
            print(fila)
        print()

if __name__ == "__main__":
    main()
