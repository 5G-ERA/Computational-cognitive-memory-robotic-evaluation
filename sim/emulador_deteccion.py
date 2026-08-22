"""Emulador de deteccion CALIBRADO — el canal de vision de la simulacion.

POR QUE existe: renderizar objetos en Isaac y pasarlos por YOLO no reproduce lo que ve el
robot real (medido: mallas de biblioteca dan 'tv' donde la realidad da 'couch'; tarjetas con
pixeles reales aciertan solo en 4 de 18 casos). Para afirmaciones calibradas, el canal de
vision se alimenta de un MODELO AJUSTADO A LOS DATOS REALES, no de pixeles sinteticos.

BASE DE EVIDENCIA: las tandas escenificadas de calib_luz, que son las unicas con denominador
limpio (objeto en marca de cinta, N fotogramas, se sabe cuantas veces se detecto). Las
etiquetas de tanda no son fiables -- hubo lotes con la etiqueta reutilizada -- asi que las
condiciones se separan por el BRILLO del fotograma, que es la variable medida:

    dist   luz(>=108)          poca(<108)
    0.3    P 1.00 conf 0.94    P 1.00 conf 0.94
    1.5    P 0.67 conf 0.93    P 0.78 conf 0.91
    1.8    P 0.95 conf 0.82    P 1.00 conf 0.54     <- el par W2: la luz parte la confianza

LIMITES DECLARADOS: la curva esta medida SOLO para 'chair' y hasta 1.8 m; fuera de ahi se
extrapola de forma conservadora y se marca `extrapolado: true` en cada deteccion emitida.
No sustituye al servidor de percepcion en el robot real: es su gemelo estadistico en sim.
"""
import bisect
import json
import math
import os
import random

HFOV = 28.07          # medio campo horizontal de la camara real (fx=600, W=640)
RMAX = 4.5
UMBRAL_LUZ = 108.0    # brillo que separa 'luz' de 'poca' (medido: ~116 toda luz / ~85 poca)

_AQUI = os.path.dirname(os.path.abspath(__file__))

# curva medida: dist -> (P(det), conf_med, conf_min, conf_max) por condicion
CURVA = {
    "luz":  {0.3: (1.00, 0.94, 0.92, 0.95),
             1.5: (0.67, 0.93, 0.92, 0.93),
             1.8: (0.95, 0.82, 0.46, 0.85)},
    "poca": {0.3: (1.00, 0.94, 0.94, 0.95),
             1.5: (0.78, 0.91, 0.90, 0.93),
             1.8: (1.00, 0.54, 0.47, 0.61)},
}
# mas alla de 1.8 m no hay medida escenificada: caida conservadora, marcada como extrapolada
CAIDA_POR_METRO = 0.35


def _interp(tabla, r):
    ds = sorted(tabla)
    if r <= ds[0]:
        return tabla[ds[0]], False
    if r >= ds[-1]:
        p, c, lo, hi = tabla[ds[-1]]
        extra = (r - ds[-1]) * CAIDA_POR_METRO
        return (max(0.05, p - extra * 0.5), max(0.05, c - extra),
                max(0.05, lo - extra), max(0.05, hi - extra)), True
    i = bisect.bisect_left(ds, r)
    d0, d1 = ds[i - 1], ds[i]
    t = (r - d0) / (d1 - d0)
    a, b = tabla[d0], tabla[d1]
    return tuple(a[k] + t * (b[k] - a[k]) for k in range(4)), False


class EmuladorDeteccion:
    """Emite detecciones con el mismo formato que el servidor real: [lab, conf, brg, rango]."""

    def __init__(self, objetos, ocupacion=None, ocell=0.2, semilla=7):
        """objetos: [(etiqueta, x, y), ...]   ocupacion: set de celdas para la linea de vista."""
        self.objetos = list(objetos)
        self.occ = ocupacion or set()
        self.ocell = ocell
        self.rng = random.Random(semilla)
        # ruido de rumbo y rango, medido sobre emparejamientos reales (ver ajusta_detector)
        self.sigma_brg = 3.0
        self.sigma_rel_rango = 0.12

    # el rayo se detiene ANTES del objeto: los muebles estan en el propio mapa de ocupacion,
    # asi que marchar hasta el centro daba SIEMPRE "tapado" (medido: 28 de 28 objetos).
    MARGEN_OBJETO = 0.45

    def _libre(self, x0, y0, x1, y1):
        """Linea de vista despejada, sin contar la ocupacion del propio objeto."""
        if not self.occ:
            return True
        d = math.hypot(x1 - x0, y1 - y0)
        util = d - self.MARGEN_OBJETO
        if util <= self.ocell:
            return True
        pasos = int(util / (self.ocell * 0.5)) + 1
        ux, uy = (x1 - x0) / d, (y1 - y0) / d
        for i in range(1, pasos):
            s = i * util / pasos
            c = (round((x0 + ux * s) / self.ocell), round((y0 + uy * s) / self.ocell))
            if c in self.occ:
                return False
        return True

    def detecta(self, x, y, yaw, brillo):
        """Detecciones para una pose y un brillo de fotograma."""
        cond = "luz" if brillo >= UMBRAL_LUZ else "poca"
        out = []
        for (lab, ox, oy) in self.objetos:
            dx, dy = ox - x, oy - y
            r = math.hypot(dx, dy)
            if not (0.2 < r <= RMAX):
                continue
            brg = (math.degrees(math.atan2(dy, dx)) - yaw + 540) % 360 - 180
            if abs(brg) > HFOV:
                continue
            if not self._libre(x, y, ox, oy):
                continue
            (p, cmed, cmin, cmax), extrapolado = _interp(CURVA[cond], r)
            # en el borde del campo la deteccion decae (el objeto sale del encuadre)
            p *= max(0.0, 1.0 - max(0.0, abs(brg) - HFOV * 0.75) / (HFOV * 0.25))
            if self.rng.random() > p:
                continue
            # TRIANGULAR con moda en la MEDIANA medida, no uniforme: la distribucion real
            # esta sesgada (a 1.8 m con luz casi todo cae cerca de 0.85 con pocos valores
            # bajos), y muestrear uniforme daba 0.65 donde la realidad da 0.82 -- justo el
            # contraste que sostiene el testigo W2.
            conf = min(0.99, max(0.05, self.rng.triangular(cmin, cmax, cmed)))
            out.append([lab, round(conf, 2),
                        round(brg + self.rng.gauss(0, self.sigma_brg), 1),
                        round(max(0.1, r * (1 + self.rng.gauss(0, self.sigma_rel_rango))), 2),
                        {"extrapolado": extrapolado} if extrapolado else None])
        # el formato real son 4 campos; el 5º (metadatos) solo si hay extrapolacion
        return [d[:4] if d[4] is None else d for d in out]


def carga_por_defecto(raiz=None):
    """Emulador con los objetos y el mapa del proyecto."""
    raiz = raiz or os.path.dirname(_AQUI)
    objs = []
    try:
        g = json.load(open(os.path.join(raiz, "dataset", "objetos_vistos.json")))
        for lab, ol in g.items():
            for o in ol:
                if o.get("n", 0) >= 12:
                    objs.append((lab, o["x"], o["y"]))
    except Exception:
        pass
    occ = set()
    try:
        pts = json.load(open(os.path.join(raiz, "summit", "ref_map_g1.json")))["points"]
        occ = {(round(p[0] / 0.2), round(p[1] / 0.2)) for p in pts}
    except Exception:
        pass
    return EmuladorDeteccion(objs, occ)
