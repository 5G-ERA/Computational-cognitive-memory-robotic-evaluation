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

# ATENUACION DE NAVEGACION LIBRE. La curva de arriba se midio con el objeto ESCENIFICADO: en
# su marca de cinta, centrado, con el robot quieto y el canal de video EN REPOSO. Es un TECHO,
# no la tasa de un run normal.
#
# CAUSA FISICA PRINCIPAL (apuntada por Adrian y medida en nuestros datos): las imagenes viajan
# por el canal WEBRTC de la app, que es un cuello de botella. Durante una travesia el canal va
# saturado y TODOS los fotogramas llegan degradados -- nitidez mediana 3.5 frente a 6.1 en las
# capturas estaticas de tanda (1.7x), y en la medicion de agosto la caida fue mucho mayor
# todavia (52 frente a 476 en nitidez de borde). A eso se suman encuadre descentrado, oclusion
# parcial y las perdidas del propio servidor (latencia, caidas).
#
# OJO: la degradacion NO depende de la velocidad instantanea -- se comprobo y la tasa real de
# deteccion es plana (16.2% parado, 16.0% lento, 16.9% rapido). Es el canal bajo carga durante
# TODO el run, no el desenfoque de movimiento. Por eso el factor es constante y no v-dependiente.
#
# CALIBRACION: sobre 15837 poses reales, atenuacion 0.18 reproduce el reparto medido de
# detecciones por muestra (real 84/13/3/0 %, emulado 83/14/3/0, error 2 pp). Sin atenuar el
# emulador disparaba en el 59% de las muestras y sacaba 3 o mas en el 21%: 23 veces mas
# detecciones por run que la realidad.
ATENUACION = float(os.environ.get("G1_EMU_ATEN", "0.18"))
# El canal degradado no solo hace FALLAR detecciones: tambien baja la CONFIANZA de las que
# salen. Medido: en runs reales la confianza mediana es 0.68 [0.55-0.83], mientras que en las
# tandas escenificadas (canal en reposo) es 0.91-0.94. Misma causa, dos efectos.
CONF_PENAL = float(os.environ.get("G1_EMU_CONFPEN", "0.14"))
MAX_DETS = int(os.environ.get("G1_EMU_MAXDETS", "2"))   # el servidor real nunca dio mas de 2

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

# POR ETIQUETA (25-ago, analysis/curvas_etiqueta.py sobre 6112 detecciones reales de
# navegacion libre con denominador geometrico). Solo CAMPO CERCANO (0-3 m), que es el
# regimen de escenificacion T5/T6: mas alla el rango REPORTADO por el servidor real esta
# envenenado -- el lidar del canal de rango atraviesa el cristal de Z2 y devuelve lo que
# hay DETRAS (clusters retroproyectados en y=5..10.7, fuera del sobre navegado real
# x[-5,1.5] y[-1.6,3]) -- y una curva sobre ese rango heredaria el artefacto. Anclas de
# cordura del ajuste: 16.3% de muestras con deteccion (real conocido ~16%) y chair 0.168
# frente a 0.162 del propio emulador (techo x atenuacion).
FACTOR_ETIQUETA = {"chair": 1.0, "couch": 0.37, "refrigerator": 0.51}
CONF_DELTA_ETIQUETA = {"chair": 0.0, "couch": -0.05, "refrigerator": -0.15}
# 'person' NO se emula: se mueve y no tiene curva ajustable con denominador honesto.


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
            p *= ATENUACION                      # de condiciones escenificadas a run normal
            p *= FACTOR_ETIQUETA.get(lab, 1.0)   # curva base medida con chair; resto relativo
            if self.rng.random() > p:
                continue
            # TRIANGULAR con moda en la MEDIANA medida, no uniforme: la distribucion real
            # esta sesgada (a 1.8 m con luz casi todo cae cerca de 0.85 con pocos valores
            # bajos), y muestrear uniforme daba 0.65 donde la realidad da 0.82 -- justo el
            # contraste que sostiene el testigo W2.
            conf = self.rng.triangular(cmin, cmax, cmed) - CONF_PENAL * self.rng.uniform(0.7, 1.3) \
                   + CONF_DELTA_ETIQUETA.get(lab, 0.0)
            # el suelo no es el umbral del servidor sino lo que se OBSERVA en runs reales: por
            # debajo de ~0.45 practicamente no hay detecciones aceptadas en el historico
            conf = min(0.99, max(0.45, conf))
            out.append([lab, round(conf, 2),
                        round(brg + self.rng.gauss(0, self.sigma_brg), 1),
                        round(max(0.1, r * (1 + self.rng.gauss(0, self.sigma_rel_rango))), 2),
                        {"extrapolado": extrapolado} if extrapolado else None])
        # el servidor real devolvia como mucho 2 detecciones; se quedan las mas cercanas
        if len(out) > MAX_DETS:
            out.sort(key=lambda d: d[3])
            out = out[:MAX_DETS]
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
