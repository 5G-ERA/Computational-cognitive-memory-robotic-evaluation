# -*- coding: utf-8 -*-
"""ESCALA DECLARADA para comparar metricas derivadas de pose entre gemelo y robot.

EL PROBLEMA (24-ago, analysis/camino_decimado.py): la variacion total de una senal ruidosa
diverge con la frecuencia de muestreo -- el problema de la longitud de la costa. path_m se
computa como suma de |delta pose| entre muestras a ~3 Hz, y el temblor del SLAM real infla
esa suma un 44% mientras la pose suave del gemelo solo un 11%. Comparar los dos a escala
nativa fabrico tres hallazgos falsos: "el gemelo va demasiado directo", VSCALE=1.20 (1.5x
alto) y una duracion "corta" que solo era la ganancia inflada.

LA REGLA:
  - Una metrica derivada de INCREMENTOS de pose (camino, eficiencia, v/cmd, serpenteo)
    que compare SISTEMAS DISTINTOS se computa SIEMPRE a la escala declarada K_COMPARA.
  - Dentro de un mismo sistema (brazo A vs brazo B del gemelo, run real vs run real) se
    puede comparar a escala nativa: el ruido es compartido y se cancela en el contraste.
  - spd integrado en el tiempo NO es arbitro del camino: es cota INFERIOR (no captura el
    desplazamiento lateral; medido 4.96 m mediano contra un suelo geometrico de 6.14).

POR QUE K_COMPARA = 8 (~2.4 s a la cadencia real de 0.3 s):
  - a partir de k=8 las pendientes de caida del camino real y del gemelo se igualan
    (-4.0% frente a -3.4% de k=8 a k=12): el ruido ya murio y lo que queda es movimiento;
  - 2.4 s aun resuelve la maniobra del vano, que dura 5-10 s -- decimar mas empezaria a
    comerse correcciones reales, no ruido.
"""
import math

K_COMPARA = 8          # muestras (~2.4 s a 3.3 Hz): la escala declarada
DT_NOMINAL = 0.3       # s por muestra en ambos sistemas (medido: tic 0.300 en los dos)


# CAMPOS AFECTADOS EN EL REGISTRO DE RUNS, para que nadie los compare mal:
#   path_m       nativo. Valido DENTRO de un sistema; entre sistemas, inflado por el ruido
#                de pose (real +44%, gemelo +11%). Se conserva por continuidad historica.
#   path_m_k8    el mismo camino a la escala declarada. ESTE es el de comparar sistemas.
#   efficiency   = straight_m / path_m, asi que HEREDA el problema: entre sistemas subestima
#                al que mas ruido de pose tiene. Comparese recomputada sobre path_m_k8, o
#                no se compare entre sistemas. No se reescribe el campo historico.
#   v_camino,
#   v/cmd        derivados de camino: misma regla.

def camino(muestras, k=K_COMPARA):
    """Camino recorrido a la escala declarada. `muestras` = lista de dicts con x,y."""
    pts = muestras[::max(1, int(k))]
    return sum(math.hypot(b["x"] - a["x"], b["y"] - a["y"])
               for a, b in zip(pts, pts[1:]))


def v_camino(muestras, k=K_COMPARA):
    """Velocidad de camino (camino declarado / duracion)."""
    if len(muestras) < 2:
        return None
    T = muestras[-1]["t"] - muestras[0]["t"]
    return camino(muestras, k) / T if T > 1e-6 else None


def v_cmd(muestras, k=K_COMPARA):
    """Realizacion orden->movimiento a la escala declarada: v_camino / |cmd| medio."""
    mags = []
    for m in muestras:
        c = m.get("cmd") or []
        if len(c) >= 2:
            try:
                mags.append(math.hypot(float(c[0]), float(c[1])))
            except (TypeError, ValueError):
                pass
    if not mags:
        return None
    v = v_camino(muestras, k)
    mm = sum(mags) / len(mags)
    return (v / mm) if (v is not None and mm > 1e-6) else None
