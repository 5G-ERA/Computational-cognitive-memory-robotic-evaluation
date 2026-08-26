# -*- coding: utf-8 -*-
"""Barrido por rayos para la memoria de voxels — paso 4 del §8.

QUE ARREGLA
-----------
La memoria de voxels (Renxi, 14-ago) recuerda una celda confirmada a distancia sana y la
reinyecta mientras esta dentro de la banda donde el fabricante recorta el laser. Con SOLO
caducidad por TTL, una celda que se sigue confirmando refresca su sello y no expira nunca:
en el gemelo salio 5/6 neutral y 1/6 CATASTROFICO -- 7 colisiones en una run, 41 celdas
sostenidas contra las 17-24 de las runs sanas. Faltaba la mitad negativa de la evidencia.

Un TTL dice "ha pasado tiempo". Un rayo dice algo mucho mas fuerte: "he MIRADO y no hay
nada". Si un rayo del barrido actual atraviesa una celda y termina MAS ALLA, esa celda esta
demostrada libre y la memoria debe soltarla en el acto, no dentro de 3 segundos.

LAS TRES CONDICIONES QUE HACEN QUE ESTO SEA VALIDO
--------------------------------------------------
1. **Sólo con barrido FRESCO.** Un barrido repetido no es una observacion nueva; despejar
   con el seria inventarse evidencia.
2. **Nunca dentro de la banda ciega** (NEAR_BLIND = 0.60 m). Ahi el laser esta recortado por
   diseno, asi que un rayo que "pasa" por esa zona NO prueba que este libre -- y es
   precisamente la zona para la que existe la memoria. Despejar ahi destruiria el mecanismo.
3. **Nunca la celda del extremo ni la anterior.** El extremo es el obstaculo, y la vecina
   cae dentro del error de cuantizacion de la rejilla.

Y una trampa que ya me costo una vez, escrita para no repetirla: los rayos se marchan contra
el barrido INSTANTANEO (la nube 'location' en vivo). En el trabajo de limpieza de la oficina
marche rayos contra `laser_snapshots.pts` creyendo que era un barrido, y es el mapa de
obstaculos ACUMULADO del robot: esos rayos nunca existieron y aquello borro geometria buena.
La fuente aqui es la de un solo instante, y esta funcion no acepta otra cosa.
"""
import math


def celdas_del_rayo(x0, y0, x1, y1, oc, paso_rel=3.0):
    """Celdas que atraviesa el segmento (x0,y0)->(x1,y1), SIN incluir la del extremo.

    Se muestrea a oc/paso_rel para no saltarse celdas en diagonales. Devuelve una lista en
    orden de recorrido, con su distancia al origen, para poder aplicar la banda ciega.
    """
    dx, dy = x1 - x0, y1 - y0
    L = math.hypot(dx, dy)
    if L < 1e-9:
        return []
    ux, uy = dx / L, dy / L
    paso = oc / paso_rel
    fin = L - oc                       # guarda de una celda antes del obstaculo
    out = []
    vistas = set()
    d = paso
    while d < fin:
        cx = int(round((x0 + ux * d) / oc))
        cy = int(round((y0 + uy * d) / oc))
        c = (cx, cy)
        if c not in vistas:
            vistas.add(c)
            out.append((c, d))
        d += paso
    return out


def despeja(pose, puntos_vivos, memoria, oc=0.20, near_blind=0.60, fresco=True):
    """Celdas de `memoria` demostradas LIBRES por el barrido actual.

    pose          (x, y) del robot en el frame del mapa
    puntos_vivos  iterable de (x, y): extremos del barrido INSTANTANEO
    memoria       iterable de celdas (cx, cy) que la memoria sostiene
    fresco        False -> no se despeja nada (un barrido repetido no es evidencia)

    Devuelve (a_soltar, n_rayos, n_celdas_miradas).
    """
    if not fresco or not memoria or not puntos_vivos:
        return set(), 0, 0
    mem = set(memoria)
    px, py = pose
    a_soltar = set()
    miradas = 0
    nr = 0
    for (qx, qy) in puntos_vivos:
        nr += 1
        for c, d in celdas_del_rayo(px, py, qx, qy, oc):
            if d <= near_blind:
                continue               # banda ciega: atravesarla no prueba nada
            miradas += 1
            if c in mem:
                a_soltar.add(c)
    return a_soltar, nr, miradas
