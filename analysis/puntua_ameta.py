import os
"""Primer intento REAL de puntuar A_meta = 1[Z_t = delta_t] sobre una run escenificada.

delta_t sale del guion (declarado antes de correr) y del registro independiente de la
transicion. Z_t sale de cada condicion C1-C4 evaluada sobre la misma run grabada -- que es
lo que significa el nivel de replay: una sola run rinde las cuatro condiciones, porque
difieren en QUE informacion usan y COMO resuelven, ambas cosas computables a posteriori.

Lo que este script NO hace todavia es A_Omega: para eso hacen falta certificados kappa_t,
que no existen. Aqui se mide exactamente hasta donde llegamos hoy.
"""
import json, os, sys, collections
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src"))
from dcc_conditions import evalua_todas, usa_pose_para, CONDICIONES

RAIZ = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "dataset")

def delta_de_guion(guion, t_transicion, t):
    """delta_t declarado: el estado esperado antes/despues del instante de la transicion."""
    return guion[0] if t < t_transicion else guion[1]

def puntua(fichero, guion, t_transicion, ventana=2.0):
    d = json.load(open(os.path.join(RAIZ, fichero)))
    ss = d.get("samples") or []
    up = usa_pose_para(d)
    acc = {c: [0, 0] for c in CONDICIONES}     # [aciertos, total]
    for m in ss:
        t = m.get("t")
        if t is None: continue
        # zona de gracia alrededor de la frontera: el retardo instrumental esta declarado
        if abs(t - t_transicion) < ventana: continue
        dt = delta_de_guion(guion, t_transicion, t)
        r = evalua_todas(m, usa_pose=up)
        for c in CONDICIONES:
            acc[c][1] += 1
            if r[c]["Z"] == dt: acc[c][0] += 1
    return acc, len(ss)

# T3 tal y como la escenifico guion.py: illumination (luz alta) -> motion (luz baja)
# OJO: es la direccion INVERTIDA respecto a la tabla de Renxi -- ver el commit 7b37d55.
CASOS = [
    ("20260824_113950_ours_B.json", ("illumination", "motion"), 20.8),
    ("20260824_115222_ours_A.json", ("illumination", "motion"), 21.0),
]

print("A_meta = 1[Z_t = delta_t] · delta_t del guion, ventana de gracia +-2 s en la frontera\n")
tot = {c: [0, 0] for c in CONDICIONES}
for f, guion, tt in CASOS:
    acc, n = puntua(f, guion, tt)
    print("%s  (%d muestras, transicion declarada a t=%.1f)" % (f, n, tt))
    for c in CONDICIONES:
        a, t_ = acc[c]
        tot[c][0] += a; tot[c][1] += t_
        print("   %s  A_meta = %3d/%3d = %.0f%%" % (c, a, t_, 100.0*a/t_ if t_ else 0))
    print()
print("=== agregado (n=%d fronteras) ===" % tot["C1"][1])
for c in CONDICIONES:
    a, t_ = tot[c]
    print("  %s  %.0f%%" % (c, 100.0*a/t_ if t_ else 0))
print()
print("contrastes:  C4-C3 %+.0f pp   C4-C2 %+.0f pp   C3-C1 %+.0f pp   C4-C1 %+.0f pp"
      % tuple(100.0*(tot[x][0]/tot[x][1] - tot[y][0]/tot[y][1])
              for x, y in (("C4","C3"), ("C4","C2"), ("C3","C1"), ("C4","C1"))))
