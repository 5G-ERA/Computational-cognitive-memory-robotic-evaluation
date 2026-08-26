"""¿En que discrepan la sala B del gemelo y el mapa de referencia?

Antes de mover geometria hay que saber si es (a) una TRASLACION -- todo corrido medio
metro, que seria un error de marco -- o (b) formas distintas, que seria un error de
modelado. Se prueba correlando: se desplaza el conjunto de celdas del gemelo sobre el
refmap y se busca el desplazamiento que maximiza el solape. Si hay un maximo claro
distinto de (0,0), es traslacion.
"""
import json, sys, collections
sys.path.insert(0, "/home/ros/Documents/G1_UNITREE_ROBOT_META_REASONING")
import g1_goto as g1

OC = g1.g.OCELL
ref = g1.load_ref_map()

# celdas que el laser del gemelo produjo (de los snapshots de una run reciente)
import glob, os
RAIZ = "/home/ros/Documents/G1_UNITREE_ROBOT_META_REASONING/dataset"
cands = [f for f in sorted(glob.glob(os.path.join(RAIZ, "20260824_*_ours_[AB].json")))
         if len(json.load(open(f)).get("samples") or []) > 30]
f = cands[-1]
d = json.load(open(f))
vivas = set()
for s in (d.get("laser_snapshots") or []):
    for x, y in (s.get("pts") or []):
        vivas.add((round(x / OC), round(y / OC)))
print("run: %s" % os.path.basename(f))
print("celdas vivas del gemelo: %d | refmap: %d" % (len(vivas), len(ref)))

# --- SALA B: x -3..-7.5, y 0.5..5.5 ---
def en_B(c):
    x, y = c[0] * OC, c[1] * OC
    return -7.5 <= x <= -3.0 and 0.5 <= y <= 5.5
vB = {c for c in vivas if en_B(c)}
rB = {c for c in ref if en_B(c)}
print("\nen la SALA B:  gemelo %d celdas | refmap %d celdas" % (len(vB), len(rB)))

def solape(dx, dy, A, B):
    return sum(1 for c in A if (c[0]+dx, c[1]+dy) in B)

print("\n--- barrido de traslacion (solape del gemelo sobre el refmap) ---")
mejor = None; rej = []
for dy in range(-4, 5):
    fila = []
    for dx in range(-4, 5):
        s = solape(dx, dy, vB, rB)
        fila.append(s)
        if mejor is None or s > mejor[0]:
            mejor = (s, dx, dy)
    rej.append((dy, fila))
print("        dx:" + "".join("%5d" % dx for dx in range(-4, 5)))
for dy, fila in rej:
    print("  dy=%3d   " % dy + "".join("%5d" % v for v in fila))
s0 = solape(0, 0, vB, rB)
print("\nsolape sin desplazar: %d de %d (%.0f%%)" % (s0, len(vB), 100.0*s0/max(1,len(vB))))
print("mejor desplazamiento: dx=%d dy=%d -> %d (%.0f%%)  = (%.2f, %.2f) m"
      % (mejor[1], mejor[2], mejor[0], 100.0*mejor[0]/max(1,len(vB)), mejor[1]*OC, mejor[2]*OC))

# comparacion: la sala A, que sabemos que va bien
def en_A(c):
    x, y = c[0]*OC, c[1]*OC
    return -2.0 <= x <= 3.0 and -2.0 <= y <= 3.0
vA = {c for c in vivas if en_A(c)}; rA = {c for c in ref if en_A(c)}
mA = max(((solape(dx,dy,vA,rA), dx, dy) for dx in range(-4,5) for dy in range(-4,5)))
print("\nSALA A (control): sin desplazar %.0f%% | mejor dx=%d dy=%d -> %.0f%%"
      % (100.0*solape(0,0,vA,rA)/max(1,len(vA)), mA[1], mA[2], 100.0*mA[0]/max(1,len(vA))))
