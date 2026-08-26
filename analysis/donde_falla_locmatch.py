"""¿DONDE fallan los retornos del gemelo contra el mapa de referencia?
loc_match = fraccion de celdas del laser en vivo que caen sobre (o junto a) el refmap.
Gemelo 0.63-0.69 contra 0.94 real. Aqui se localizan los fallos en el espacio en vez de
seguir conjeturando la causa."""
import glob, json, os, sys, collections, math
sys.path.insert(0, "/home/ros/Documents/G1_UNITREE_ROBOT_META_REASONING")
import g1_goto as g1

OC = g1.g.OCELL
ref = g1.load_ref_map()

def celdas(pts):
    return [(round(x / OC), round(y / OC)) for x, y in pts]

def acierta(c):
    return any((c[0]+dx, c[1]+dy) in ref for dx in (-1,0,1) for dy in (-1,0,1))

def analiza(f, et):
    d = json.load(open(f))
    snaps = d.get("laser_snapshots") or []
    if not snaps:
        print("%-30s sin laser_snapshots" % et); return
    fallos = collections.Counter(); tot = 0; hit = 0
    for s in snaps:
        for c in celdas(s.get("pts") or []):
            tot += 1
            if acierta(c): hit += 1
            else: fallos[c] += 1
    if not tot:
        print("%-30s sin puntos" % et); return
    print("\n=== %s ===" % et)
    print("  celdas de laser: %d | aciertan el refmap: %.0f%%" % (tot, 100.0*hit/tot))
    # ¿los fallos estan concentrados o repartidos?
    print("  celdas distintas que fallan: %d" % len(fallos))
    top = fallos.most_common(12)
    print("  las que mas fallan (metros):")
    for (cx, cy), n in top:
        print("     (%6.2f,%6.2f)  %4d puntos   dist al refmap mas cercano: %.2f m"
              % (cx*OC, cy*OC, n,
                 OC*min(math.hypot(cx-rx, cy-ry) for rx, ry in ref)))
    # reparto por franja: ¿fuera del recinto del refmap o dentro?
    xs=[c[0] for c in ref]; ys=[c[1] for c in ref]
    x0,x1,y0,y1 = min(xs),max(xs),min(ys),max(ys)
    fuera = sum(n for (cx,cy),n in fallos.items() if not (x0<=cx<=x1 and y0<=cy<=y1))
    print("  fallos FUERA de la caja del refmap: %.0f%% de los fallos" % (100.0*fuera/max(1,sum(fallos.values()))))

RAIZ = "/home/ros/Documents/G1_UNITREE_ROBOT_META_REASONING/dataset"
gem = sorted(glob.glob(os.path.join(RAIZ, "20260824_*_ours_[AB].json")))
gem = [f for f in gem if len(json.load(open(f)).get("samples") or []) > 30]
if gem: analiza(gem[-1], "GEMELO (ultima run)")
reales = [f for f in sorted(glob.glob(os.path.join(RAIZ, "20260821_*_ours_[AB].json")))
          if len(json.load(open(f)).get("samples") or []) > 30]
if reales: analiza(reales[0], "REAL 21-ago")
