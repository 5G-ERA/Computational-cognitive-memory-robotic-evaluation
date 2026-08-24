"""¿Estoy comparando lo mismo? El loc_match de una run es global. Si el robot real apenas
produce retornos en la sala B, su 0.94 lo domina la sala A y la comparacion no es justa.
Aqui se separa el acierto POR SALA, en real y en gemelo."""
import glob, json, os, sys
sys.path.insert(0, "/home/ros/Documents/G1_UNITREE_ROBOT_META_REASONING")
import g1_goto as g1
OC = g1.g.OCELL
ref = g1.load_ref_map()

def zona(c):
    x, y = c[0]*OC, c[1]*OC
    if -7.5 <= x <= -3.0 and 0.5 <= y <= 5.5: return "B"
    if -2.0 <= x <= 3.0 and -2.0 <= y <= 3.0: return "A"
    return "otro"

def acierta(c):
    return any((c[0]+dx, c[1]+dy) in ref for dx in (-1,0,1) for dy in (-1,0,1))

def analiza(f, et):
    d = json.load(open(f))
    tot = {"A":0,"B":0,"otro":0}; hit = {"A":0,"B":0,"otro":0}
    for s in (d.get("laser_snapshots") or []):
        for x, y in (s.get("pts") or []):
            c = (round(x/OC), round(y/OC)); z = zona(c)
            tot[z] += 1
            if acierta(c): hit[z] += 1
    n = sum(tot.values())
    if not n: print("%-34s sin puntos"%et); return
    print("%-34s" % et, end="")
    for z in ("A","B","otro"):
        pct = 100.0*tot[z]/n
        ac = 100.0*hit[z]/tot[z] if tot[z] else -1
        print("  %s: %4.0f%% del laser, acierta %3.0f%%" % (z, pct, ac), end="")
    print("   | global %3.0f%%" % (100.0*sum(hit.values())/n))

RAIZ = "/home/ros/Documents/G1_UNITREE_ROBOT_META_REASONING/dataset"
print("%-34s%s" % ("", "  reparto y acierto por sala"))
for pat, et in (("20260821_*_ours_[AB].json", "REAL 21-ago"),
                ("20260824_*_ours_[AB].json", "GEMELO 24-ago")):
    fs = [f for f in sorted(glob.glob(os.path.join(RAIZ, pat)))
          if len(json.load(open(f)).get("samples") or []) > 30]
    for f in fs[:3]:
        analiza(f, "%s %s" % (et, os.path.basename(f)[9:15]))
