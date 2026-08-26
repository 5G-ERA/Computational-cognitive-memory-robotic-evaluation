import os
"""Las cuatro condiciones sobre material grabado. Todavia NO es A_meta -- para eso hace falta
delta_t, que sale del guion del experimento y aun no existe. Lo que se comprueba aqui es que
cada condicion se comporta como el protocolo describe, que es el requisito previo."""
import collections, glob, json, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src"))
from dcc_conditions import evalua_todas, CONDICIONES

RAIZ = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "dataset")
def carga(f):
    try: return json.load(open(f))
    except Exception: return None

def corre(pat, et, filtro=None, limite=None):
    cnt = {c: collections.Counter() for c in CONDICIONES}
    inc = collections.Counter(); tot = 0; nr = 0
    for f in sorted(glob.glob(os.path.join(RAIZ, pat))):
        d = carga(f)
        if not d or (filtro and not filtro(d, f)): continue
        ss = d.get("samples") or []
        if len(ss) < 30: continue
        nr += 1
        if limite and nr > limite: break
        for m in ss:
            r = evalua_todas(m); tot += 1
            for c in CONDICIONES:
                cnt[c][r[c]["Z"]] += 1
            inc[r["C3"]["estado_incumbente"]] += 1
    if not tot:
        print("%-34s sin datos" % et); return
    print("\n=== %s — %d runs, %d boundaries ===" % (et, nr, tot))
    roles = ["motion","lidar_quality","illumination","object","energy","review","defer","no_use"]
    print("      " + "".join("%14s" % r for r in roles))
    for c in CONDICIONES:
        print("  %-4s" % c + "".join("%13.0f%%" % (100.0*cnt[c][r]/tot) if cnt[c][r] else "%14s" % "-" for r in roles))
    print("  estado incumbente (C3): " + "  ".join("%s %.0f%%" % (k, 100.0*v/tot) for k, v in inc.most_common()))
    # contrastes en el unico sentido comprobable sin delta_t: cuanto DIFIEREN
    def dif(a, b):
        return 100.0 * sum((cnt[a] - cnt[b])[k] for k in cnt[a]) / tot
    print("  divergencia de decisiones:  C4 vs C3 %.0f%%   C4 vs C2 %.0f%%   C2 vs C1 %.0f%%   C4 vs C1 %.0f%%"
          % (dif("C4","C3"), dif("C4","C2"), dif("C2","C1"), dif("C4","C1")))

corre("2026*_ours_[AB].json", "HISTORICO REAL (interfaz I0 de facto)",
      lambda d, f: d.get("sim_id") is None and not os.path.basename(f).startswith("20260823_"), limite=40)
corre("20260821_*_ours_[AB].json", "21-ago real: campos META presentes")
corre("20260823_2107*_ours_B.json", "gemelo ILUMINADO (frame 118)")
corre("20260823_2110*_ours_B.json", "gemelo OSCURO (frame 85)")
