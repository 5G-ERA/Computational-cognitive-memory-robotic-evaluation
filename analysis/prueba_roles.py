"""El resolutor de rol aplicado a material YA grabado. Sirve de dos cosas:
   - comprobar que resuelve algo sensato antes de meterlo en el lazo de control
   - medir cuanto material historico es puntuable (y cuanto no, y por que)
"""
import glob, json, os, sys, collections
sys.path.insert(0, "/home/ros/Documents/G1_UNITREE_ROBOT_META_REASONING")
from dcc_roles import resuelve_rol, ROLES

RAIZ = "/home/ros/Documents/G1_UNITREE_ROBOT_META_REASONING/dataset"
def carga(f):
    try: return json.load(open(f))
    except Exception: return None

def grupo(pat, et, filtro=None):
    cnt = collections.Counter(); aut = collections.Counter()
    razones = collections.Counter(); tot = 0; nr = 0
    for f in sorted(glob.glob(os.path.join(RAIZ, pat))):
        d = carga(f)
        if not d: continue
        if filtro and not filtro(d, f): continue
        ss = d.get("samples") or []
        if len(ss) < 30: continue
        nr += 1
        for m in ss:
            r, why, a = resuelve_rol(m)
            cnt[r] += 1; aut[a] += 1; tot += 1
            pt = why.split(":")
            clave = pt[0] if len(pt) < 2 else pt[0] + ":" + pt[1].split("=")[0].split(",")[0]
            razones[clave] += 1
    if not tot:
        print("%-30s sin datos" % et); return
    print("\n=== %s — %d runs, %d muestras ===" % (et, nr, tot))
    print("  ROL:      " + "  ".join("%s %.0f%%" % (r, 100.0*cnt[r]/tot) for r in ROLES if cnt[r]))
    print("  AUTORIDAD:" + "  ".join(" %s %.0f%%" % (a, 100.0*aut[a]/tot) for a, _ in aut.most_common()))
    print("  razones dominantes:")
    for k, v in razones.most_common(5):
        print("     %-46s %5.1f%%" % (k, 100.0*v/tot))

grupo("2026*_ours_[AB].json", "TODO EL HISTORICO REAL (132 runs)",
      lambda d, f: d.get("sim_id") is None and not os.path.basename(f).startswith("20260823_"))
grupo("20260821_*_ours_[AB].json", "21-ago: sesion con campos META")
grupo("20260823_19*_ours_[AB].json", "hoy: gemelo calibrado")
