"""¿Cuanto aporta de verdad el disparador de POSE al verificador C1?

Si loc_match esta saturado en el robot real (p5=0.81, mediana 0.94), un umbral en 0.80
apenas dispara y el disparador no sostiene nada. En ese caso renunciar a el en el gemelo
-- opcion (a) -- no cuesta casi nada, y regenerar el mapa de referencia -- opcion (b) --
seria cambiar la planificacion para arreglar una senal que ya no mide.
"""
import collections, glob, json, os, sys
sys.path.insert(0, "/home/ros/Documents/G1_UNITREE_ROBOT_META_REASONING")
import dcc_conditions as DC

RAIZ = "/home/ros/Documents/G1_UNITREE_ROBOT_META_REASONING/dataset"
def carga(f):
    try: return json.load(open(f))
    except Exception: return None

def corre(pat, et, filtro=None, limite=None):
    orig = DC.verifica_incumbente
    def sin_pose(v):
        w = dict(v); w.pop("loc_conf", None); w.pop("loc_match", None)
        return orig(w)
    def solo_pose(v):
        w = {k: val for k, val in v.items() if k in ("loc_conf", "loc_match", "err")}
        return orig(w)
    fs = [f for f in sorted(glob.glob(os.path.join(RAIZ, pat)))
          if carga(f) and (not filtro or filtro(carga(f), f))
          and len(carga(f).get("samples") or []) >= 30]
    if limite: fs = fs[:limite]
    tot = 0; est = {"completo": collections.Counter(), "sin_pose": collections.Counter(),
                    "solo_pose": collections.Counter()}
    for f in fs:
        for m in (carga(f).get("samples") or []):
            v = DC.vista(m, "I0"); tot += 1
            est["completo"][orig(v)[0]] += 1
            est["sin_pose"][sin_pose(v)[0]] += 1
            est["solo_pose"][solo_pose(v)[0]] += 1
    if not tot: print("%-30s sin datos" % et); return
    print("\n=== %s — %d runs, %d boundaries ===" % (et, len(fs), tot))
    for k in ("completo", "sin_pose", "solo_pose"):
        c = est[k]
        print("  C1 %-10s  retain %5.1f%%   unresolved %5.1f%%   reject %4.1f%%"
              % (k, 100.0*c["retain"]/tot, 100.0*c["unresolved"]/tot, 100.0*c["reject"]/tot))
    d = abs(est["completo"]["unresolved"] - est["sin_pose"]["unresolved"]) / tot
    print("  --> quitar el disparador de pose mueve C1 en %.1f puntos" % (100.0*d))

corre("20260821_*_ours_[AB].json", "21-ago REAL (los campos existen)")
corre("2026*_ours_[AB].json", "HISTORICO REAL",
      lambda d, f: d.get("sim_id") is None and not os.path.basename(f).startswith("202608 2"), limite=40)
