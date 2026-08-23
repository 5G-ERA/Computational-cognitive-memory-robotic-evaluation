"""Recalibracion de VSCALE y TAU CON EL FRENO PUESTO, contra las 132 runs reales.

Metodo: no hace falta simular. El cuerpo cinematico del puente son cinco lineas, asi que
se REPRODUCE offline la secuencia de ordenes REAL (cmd por muestra, con sus tiempos reales)
a traves del mismo modelo, y se compara la respuesta con la trayectoria que el robot real
produjo ante esas mismas ordenes. Mismo estimulo, se compara la salida.

Con el freno puesto, tiempo de simulacion = tiempo de pared, asi que el modelo offline con
DT=0.05 es exactamente lo que hara el puente. (Sin freno no lo seria: de ahi el acoplamiento.)

Se ajustan dos observables, no uno:
  - v/cmd  : cuanto del mando se convierte en avance  -> gobernado sobre todo por VSCALE
  - subida : fraccion de la velocidad alcanzada en el primer ciclo de mando (~0.4 s)
             -> gobernado sobre todo por TAU
"""
import glob, json, math, os, statistics

RAIZ = "/home/ros/Documents/G1_UNITREE_ROBOT_META_REASONING"
DT = 0.05
CADUCA = 0.6           # el puente caduca el cmd_vel a los 0.6 s

def carga(f):
    try: return json.load(open(f))
    except Exception: return None

def serie(d):
    """(t, lx, ly, rx) y la trayectoria real, de una run."""
    ss = d.get("samples") or []
    out = []
    for m in ss:
        c = m.get("cmd") or []
        if len(c) < 3 or not isinstance(m.get("t"), (int, float)):
            continue
        out.append((float(m["t"]), float(c[0]), float(c[1]), float(c[2]),
                    float(m["x"]), float(m["y"])))
    return out

def replay(sr, vscale, tau):
    """Integra el modelo del puente con las ordenes reales. Devuelve camino y duracion."""
    vx = vy = 0.0
    yaw = 0.0
    L = 0.0
    k = min(1.0, DT / max(1e-3, tau))
    t = sr[0][0]
    i = 0
    t_fin = sr[-1][0]
    t_cmd = t
    cx = cy = 0.0
    while t < t_fin:
        while i + 1 < len(sr) and sr[i + 1][0] <= t:
            i += 1
            cx, cy = sr[i][1], sr[i][2]
            t_cmd = sr[i][0]
        ex, ey = (0.0, 0.0) if (t - t_cmd) > CADUCA else (cx, cy)
        vx += (ex * vscale - vx) * k
        vy += (ey * vscale - vy) * k
        L += math.hypot(vx, vy) * DT
        t += DT
    return L, t_fin - sr[0][0]

def camino_real(sr):
    return sum(math.hypot(b[4]-a[4], b[5]-a[5]) for a, b in zip(sr, sr[1:]))

# --- material real ---
todos = [f for f in sorted(glob.glob(os.path.join(RAIZ, "dataset", "2026*_ours_[AB].json"))) if carga(f)]
series = []
for f in todos:
    d = carga(f)
    if d.get("sim_id") is not None:
        continue
    sr = serie(d)
    if len(sr) < 40:
        continue
    T = sr[-1][0] - sr[0][0]
    Lr = camino_real(sr)
    if T < 15 or Lr < 2:
        continue
    series.append((sr, Lr, T))
print("runs reales utilizables: %d" % len(series))
print("camino real mediano %.2f m | duracion mediana %.1f s | v real %.3f m/s"
      % (statistics.median(s[1] for s in series), statistics.median(s[2] for s in series),
         statistics.median(s[1]/s[2] for s in series)))

def error(vscale, tau, muestra):
    """Error relativo mediano de camino recorrido: el gemelo debe recorrer lo mismo que el
    robot real ante las MISMAS ordenes."""
    rel = []
    for sr, Lr, T in muestra:
        Ls, _ = replay(sr, vscale, tau)
        rel.append(Ls / Lr)
    return statistics.median(rel)

muestra = series[::3]          # una de cada tres: suficiente y rapido
print("muestra de ajuste: %d runs\n" % len(muestra))

print("actual (VSCALE=0.62, TAU=0.55): camino gemelo / camino real = %.3f" % error(0.62, 0.55, muestra))
print()
print("  TAU \\ VSCALE" + "".join("%8.2f" % v for v in (0.7,0.8,0.9,1.0,1.1,1.2,1.3)))
mejor = None
for tau in (0.15, 0.25, 0.35, 0.45, 0.55, 0.75):
    fila = []
    for v in (0.7,0.8,0.9,1.0,1.1,1.2,1.3):
        r = error(v, tau, muestra)
        fila.append(r)
        if mejor is None or abs(r-1.0) < abs(mejor[0]-1.0):
            mejor = (r, v, tau)
    print("  %5.2f       " % tau + "".join("%8.3f" % x for x in fila))
print("\nmejor de la rejilla: VSCALE=%.2f TAU=%.2f -> razon %.3f" % (mejor[1], mejor[2], mejor[0]))

# afinado fino alrededor del mejor
r0, v0, t0 = mejor
fino = None
for tau in [t0 + d for d in (-0.06,-0.03,0,0.03,0.06) if t0 + d > 0.05]:
    for v in [v0 + d for d in (-0.08,-0.04,0,0.04,0.08)]:
        r = error(v, tau, muestra)
        if fino is None or abs(r-1.0) < abs(fino[0]-1.0):
            fino = (r, v, tau)
print("afinado: VSCALE=%.3f TAU=%.3f -> razon %.4f" % (fino[1], fino[2], fino[0]))

# validacion en las runs NO usadas para ajustar
resto = [s for s in series if s not in muestra]
print("\nvalidacion en %d runs no usadas: razon %.3f (actual: %.3f)"
      % (len(resto), error(fino[1], fino[2], resto), error(0.62, 0.55, resto)))
