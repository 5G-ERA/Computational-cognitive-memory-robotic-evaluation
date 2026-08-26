"""Comparacion OBJETIVA Isaac vs Gazebo, contra el mismo patron oro: el robot REAL."""
import json, math, statistics, collections, os

DX, DY = -3.90, 1.25
AX = math.radians(135.0); UX, UY = math.cos(AX), math.sin(AX)
def lat(p): return -(p["x"]-DX)*UY + (p["y"]-DY)*UX
def srob(p): return (p["x"]-DX)*UX + (p["y"]-DY)*UY

GRUPOS = {
    "ISAAC":  ["20260822_181303_ours_B", "20260822_181327_ours_B", "20260822_181438_ours_B",
               "20260822_180519_ours_B", "20260822_180658_ours_B"],
    "GAZEBO": ["20260822_181643_ours_B", "20260822_181739_ours_A", "20260822_172340_ours_B"],
    "REAL":   ["20260821_160208_ours_A", "20260821_144604_ours_B", "20260821_145228_ours_A",
               "20260821_155536_ours_B"],
}

def carga(b):
    try:
        return json.load(open("dataset/%s.json" % b))
    except Exception:
        return None

def metricas(bases):
    m = {"t": [], "err": [], "col": 0, "n": 0, "scan_n": [], "c0": [],
         "door_n": [], "lat": [], "engc": [], "dets": [], "conf": []}
    for b in bases:
        d = carga(b)
        if not d:
            continue
        ss = d.get("samples") or []
        if len(ss) < 20:
            continue
        m["n"] += 1
        s = d.get("summary") or {}
        if s.get("time_s"): m["t"].append(float(s["time_s"]))
        m["col"] += (s.get("collisions") or 0)
        # densidad del laser: nobs por muestra
        nobs = [x.get("nobs") for x in ss if isinstance(x.get("nobs"), (int, float))]
        if nobs: m["scan_n"].append(statistics.median(nobs))
        c0 = [x.get("c0") for x in ss if isinstance(x.get("c0"), (int, float))]
        if c0: m["c0"].append(statistics.median(c0))
        cerca = [x for x in ss if abs(srob(x)) <= 1.5]
        dbs = [x.get("door_b") for x in cerca if isinstance(x.get("door_b"), (int, float))]
        m["door_n"].append(len(dbs))
        m["engc"].append(sum(1 for x in cerca if "ENG-C" in str(x.get("phase", ""))))
        vano = [lat(x) for x in ss if abs(srob(x)) <= 0.25]
        if vano: m["lat"].append(abs(statistics.median(vano)))
        dd = [z for x in ss for z in (x.get("dets") or []) if z[0] != "door"]
        m["dets"].append(len(dd))
        m["conf"] += [z[1] for z in dd if isinstance(z[1], (int, float))]
    return m

def med(v):
    return statistics.median(v) if v else None

R = {k: metricas(v) for k, v in GRUPOS.items()}
def fila(nombre, clave, fmt="%.2f", trans=med):
    vals = []
    for k in ("REAL", "GAZEBO", "ISAAC"):
        v = trans(R[k][clave]) if isinstance(R[k][clave], list) else R[k][clave]
        vals.append(fmt % v if v is not None else "-")
    return "%-34s %10s %10s %10s" % (nombre, *vals)

print("%-34s %10s %10s %10s" % ("metrica", "REAL", "GAZEBO", "ISAAC"))
print("-" * 68)
print("%-34s %10d %10d %10d" % ("runs comparados", R["REAL"]["n"], R["GAZEBO"]["n"], R["ISAAC"]["n"]))
print(fila("duracion de travesia (s)", "t", "%.0f"))
print(fila("obstaculos por muestra (nobs)", "scan_n", "%.0f"))
print(fila("holgura frontal c0 (m)", "c0"))
print(fila("observaciones de vano (n)", "door_n", "%.0f"))
print(fila("fases ENG-C junto a la puerta", "engc", "%.0f"))
print(fila("|lateral| en el vano (m)", "lat", "%.3f"))
print(fila("detecciones de objeto por run", "dets", "%.0f"))
print(fila("confianza mediana de objeto", "conf"))
print("%-34s %10d %10d %10d" % ("colisiones (total)", R["REAL"]["col"], R["GAZEBO"]["col"], R["ISAAC"]["col"]))
