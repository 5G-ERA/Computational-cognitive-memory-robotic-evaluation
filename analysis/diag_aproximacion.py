"""Geometria de aproximacion a la puerta: real vs gemelos, tramo a tramo."""
import json, math, statistics

DX, DY = -3.90, 1.25
AX = math.radians(135.0); UX, UY = math.cos(AX), math.sin(AX)
def lat(p): return -(p["x"]-DX)*UY + (p["y"]-DY)*UX
def srob(p): return (p["x"]-DX)*UX + (p["y"]-DY)*UY

GRUPOS = {
    "REAL":   ["20260821_160208_ours_A", "20260821_144604_ours_B",
               "20260821_145228_ours_A", "20260821_155536_ours_B"],
    "ISAAC":  ["20260822_181303_ours_B", "20260822_181327_ours_B", "20260822_181438_ours_B",
               "20260822_180519_ours_B"],
    "GAZEBO": ["20260822_181643_ours_B", "20260822_181739_ours_A"],
}
TRAMOS = [(2.5, 3.5, "lejos 2.5-3.5"), (1.5, 2.5, "medio 1.5-2.5"),
          (0.8, 1.5, "cerca 0.8-1.5"), (0.25, 0.8, "boca 0.25-0.8"),
          (0.0, 0.25, "VANO")]

print("%-8s %-16s %9s %9s %8s" % ("origen", "tramo", "lat med", "|lat|", "n"))
for et, bases in GRUPOS.items():
    for lo, hi, nom in TRAMOS:
        vals = []
        for b in bases:
            try:
                d = json.load(open("dataset/%s.json" % b))
            except Exception:
                continue
            ss = d.get("samples") or []
            if len(ss) < 20:
                continue
            vals += [lat(m) for m in ss if lo <= abs(srob(m)) < hi]
        if vals:
            print("%-8s %-16s %+9.3f %9.3f %8d" % (
                et, nom, statistics.median(vals),
                statistics.median([abs(v) for v in vals]), len(vals)))
    print()

# velocidad de aproximacion y yaw respecto al eje de la puerta
print("%-8s %10s %10s %10s" % ("origen", "vel cerca", "|yaw-eje|", "c0 cerca"))
for et, bases in GRUPOS.items():
    sp, ye, c0 = [], [], []
    for b in bases:
        try:
            d = json.load(open("dataset/%s.json" % b))
        except Exception:
            continue
        for m in d.get("samples") or []:
            s_ = abs(srob(m))
            if 0.25 <= s_ < 1.5:
                if isinstance(m.get("spd"), (int, float)): sp.append(m["spd"])
                if isinstance(m.get("c0"), (int, float)): c0.append(m["c0"])
                if m.get("yaw") is not None:
                    e = abs(((m["yaw"] - 135.0 + 180) % 360) - 180)
                    ye.append(min(e, abs(e - 180)))
    print("%-8s %10s %10s %10s" % (
        et, "%.2f" % statistics.median(sp) if sp else "-",
        "%.0f" % statistics.median(ye) if ye else "-",
        "%.2f" % statistics.median(c0) if c0 else "-"))
