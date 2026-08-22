"""Veredicto del gate con el mapa limpio y misma mezcla de direcciones en ambos brazos."""
import json, math, statistics

DX, DY = -3.90, 1.25
AX = math.radians(135.0); UX, UY = math.cos(AX), math.sin(AX)
def lat(p): return -(p["x"]-DX)*UY + (p["y"]-DY)*UX
def srob(p): return (p["x"]-DX)*UX + (p["y"]-DY)*UY

OFF = ["20260822_183111_ours_A", "20260822_183124_ours_B",
       "20260822_183153_ours_A", "20260822_183209_ours_B"]
ON = ["20260822_183237_ours_A", "20260822_183250_ours_B",
      "20260822_183322_ours_A", "20260822_183337_ours_B"]

def mide(b):
    d = json.load(open("dataset/%s.json" % b))
    ss = d.get("samples") or []
    if len(ss) < 20:
        return None
    vano = [lat(m) for m in ss if abs(srob(m)) <= 0.25]
    cerca = [m for m in ss if abs(srob(m)) <= 1.5]
    engc = sum(1 for m in cerca if "ENG-C" in str(m.get("phase", "")))
    dbs = [m.get("door_b") for m in cerca if isinstance(m.get("door_b"), (int, float))]
    gat = [m.get("dvis_gate") for m in ss if m.get("dvis_gate") is not None]
    return {"lat": statistics.median(vano) if vano else None, "engc": engc,
            "door_n": len(dbs), "gated": sum(1 for v in gat if v == 1),
            "col": (d.get("summary") or {}).get("collisions"),
            "dir": "A" if b.endswith("_A") else "B"}

print("%-6s %-4s %10s %7s %9s %7s" % ("brazo", "dir", "lat vano", "ENG-C", "door_b n", "gated"))
res = {"OFF": [], "ON": []}
cols = {"OFF": 0, "ON": 0}
for et, lista in (("OFF", OFF), ("ON", ON)):
    for b in lista:
        m = mide(b)
        if not m or m["lat"] is None:
            print("%-6s  --  (sin muestras en el vano: %s)" % (et, b)); continue
        res[et].append(abs(m["lat"])); cols[et] += (m["col"] or 0)
        print("%-6s %-4s %+10.3f %7d %9d %7d" % (
            et, m["dir"], m["lat"], m["engc"], m["door_n"], m["gated"]))
print()
for et in ("OFF", "ON"):
    v = res[et]
    if v:
        print("gate %-4s |lateral| : %s | mediana %.3f | media %.3f | colisiones %d" % (
            et, " ".join("%.3f" % x for x in v), statistics.median(v),
            sum(v)/len(v), cols[et]))
if res["OFF"] and res["ON"]:
    a, b = statistics.median(res["OFF"]), statistics.median(res["ON"])
    print("\nEFECTO DEL GATE: %.3f -> %.3f m  (%s%.0f%%)" % (
        a, b, "-" if b < a else "+", abs(100*(a-b)/a) if a else 0))
print("referencia REAL: toda-luz +0.130 con golpe | oscuro -0.001")
