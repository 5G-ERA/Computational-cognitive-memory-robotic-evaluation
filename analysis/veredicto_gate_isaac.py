import json, math, statistics

DX, DY = -3.90, 1.25
AX = math.radians(135.0); UX, UY = math.cos(AX), math.sin(AX)
def lat(p): return -(p["x"]-DX)*UY + (p["y"]-DY)*UX
def srob(p): return (p["x"]-DX)*UX + (p["y"]-DY)*UY

OFF = ["20260822_181303_ours_B", "20260822_181327_ours_B", "20260822_181438_ours_B"]
ON  = ["20260822_181304_ours_A", "20260822_181414_ours_A", "20260822_181518_ours_A"]

def mide(b):
    d = json.load(open("dataset/%s.json" % b))
    ss = d.get("samples") or []
    if len(ss) < 20:
        return None
    vano = [lat(m) for m in ss if abs(srob(m)) <= 0.25]
    cerca = [m for m in ss if abs(srob(m)) <= 1.5]
    dbs = [m.get("door_b") for m in cerca if isinstance(m.get("door_b"), (int, float))]
    gat = [m.get("dvis_gate") for m in ss if m.get("dvis_gate") is not None]
    return {"lat": statistics.median(vano) if vano else None, "n_vano": len(vano),
            "door_n": len(dbs), "gated": sum(1 for v in gat if v == 1), "n_gat": len(gat),
            "col": (d.get("summary") or {}).get("collisions")}

print("%-8s %-24s %9s %8s %8s %s" % ("gate", "run", "lat vano", "door_b n", "gated", "col"))
res = {"OFF": [], "ON": []}
for et, lista in (("OFF", OFF), ("ON", ON)):
    for b in lista:
        m = mide(b)
        if not m or m["lat"] is None:
            print("%-8s %-24s  (sin muestras en el vano)" % (et, b)); continue
        res[et].append(abs(m["lat"]))
        print("%-8s %-24s %+9.3f %8d %8s %s" % (
            et, b, m["lat"], m["door_n"], "%d/%d" % (m["gated"], m["n_gat"]), m["col"]))
print()
for et in ("OFF", "ON"):
    v = res[et]
    if v:
        print("gate %-4s desviacion |lateral| : %s  -> mediana %.3f m" % (
            et, " ".join("%.3f" % x for x in v), statistics.median(v)))
if res["OFF"] and res["ON"]:
    a, b = statistics.median(res["OFF"]), statistics.median(res["ON"])
    print("\nefecto del gate: %.3f -> %.3f m (%s%.0f%%)" % (
        a, b, "-" if b < a else "+", abs(100*(a-b)/a)))
print("referencia REAL: toda-luz +0.130 (con golpe) | oscuro -0.001")
