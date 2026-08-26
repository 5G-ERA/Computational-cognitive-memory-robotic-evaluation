"""Curva REAL del detector: P(deteccion) y confianza por RANGO y BRILLO.

Las etiquetas de tanda son poco fiables (lotes con la etiqueta reutilizada); el BRILLO del
fotograma es la variable honesta y se usa para separar condiciones dentro de cada tanda.
"""
import json, glob, re, statistics
import collections

filas = []
for f in sorted(glob.glob("calib_luz/*/muestras.jsonl")):
    for l in open(f):
        if not l.strip():
            continue
        r = json.loads(l)
        et = r.get("etiqueta", "")
        m = re.search(r"(\d+\.?\d*)\s*m", et)
        if not m:
            continue
        dist = float(m.group(1))
        br = (r.get("imagen") or {}).get("brillo_medio")
        dets = (r.get("percepcion") or {}).get("detecciones") or []
        c = [d[1] for d in dets if d[0] == "chair"]
        filas.append({"dist": dist, "brillo": br, "conf": max(c) if c else None})

print("muestras escenificadas con distancia:", len(filas))
por = collections.defaultdict(list)
for r in filas:
    if r["brillo"] is None:
        continue
    luz = "luz" if r["brillo"] >= 108 else "poca"
    por[(r["dist"], luz)].append(r)

print("\n%-8s %-6s %5s %8s %10s %14s" % ("dist", "luz", "n", "brillo", "P(det)", "conf mediana"))
modelo = {}
for (d, luz), v in sorted(por.items()):
    n = len(v)
    si = [r for r in v if r["conf"] is not None]
    confs = [r["conf"] for r in si]
    p = len(si) / n
    cm = statistics.median(confs) if confs else None
    print("%-8.1f %-6s %5d %8.0f %10.2f %14s" % (
        d, luz, n, statistics.median([r["brillo"] for r in v]), p,
        "%.2f (min %.2f max %.2f)" % (cm, min(confs), max(confs)) if confs else "-"))
    modelo["%.1f_%s" % (d, luz)] = {"n": n, "p_det": round(p, 3),
                                    "conf_med": round(cm, 3) if cm else None,
                                    "conf_min": round(min(confs), 3) if confs else None,
                                    "conf_max": round(max(confs), 3) if confs else None,
                                    "brillo": round(statistics.median([r["brillo"] for r in v]))}
json.dump(modelo, open("/home/ros/isaac_ws/curva_detector.json", "w"), indent=1)
print("\nescrito curva_detector.json")
