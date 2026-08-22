"""El detector REAL juzga los renders de las tarjetas: ¿misma etiqueta, misma confianza?"""
import json
import numpy as np
from PIL import Image
from ultralytics import YOLO

m = YOLO("yolo11x.pt")
lista = json.load(open("/home/ros/isaac_ws/bench_tarjetas/lista.json"))
print("%-14s %-10s %-6s %-28s %s" % ("objeto", "pos", "real", "sim (top-2)", "veredicto"))
ok = casi = mal = 0
aprobadas = []
for e in lista:
    img = np.asarray(Image.open("/home/ros/isaac_ws/bench_tarjetas/" + e["png"]).convert("RGB"))
    r = m.predict(img, conf=0.25, verbose=False)[0]
    dets = sorted([(r.names[int(b.cls)].lower(), float(b.conf)) for b in r.boxes],
                  key=lambda t: -t[1])
    top = " ".join("%s:%.2f" % d for d in dets[:2]) or "—"
    igual = [c for l, c in dets if l == e["lab"]]
    if igual:
        dif = abs(igual[0] - e["conf_real"])
        ver = "OK" if dif <= 0.25 else "etiqueta ok, conf lejos (%.2f)" % dif
        if dif <= 0.25:
            aprobadas.append({"pos": e["pos"], "lab": e["lab"],
                              "conf_real": e["conf_real"], "conf_sim": round(igual[0], 2)})
        ok += 1 if dif <= 0.25 else 0
        casi += 1 if dif > 0.25 else 0
    else:
        ver = "FALLA (etiqueta distinta)"
        mal += 1
    print("%-14s %-10s %-6.2f %-28s %s" % (
        e["lab"], "%.1f,%.1f" % tuple(e["pos"]), e["conf_real"], top, ver))
print("\nRESUMEN: %d con misma etiqueta y confianza cercana | %d etiqueta ok pero conf lejos | %d fallan" % (
    ok, casi, mal))
json.dump(aprobadas, open("/home/ros/isaac_ws/recortes/aprobadas.json", "w"), indent=1)
print("aprobadas escritas:", len(aprobadas))
