"""El detector REAL (yolo11x, el del perception_server) juzga cada malla candidata."""
import glob, os, collections
from ultralytics import YOLO

m = YOLO("yolo11x.pt")
filas = collections.defaultdict(dict)
for f in sorted(glob.glob("/home/ros/isaac_ws/bench/*.jpg")):
    base = os.path.basename(f)[:-4]
    nombre, dist = base.rsplit("_", 1)
    r = m.predict(f, conf=0.25, verbose=False)[0]
    dets = []
    for b in r.boxes:
        dets.append((r.names[int(b.cls)], float(b.conf)))
    dets.sort(key=lambda t: -t[1])
    filas[nombre][float(dist)] = dets[:2]

print("%-18s %-22s %-22s %-22s %-22s" % ("malla", "1.0m", "1.5m", "1.8m", "2.5m"))
for nombre in ("SM_ChairOffice_A", "SM_Chair_01a", "SM_Armchair", "SM_BoxBigA", "SM_BoxA", "BLOQUE_GRIS"):
    cols = []
    for d in (1.0, 1.5, 1.8, 2.5):
        dets = filas[nombre].get(d, [])
        cols.append(" ".join("%s:%.2f" % (l, c) for l, c in dets) or "—")
    print("%-18s %-22s %-22s %-22s %-22s" % (nombre, *cols))
print()
print("FIRMAS REALES (medidas): silla->chair 0.92@1.5 luz / 0.82@1.8 luz;")
print("  sofas->couch 0.95+; cajas de carton->refrigerator ~0.9")
