"""Extrae de los fotogramas REALES el recorte que produjo cada deteccion, con su geometria.

Para cada grupo de objeto de objetos_vistos.json busca el MEJOR avistamiento real:
  - fotograma cuyo t case con una muestra que tenga dets de esa etiqueta
  - se reejecuta yolo11x para obtener la CAJA (los dets guardados no la traen)
  - se empareja caja<->det por etiqueta y rumbo (rumbo = -atan2(u-cx, fx))
  - el det da el RANGO -> posicion en el mapa -> grupo al que pertenece
  - de la caja + rango salen ANCHO y ALTO fisicos y la altura del centro sobre el suelo
Se guarda el recorte PNG y su ficha. Criterio: mayor confianza, y a igualdad, mas cerca
(mas pixeles = mejor textura).
"""
import json, glob, math, os, re
import collections

import numpy as np
from PIL import Image
from ultralytics import YOLO

# INTRINSECOS: los del servidor (fx=600,cx=320,cy=240) son para 640x480. Los fotogramas
# reales son 320x180 (16:9), asi que se re-derivan del ancho: fx = 600*W/640, y el eje optico
# es el CENTRO del fotograma (cy = H/2), NO 240*W/640 -- ese error metia 5.7 grados de
# elevacion y hacia FLOTAR todos los objetos.
FX_640, CAM_H = 600.0, 1.10
CAM_PITCH = float(__import__("os").environ.get("PITCH", "-10.0"))
CLASES = ("chair", "couch", "refrigerator")
SALIDA = "/home/ros/isaac_ws/recortes"
os.makedirs(SALIDA, exist_ok=True)

grupos = json.load(open("/home/ros/isaac_ws/objetos_vistos.json"))
lista = [(lab, o["x"], o["y"], o["n"]) for lab, ol in grupos.items() if lab in CLASES
         for o in ol if o["n"] >= 12]
print("grupos objetivo:", len(lista))

modelo = YOLO("yolo11x.pt")
mejor = {}          # (lab, gx, gy) -> dict

runs = sorted(glob.glob("dataset/2026*_ours_[AB].json"))
n_frames = 0
for rf in runs:
    try:
        d = json.load(open(rf))
    except Exception:
        continue
    if d.get("sim_id"):
        continue
    base = os.path.basename(rf)[:-5]
    # muestras con dets de nuestras clases, indexadas por t redondeado
    porT = {}
    for m in d.get("samples") or []:
        if not m.get("dets") or m.get("x") is None:
            continue
        if not any(dd[0] in CLASES for dd in m["dets"]):
            continue
        porT[int(round(m["t"]))] = m
    if not porT:
        continue
    for fr in sorted(glob.glob("dataset/%s_t*.jpg" % base)):
        mt = re.search(r"_t(\d+)s\.jpg$", fr)
        if not mt:
            continue
        t = int(mt.group(1))
        cand = porT.get(t) or porT.get(t - 1) or porT.get(t + 1)
        if cand is None:
            continue
        n_frames += 1
        try:
            img = Image.open(fr).convert("RGB")
        except Exception:
            continue
        W, H = img.size
        fx = FX_640 * W / 640.0
        cx_f, cy_f = W / 2.0, H / 2.0
        res = modelo.predict(np.asarray(img), conf=0.45, verbose=False)[0]
        for b in res.boxes:
            lab = res.names[int(b.cls)].lower()
            if lab not in CLASES:
                continue
            conf = float(b.conf)
            x1, y1, x2, y2 = [float(v) for v in b.xyxy[0].tolist()]
            u = 0.5 * (x1 + x2)
            brg = -math.degrees(math.atan2(u - cx_f, fx))
            # emparejar con el det guardado (misma etiqueta, rumbo cercano) para tomar el RANGO
            rango = None
            for dd in cand["dets"]:
                if dd[0] == lab and dd[2] is not None and abs(float(dd[2]) - brg) < 8.0:
                    rango = float(dd[3]) if dd[3] is not None else None
                    break
            if rango is None or not (0.6 < rango < 4.5):
                continue
            a = math.radians(cand["yaw"] + brg)
            wx, wy = cand["x"] + rango * math.cos(a), cand["y"] + rango * math.sin(a)
            # a que grupo pertenece
            gl = None
            for (glab, gx, gy, gn) in lista:
                if glab == lab and math.hypot(wx - gx, wy - gy) < 0.9:
                    gl = (glab, gx, gy)
                    break
            if gl is None:
                continue
            clave = gl
            prev = mejor.get(clave)
            puntos = conf - 0.02 * rango          # mas confianza y mas cerca
            if prev and prev["puntos"] >= puntos:
                continue
            # geometria fisica del objeto a partir de la caja
            ang_w = 2 * math.atan((x2 - x1) / 2.0 / fx)
            ancho = 2 * rango * math.tan(ang_w / 2)
            # elevaciones de los bordes superior e inferior (con el cabeceo de camara)
            def elev(v):
                return math.degrees(math.atan2(cy_f - v, fx)) + CAM_PITCH
            z_top = CAM_H + rango * math.tan(math.radians(elev(y1)))
            z_bot = CAM_H + rango * math.tan(math.radians(elev(y2)))
            mejor[clave] = {"puntos": puntos, "lab": lab, "conf": round(conf, 3),
                            "rango": round(rango, 2), "frame": os.path.basename(fr),
                            "box": [round(v, 1) for v in (x1, y1, x2, y2)],
                            "pos": [round(gl[1], 2), round(gl[2], 2)],
                            # posicion implicada por ESTA observacion: la tarjeta va aqui, no
                            # en el centroide del grupo (que promedia muchas vistas y deja la
                            # distancia camara-tarjeta distinta de la que produjo el recorte)
                            "pos_obs": [round(wx, 2), round(wy, 2)],
                            "ancho_m": round(ancho, 2),
                            "alto_m": round(max(0.25, z_top - z_bot), 2),
                            # el objeto SE APOYA EN EL SUELO (silla/sofa/caja): se conserva la
                            # EXTENSION medida y se posa en z=0, en vez de fiarse del offset
                            # absoluto (que depende del cabeceo de camara, no calibrado).
                            "z_top": round(z_top, 2), "z_bot": round(z_bot, 2),
                            "observador": [round(cand["x"], 2), round(cand["y"], 2)],
                            # el YAW del robot en ese instante: para reproducir el ENCUADRE
                            # exacto (el objeto descentrado, con su contexto), no una mirada
                            # centrada -- YOLO es muy sensible a eso.
                            "obs_yaw": round(float(cand["yaw"]), 1),
                            "brg": round(brg, 1)}
            # MARGEN DE CONTEXTO: el recorte ajustado a la caja le quita a YOLO el suelo y la
            # pared que usa para clasificar (sintoma medido: un sofa recortado al ras se
            # convierte en "bed"). Se amplia un 60% por lado y se anota, para que la tarjeta
            # se dimensione con la region AMPLIADA y el objeto conserve su tamano angular.
            MG = 0.60
            bw, bh = x2 - x1, y2 - y1
            ex1, ey1 = max(0, x1 - MG*bw), max(0, y1 - MG*bh)
            ex2, ey2 = min(W, x2 + MG*bw), min(H, y2 + MG*bh)
            mejor[clave]["ancho_m"] = round(ancho * (ex2 - ex1) / max(bw, 1e-6), 2)
            mejor[clave]["alto_m"] = round(max(0.25, z_top - z_bot) * (ey2 - ey1) / max(bh, 1e-6), 2)
            mejor[clave]["margen"] = MG
            crop = img.crop((ex1, ey1, ex2, ey2))
            nombre = "%s_%.2f_%.2f.png" % (lab, gl[1], gl[2])
            crop.save(os.path.join(SALIDA, nombre))
            mejor[clave]["png"] = nombre

print("fotogramas examinados:", n_frames)
print("grupos con recorte:", len(mejor))
fichas = []
for (lab, gx, gy), v in sorted(mejor.items()):
    v.pop("puntos", None)
    fichas.append(v)
    print("  %-13s (%+.2f,%+.2f) conf %.2f a %.1fm | %.2f x %.2f m | visto desde (%.1f,%.1f)" % (
        lab, gx, gy, v["conf"], v["rango"], v["ancho_m"], v["alto_m"],
        v["observador"][0], v["observador"][1]))
json.dump(fichas, open("/home/ros/isaac_ws/recortes/fichas.json", "w"), indent=1)

# --- diagnostico de apoyo: un objeto de suelo debe dar z_bot ~ 0 ---
bots = [v["z_bot"] for v in fichas]
if bots:
    bots_s = sorted(bots)
    print("\nz_bot (base de los objetos): mediana %.2f m  min %.2f  max %.2f" % (
        bots_s[len(bots_s)//2], bots_s[0], bots_s[-1]))
    print("(con PITCH=%.1f) -- si la mediana no esta cerca de 0, el cabeceo esta mal" % CAM_PITCH)
