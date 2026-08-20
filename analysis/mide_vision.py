#!/usr/bin/env python3
"""Bateria de medidas de la vision sobre los fotogramas ya grabados (pelicula 1s + calibracion).

Para cada fotograma: estadisticos de imagen (brillo/contraste/grano + NITIDEZ por varianza del
laplaciano, que separa desenfoque de movimiento de falta de luz), la respuesta completa de
/perceive, y -- para los frames de pelicula t###s -- la velocidad del robot en ese instante
(muestras del run). Cachea en JSONL para no repetir GPU.

    python3 analysis/mide_vision.py            # procesa lo que falte y resume
"""
import base64
import glob
import io
import json
import os
import re
import sys
import urllib.request

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PERC = "127.0.0.1:8008"
CACHE = os.path.join(RAIZ, "calib_luz", "medidas_vision_20260820.jsonl")


def stats(im):
    from PIL import ImageStat, ImageFilter
    import numpy as np
    g = im.convert("L")
    st = ImageStat.Stat(g)
    grano = ImageStat.Stat(g.filter(ImageFilter.FIND_EDGES)).mean[0]
    a = np.asarray(g, dtype=np.float32)
    lap = (a[1:-1, 2:] + a[1:-1, :-2] + a[2:, 1:-1] + a[:-2, 1:-1] - 4 * a[1:-1, 1:-1])
    return {"brillo": round(st.mean[0], 1), "contraste": round(st.stddev[0], 1),
            "grano": round(grano, 1), "nitidez": round(float(lap.var()), 1)}


def pide(jpg):
    d = json.dumps({"image": "data:image/jpeg;base64," + base64.b64encode(jpg).decode()}).encode()
    r = urllib.request.urlopen(urllib.request.Request(
        "http://%s/perceive" % PERC, d, {"Content-Type": "application/json"}), timeout=120)
    o = json.loads(r.read())
    dets = o.get("detections") or []
    return {"dets": [[x.get("label"), round(float(x.get("conf", 0)), 2),
                      x.get("bearing_deg"), x.get("range_m")] for x in dets][:8],
            "free_center": o.get("free_center"),
            "n_scan": len(o.get("scan") or []),
            "color_pts": o.get("color_pts"), "carpet_pct": o.get("carpet_pct")}


def vel_en(run_json, t):
    try:
        d = json.load(open(run_json))
    except Exception:
        return None
    ss = d.get("samples") or []
    m = min(ss, key=lambda s: abs(s.get("t", 1e9) - t), default=None)
    if m is None or abs(m.get("t", 1e9) - t) > 2.0:
        return None
    return m.get("spd")


def main():
    from PIL import Image
    hechas = set()
    if os.path.exists(CACHE):
        for l in open(CACHE):
            try:
                hechas.add(json.loads(l)["fichero"])
            except Exception:
                pass
    fotos = sorted(glob.glob(os.path.join(RAIZ, "dataset", "20260820_1[56]*_ours_[AB]_*.jpg")))
    fotos += sorted(glob.glob(os.path.join(RAIZ, "calib_luz", "2026-08-20", "*.jpg")))
    print("fotogramas: %d (cacheados %d)" % (len(fotos), len(hechas)))
    w = open(CACHE, "a")
    for i, p in enumerate(fotos):
        base = os.path.basename(p)
        if base in hechas:
            continue
        try:
            raw = open(p, "rb").read()
            im = Image.open(io.BytesIO(raw)).convert("RGB")
        except Exception:
            continue
        fila = {"fichero": base, "origen": "pelicula" if "_ours_" in base else "calib",
                "imagen": stats(im)}
        m = re.search(r"_ours_([AB])_t(\d+)s\.jpg$", base)
        if m and "_ours_" in base:
            run = os.path.join(RAIZ, "dataset", base.rsplit("_t", 1)[0] + ".json")
            fila["t"] = int(m.group(2))
            fila["spd"] = vel_en(run, fila["t"])
        try:
            fila["perc"] = pide(raw)
        except Exception as e:
            fila["perc"] = {"error": str(e)[:80]}
        w.write(json.dumps(fila) + "\n")
        w.flush()
        if (i + 1) % 25 == 0:
            print("  %d/%d" % (i + 1, len(fotos)))
    w.close()
    print("cache completo: %s" % os.path.relpath(CACHE, RAIZ))


if __name__ == "__main__":
    main()
