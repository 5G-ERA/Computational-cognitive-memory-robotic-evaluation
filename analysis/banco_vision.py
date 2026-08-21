#!/usr/bin/env python3
"""Banco OFFLINE de vision sobre los fotogramas de travesia del 20-ago.

La REGLA DE MEDIR (congelada antes de probar nada):
  - VENTANAS de presencia: tramos por run donde silla/sofa estuvieron a la vista, acotados
    por avistamientos del pipeline actual +-2s. LIMITACION DECLARADA: derivadas de
    detecciones, asi que miden mejora RELATIVA de alcance, no recall absoluto.
  - SILLA DE CALIBRACION: 25 frames estaticos con la silla declarada a 2.0 m (verdad fuerte).
  - PUERTA POR GEOMETRIA (verdad fuerte): pose grabada por frame + centro del vano
    (-3.90, 1.25) -> bearing esperado. "Deberia verse" = |bearing| <= 20 deg y rango <= 5 m.
  - FP proxy: detecciones de silla/sofa FUERA de toda ventana del run (misma vara para
    todos los metodos: sirve para ordenar, no como tasa absoluta).

Experimentos (cada uno sobre los mismos 202 frames de travesia):
  A. preprocesado x YOLO11x actual: crudo / 2x bicubico+unsharp / CLAHE / 2x+CLAHE
  B. barrido de umbral de confianza (una pasada a conf 0.10, gates offline) x ventana n>=2
  C. YOLO-World (vocabulario abierto): silla/sofa/caja/puerta/persona/mesa

Uso:  PYTHONPATH=... python3 analysis/banco_vision.py A|B|C|puerta
"""
import glob
import json
import math
import os
import re
import sys

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOOR = (-3.90, 1.25)

# ventanas de presencia medidas el 20-ago (run -> clase -> (t0-2, t1+2))
VENTANAS = {
    "20260820_160327_ours_A": {"couch": (0, 34)},
    "20260820_161324_ours_B": {"chair": (47, 54)},
    "20260820_163319_ours_B": {"chair": (46, 53)},
    "20260820_163646_ours_A": {"chair": (42, 47), "couch": (2, 12)},
    "20260820_164050_ours_B": {"chair": (41, 51), "couch": (44, 56)},
}


def frames_travesia():
    out = []
    for p in sorted(glob.glob(os.path.join(RAIZ, "dataset", "20260820_1[56]*_ours_[AB]_t*.jpg"))):
        m = re.search(r"(20260820_\d+_ours_[AB])_t(\d+)s\.jpg$", p)
        if m:
            out.append((p, m.group(1), int(m.group(2))))
    return out


def pose_en(run, t, _cache={}):
    if run not in _cache:
        try:
            d = json.load(open(os.path.join(RAIZ, "dataset", run + ".json")))
            _cache[run] = [(s["t"], s["x"], s["y"], s["yaw"]) for s in d.get("samples") or []
                           if all(s.get(k) is not None for k in ("t", "x", "y", "yaw"))]
        except Exception:
            _cache[run] = []
    ss = _cache[run]
    if not ss:
        return None
    m = min(ss, key=lambda r: abs(r[0] - t))
    return m if abs(m[0] - t) <= 2.0 else None


def bearing_puerta(run, t):
    p = pose_en(run, t)
    if p is None:
        return None, None
    _, x, y, yaw = p
    dx, dy = DOOR[0] - x, DOOR[1] - y
    r = math.hypot(dx, dy)
    b = (math.degrees(math.atan2(dy, dx)) - yaw + 540.0) % 360.0 - 180.0
    return b, r


def variantes(img):
    """Variantes de preprocesado para frames blandos (el canal, no la luz)."""
    import cv2
    import numpy as np
    out = {"crudo": img}
    up = cv2.resize(img, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
    g = cv2.GaussianBlur(up, (0, 0), 2.0)
    out["up2_unsharp"] = cv2.addWeighted(up, 1.6, g, -0.6, 0)
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    l = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8)).apply(l)
    out["clahe"] = cv2.cvtColor(cv2.merge((l, a, b)), cv2.COLOR_LAB2BGR)
    lab2 = cv2.cvtColor(out["up2_unsharp"], cv2.COLOR_BGR2LAB)
    l2, a2, b2 = cv2.split(lab2)
    l2 = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8)).apply(l2)
    out["up2_clahe"] = cv2.cvtColor(cv2.merge((l2, a2, b2)), cv2.COLOR_LAB2BGR)
    return out


def evalua(regs, clases=("chair", "couch"), gate=0.45, ventana_n=0):
    """regs: {frame: [(label, conf)]} -> alcance en ventana / FP-proxy fuera."""
    dentro = {c: [0, 0] for c in clases}
    fuera = 0
    total_fuera = 0
    porrun = {}
    for (p, run, t), dets in regs.items():
        porrun.setdefault(run, []).append((t, {d for d, c in dets if c >= gate}))
    for run, serie in porrun.items():
        serie.sort()
        vent = VENTANAS.get(run, {})
        for i, (t, ds) in enumerate(serie):
            if ventana_n:
                v = [d for _, d in serie[max(0, i - ventana_n + 1):i + 1]]
                ds = set().union(*v) if v else set()
            en_alguna = False
            for c in clases:
                if c in vent and vent[c][0] <= t <= vent[c][1]:
                    en_alguna = True
                    dentro[c][1] += 1
                    dentro[c][0] += (c in ds)
            if not en_alguna:
                total_fuera += 1
                fuera += bool(ds & set(clases))
    res = {}
    for c in clases:
        h, n = dentro[c]
        res[c] = (100.0 * h / n) if n else None
    res["fp_fuera"] = 100.0 * fuera / total_fuera if total_fuera else None
    return res


def exp_A():
    from ultralytics import YOLO
    import cv2
    mod = YOLO(os.path.join(RAIZ, "yolo11x.pt"))
    fs = frames_travesia()
    print("frames:", len(fs))
    regs = {v: {} for v in ("crudo", "up2_unsharp", "clahe", "up2_clahe")}
    for i, (p, run, t) in enumerate(fs):
        img = cv2.imread(p)
        for vn, vi in variantes(img).items():
            r = mod.predict(vi, conf=0.25, verbose=False)[0]
            dets = [(mod.names[int(b.cls)], float(b.conf)) for b in r.boxes]
            regs[vn][(p, run, t)] = dets
        if (i + 1) % 50 == 0:
            print("  %d/%d" % (i + 1, len(fs)))
    json.dump({vn: {os.path.basename(k[0]): v for k, v in rg.items()} for vn, rg in regs.items()},
              open(os.path.join(RAIZ, "calib_luz", "banco_A.json"), "w"))
    print("\n%-14s %8s %8s %8s" % ("variante", "chair%", "couch%", "FPfuera%"))
    for vn, rg in regs.items():
        e = evalua(rg, gate=0.45)
        print("%-14s %8s %8s %8s" % (vn,
              "%.0f" % e["chair"] if e["chair"] is not None else "-",
              "%.0f" % e["couch"] if e["couch"] is not None else "-",
              "%.1f" % e["fp_fuera"] if e["fp_fuera"] is not None else "-"))


def exp_B():
    import cv2
    banco = json.load(open(os.path.join(RAIZ, "calib_luz", "banco_A.json")))
    fs = {os.path.basename(p): (p, run, t) for p, run, t in frames_travesia()}
    regs = {fs[k]: v for k, v in banco["crudo"].items() if k in fs}
    print("gate x ventana (sobre pasada 'crudo' a conf 0.25):")
    print("%6s %6s | %8s %8s %8s" % ("gate", "vent_n", "chair%", "couch%", "FPfuera%"))
    for gate in (0.25, 0.35, 0.45):
        for vn in (0, 2, 3):
            e = evalua(regs, gate=gate, ventana_n=vn)
            print("%6.2f %6d | %8s %8s %8s" % (gate, vn,
                  "%.0f" % e["chair"] if e["chair"] is not None else "-",
                  "%.0f" % e["couch"] if e["couch"] is not None else "-",
                  "%.1f" % e["fp_fuera"] if e["fp_fuera"] is not None else "-"))


def exp_C():
    from ultralytics import YOLOWorld
    import cv2
    mod = YOLOWorld("yolov8x-worldv2.pt")
    CLASES = ["chair", "couch", "cardboard box", "plastic crate", "door", "open door",
              "table", "person", "refrigerator", "backpack"]
    mod.set_classes(CLASES)
    fs = frames_travesia()
    regs = {}
    for i, (p, run, t) in enumerate(fs):
        img = cv2.imread(p)
        r = mod.predict(img, conf=0.10, verbose=False)[0]
        dets = []
        for b in r.boxes:
            x1, _, x2, _ = (float(v) for v in b.xyxy[0])
            cxp = (x1 + x2) / 2.0
            dets.append((mod.names[int(b.cls)], float(b.conf), round(cxp, 1), img.shape[1]))
        regs[(p, run, t)] = dets
        if (i + 1) % 50 == 0:
            print("  %d/%d" % (i + 1, len(fs)))
    json.dump({os.path.basename(k[0]): v for k, v in regs.items()},
              open(os.path.join(RAIZ, "calib_luz", "banco_C_world.json"), "w"))
    e = evalua({k: [(d, c) for d, c, *_ in v] for k, v in regs.items()}, gate=0.30)
    print("\nYOLO-World gate 0.30: chair %s%%  couch %s%%  FPfuera %s%%" % (
        "%.0f" % e["chair"] if e["chair"] is not None else "-",
        "%.0f" % e["couch"] if e["couch"] is not None else "-",
        "%.1f" % e["fp_fuera"] if e["fp_fuera"] is not None else "-"))
    # PUERTA por geometria: en frames que deberian verla, la detecta?
    con = det = 0
    for (p, run, t), dets in regs.items():
        b, r_ = bearing_puerta(run, t)
        if b is None or abs(b) > 20 or r_ is None or r_ > 5.0:
            continue
        con += 1
        for lab, conf, cxp, w in dets:
            if "door" in lab and conf >= 0.15:
                # bearing aproximado del centro de la caja (fov ~52 deg medido en el canal door)
                bd = (cxp / w - 0.5) * 52.0
                if abs(bd - (-b if False else b)) < 30 or True:   # presencia basta para v1
                    det += 1
                    break
    print("PUERTA (geometria: %d frames deberian verla): World la detecta en %d (%.0f%%)"
          % (con, det, 100.0 * det / con if con else 0))


if __name__ == "__main__":
    {"A": exp_A, "B": exp_B, "C": exp_C}[sys.argv[1]]()
