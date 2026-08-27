#!/usr/bin/env python3
"""Revisa una captura de reconstruccion: que se cubrio y que falto.

Contar fotos no dice si la captura sirve. Lo que importa es la COBERTURA: por
donde se paso, que vio el laser y desde cuantos sitios distintos se miro cada
zona -- lo que se reconstruye mal es lo que nadie miro desde dos angulos.

    python3 tools/revisa_captura.py                       # la ultima captura
    python3 tools/revisa_captura.py dataset/reconstruccion/2026...

Escribe `cobertura.png` dentro de la propia carpeta: trayectoria + nube del
laser + los puntos de vista de las fotos. Sin robot: solo lee ficheros.
"""
import glob
import json
import math
import os
import sys

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def carga(d):
    def jl(n):
        p = os.path.join(d, n)
        return [json.loads(l) for l in open(p)] if os.path.exists(p) else []
    meta = {}
    if os.path.exists(os.path.join(d, "meta.json")):
        meta = json.load(open(os.path.join(d, "meta.json")))
    return meta, jl("frames.jsonl"), jl("poses.jsonl"), jl("nube.jsonl")


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else None
    if not d:
        cs = sorted(glob.glob(os.path.join(RAIZ, "dataset", "reconstruccion", "*")))
        if not cs:
            print("no hay capturas en dataset/reconstruccion/")
            return 1
        d = cs[-1]
    if not os.path.isdir(d):
        print("no existe: %s" % d)
        return 1

    meta, fr, po, nu = carga(d)
    print("CAPTURA %s" % os.path.basename(d.rstrip("/")))
    if meta.get("nota"):
        print("  nota: %s" % meta["nota"])
    print("  %.1f min | %d fotos (%d repetidas) | %d poses | %d nubes"
          % (meta.get("duracion_s", 0) / 60.0, meta.get("fotos", 0),
             meta.get("fotos_repetidas", 0), len(po), len(nu)))

    # --- cobertura del recorrido ---
    if po:
        xs = [p["x"] for p in po]
        ys = [p["y"] for p in po]
        print("  recorrido %.1f m | zona pisada x[%.1f, %.1f] y[%.1f, %.1f] (%.1f x %.1f m)"
              % (meta.get("recorrido_m", 0), min(xs), max(xs), min(ys), max(ys),
                 max(xs) - min(xs), max(ys) - min(ys)))
        malas = [p for p in po if p.get("src") != "slam_info"]
        if malas:
            print("  AVISO: %d de %d poses NO relocalizadas (%.0f%%) -- esos tramos "
                  "no valen para reconstruir" % (len(malas), len(po),
                                                 100.0 * len(malas) / len(po)))

    # --- que vio el laser ---
    pts = [q for n in nu for q in n.get("pts", [])]
    if pts:
        px = [q[0] for q in pts]
        py = [q[1] for q in pts]
        pz = [q[2] for q in pts]
        print("  laser: %d puntos | x[%.1f, %.1f] y[%.1f, %.1f] z[%.2f, %.2f]"
              % (len(pts), min(px), max(px), min(py), max(py), min(pz), max(pz)))
    else:
        print("  laser: SIN PUNTOS -- la captura no tiene geometria")

    # --- angulos de vista por celda: lo que de verdad decide si reconstruye ---
    OC = 0.5
    vistas = {}
    for f in fr:
        if f.get("dup") or f.get("x") is None:
            continue
        c = (round(f["x"] / OC), round(f["y"] / OC))
        vistas.setdefault(c, []).append(f.get("yaw", 0.0))
    if vistas:
        multi = 0
        for c, ys_ in vistas.items():
            if len(ys_) < 2:
                continue
            span = max(ys_) - min(ys_)
            if span > 40.0 or span < -40.0:
                multi += 1
        print("  puntos de vista: %d celdas de %.1f m fotografiadas, %d desde "
              "angulos distintos (>40 deg)" % (len(vistas), OC, multi))

    # --- dibujo ---
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        print("\n(sin matplotlib: %s -- no dibujo el mapa)" % type(e).__name__)
        return 0

    fig, ax = plt.subplots(figsize=(9, 8))
    if pts:
        ax.scatter(px, py, s=0.7, c="#9fb3bb", label="laser (geometria)", linewidths=0)
    if po:
        ax.plot(xs, ys, "-", c="#0F6E77", lw=1.6, label="recorrido")
    for f in fr:
        if f.get("dup") or f.get("x") is None:
            continue
        yaw = math.radians(f.get("yaw", 0.0))
        ax.arrow(f["x"], f["y"], 0.28 * math.cos(yaw), 0.28 * math.sin(yaw),
                 head_width=0.10, color="#9E4E2C", alpha=.75, length_includes_head=True)
    ax.plot([], [], color="#9E4E2C", label="fotos (hacia donde miran)")
    ax.set_aspect("equal")
    ax.grid(alpha=.25, lw=.5)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Cobertura de la captura %s" % os.path.basename(d.rstrip("/")))
    ax.legend(loc="upper right", fontsize=9)
    dst = os.path.join(d, "cobertura.png")
    fig.savefig(dst, dpi=110, bbox_inches="tight")
    print("\n  mapa -> %s" % os.path.relpath(dst, RAIZ))
    return 0


if __name__ == "__main__":
    sys.exit(main())
