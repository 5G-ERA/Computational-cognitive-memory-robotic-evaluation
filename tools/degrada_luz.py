#!/usr/bin/env python3
"""Busca donde se rompe la vision degradando fotogramas REALES con verdad de terreno.

Usa las muestras estaticas del 20-ago: silla a 2.0 m, detectada 25 de 25 veces al 0.91 con luz
declarada. Se degradan progresivamente y se le vuelven a pasar al servidor de percepcion, para
encontrar el nivel al que deja de verla. Como sabemos que la silla ESTA, un fallo de deteccion es
inequivocamente "no puedo verla" y no "no hay nada" -- que es justo la distincion de W2.

EL MODELO DE DEGRADACION IMPORTA. Oscurecer los pixeles sin mas seria falso: la camara real sube
la ganancia cuando falta luz, y por eso el brillo medio se queda clavado cerca de 105 mientras la
imagen se vuelve ruidosa. Aqui se reproduce esa cadena:
    1. se reduce la luz que llega al sensor por un factor k
    2. se anade el ruido de disparo que corresponde a esa señal mas debil (crece como 1/sqrt(k))
    3. se renormaliza el brillo al valor original, que es lo que hace la ganancia automatica
El resultado tiene el MISMO brillo medio y peor relacion senal-ruido, que es lo que de verdad
distingue una escena mal iluminada de una bien iluminada en esta camara.

ESTO ES UN MODELO, NO UNA MEDIDA. Da un umbral PROVISIONAL para llevar a la sesion nocturna y
confirmarlo o refutarlo contra oscuridad real. Si la noche no lo reproduce, manda la noche.

    python3 tools/degrada_luz.py            # usa las muestras con luz declarada de hoy
"""
import base64
import glob
import io
import json
import os
import sys
import urllib.request

PERC = os.environ.get("G1_PERC", "127.0.0.1:8008")
NIVELES = [1.0, 0.60, 0.35, 0.20, 0.12, 0.07, 0.04, 0.02]     # fraccion de luz que llega
RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def degrada(im, k, rng):
    """Menos luz + ruido de disparo + ganancia automatica que devuelve el brillo."""
    import numpy as np
    a = np.asarray(im).astype(np.float32)
    m0 = a.mean()
    a = a * k                                     # 1) llega menos luz
    ruido = rng.normal(0.0, np.sqrt(np.maximum(a, 1.0)) * 0.9)   # 2) ruido de disparo
    a = a + ruido
    m = max(a.mean(), 1e-3)
    a = a * (m0 / m)                              # 3) la ganancia recupera el brillo
    return np.clip(a, 0, 255).astype("uint8")


def stats(arr):
    from PIL import Image, ImageStat, ImageFilter
    im = Image.fromarray(arr).convert("L")
    st = ImageStat.Stat(im)
    grano = ImageStat.Stat(im.filter(ImageFilter.FIND_EDGES)).mean[0]
    h = im.histogram()
    n = sum(h) or 1
    return (round(st.mean[0], 1), round(st.stddev[0], 1), round(grano, 1),
            round(100.0 * sum(h[:32]) / n, 1))


def pide(arr):
    from PIL import Image
    b = io.BytesIO()
    Image.fromarray(arr).save(b, format="JPEG", quality=85)
    d = json.dumps({"image": "data:image/jpeg;base64,"
                    + base64.b64encode(b.getvalue()).decode()}).encode()
    r = urllib.request.urlopen(urllib.request.Request(
        "http://%s/perceive" % PERC, d, {"Content-Type": "application/json"}), timeout=120)
    o = json.loads(r.read())
    dets = o.get("detections") or []
    silla = max([float(x.get("conf", 0)) for x in dets if x.get("label") == "chair"], default=0.0)
    return silla, len(dets), len(o.get("scan") or [])


def main():
    import numpy as np
    from PIL import Image
    rng = np.random.default_rng(7)

    reg = sorted(glob.glob(os.path.join(RAIZ, "calib_luz", "*", "muestras.jsonl")))[-1]
    dia = os.path.dirname(reg)
    filas = [json.loads(l) for l in open(reg) if l.strip()]
    # las que tienen la silla declarada y detectada: son la verdad de terreno
    base = [f for f in filas
            if "silla" in f["etiqueta"].lower()
            and any(d[0] == "chair" for d in ((f.get("percepcion") or {}).get("detecciones") or []))]
    if not base:
        sys.exit("no encuentro muestras con la silla detectada")
    usar = base[::max(1, len(base) // 4)][:4]
    print("fotogramas de partida: %d (de %d con la silla detectada)\n" % (len(usar), len(base)))

    print("%6s %8s %9s %8s %9s %14s" % ("luz", "brillo", "contraste", "grano", "oscuros", "silla"))
    for k in NIVELES:
        confs = []
        est = []
        for f in usar:
            p = os.path.join(dia, f["fichero"])
            if not os.path.exists(p):
                continue
            arr = degrada(Image.open(p).convert("RGB"), k, rng)
            est.append(stats(arr))
            try:
                c, _, _ = pide(arr)
            except Exception as e:
                print("   error en /perceive: %s" % e)
                c = -1.0
            confs.append(c)
        if not confs:
            continue
        vistas = sum(1 for c in confs if c > 0)
        med = [sum(x[i] for x in est) / len(est) for i in range(4)]
        print("%5.0f%% %8.1f %9.1f %8.1f %8.1f%% %6d/%d  conf %s" % (
            100 * k, med[0], med[1], med[2], med[3], vistas, len(confs),
            ("%.2f" % (sum(c for c in confs if c > 0) / vistas)) if vistas else "  -- "))

    print("\nLa silla ESTA en todos los fotogramas. Un 0/N significa 'no puedo verla',")
    print("no 'no hay nada'. El nivel donde cae es el umbral PROVISIONAL para la sesion nocturna.")


if __name__ == "__main__":
    main()
