#!/usr/bin/env python3
"""Captura UNA muestra de lo que ve el robot, etiquetada con el estado de luz que declaras tu.

El robot NO se mueve: sólo se le pide el fotograma actual, se mide, y se le pregunta al servidor
de percepcion que ve en el. Se lanza tantas veces como se quiera, cambiando la luz entre medias.

    python3 tools/vision_sample.py "luces encendidas"
    python3 tools/vision_sample.py "luces apagadas, persiana subida"
    python3 tools/vision_sample.py "solo lampara de mesa" --n 5

POR QUE HACE FALTA. Hoy el robot no distingue "he mirado y no hay objeto" de "no he podido ver
por falta de luz", y esa confusion es justo el par W2 del protocolo. Lo obvio seria mirar el
BRILLO de la imagen, pero no sirve: la camara se autoajusta, y sobre 1.978 fotogramas de una
jornada entera el brillo medio sale clavado cerca de 105 a cualquier hora -- mide cuanto esta
compensando la camara, no cuanta luz hay. Asi que hay que descubrir QUE estadistico si sigue a la
luz, y para eso hacen falta muestras con el estado de luz declarado POR UNA PERSONA.

LA ETIQUETA ES LA REFERENCIA. Si el estado de luz se dedujera de la propia imagen, estariamos
comprobando la camara consigo misma. Por eso el texto es argumento obligatorio.

Guarda en calib_luz/<fecha>/: el JPEG de cada muestra y una linea por muestra en muestras.jsonl
con los estadisticos y lo que respondio la percepcion.
"""
import base64
import io
import json
import os
import sys
import time
import urllib.request

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)

PERC = os.environ.get("G1_PERC", "127.0.0.1:8008")

# Captura ADICIONAL de alta calidad: resolucion NATIVA del <video> y JPEG 0.85, frente a los
# 320px/0.5 de CAM_JS (medido 20-ago: los frames de navegacion salen a ~4KB con nitidez 52
# frente a 476 en reposo -- el canal WebRTC colapsa el bitrate durante la navegacion). Esta
# captura NO toca la ruta de navegacion: solo la usa este muestreador, y se guarda JUNTO a la
# estandar para no romper la comparabilidad con las muestras del 20-ago. La sonda devuelve
# ademas videoWidth/Height: el estado real del stream (ABR) en el momento de la muestra.
CAM_HQ_JS = (
    "(function(){var v=document.querySelector('video');"
    "if(!v||!v.videoWidth)return '';"
    "var c=window.__camhq||(window.__camhq=document.createElement('canvas'));"
    "c.width=v.videoWidth;c.height=v.videoHeight;"
    "c.getContext('2d').drawImage(v,0,0);"
    "try{return JSON.stringify({w:v.videoWidth,h:v.videoHeight,"
    "d:c.toDataURL('image/jpeg',0.85)});}catch(e){return '';}})()"
)


def stats(jpg_bytes):
    """Estadisticos de la imagen. El brillo medio va incluido a proposito aunque se sepa que no
    sirve: es el control negativo, y conviene poder ensenar que no se movio."""
    from PIL import Image, ImageStat, ImageFilter
    im = Image.open(io.BytesIO(jpg_bytes)).convert("L")
    st = ImageStat.Stat(im)
    grano = ImageStat.Stat(im.filter(ImageFilter.FIND_EDGES)).mean[0]
    h = im.histogram()
    n = sum(h) or 1
    return {
        "ancho": im.width, "alto": im.height,
        "brillo_medio": round(st.mean[0], 2),          # control negativo: lo aplana el autoajuste
        "contraste": round(st.stddev[0], 2),           # candidato
        "grano": round(grano, 2),                      # candidato
        "oscuros": round(sum(h[:32]) / n, 4),          # fraccion de pixeles casi negros
        "quemados": round(sum(h[224:]) / n, 4),        # fraccion saturada
    }


def perceive(jpg_b64):
    """Le pregunta al servidor que ve. Devuelve detecciones y numero de celdas del scan virtual."""
    d = json.dumps({"image": "data:image/jpeg;base64," + jpg_b64}).encode()
    req = urllib.request.Request("http://%s/perceive" % PERC, d,
                                 {"Content-Type": "application/json"})
    o = json.loads(urllib.request.urlopen(req, timeout=90).read())
    dets = o.get("detections") or []
    return {
        "n_detecciones": len(dets),
        "detecciones": [[x.get("label"), round(float(x.get("conf", 0)), 2),
                         x.get("range_m")] for x in dets][:6],
        "n_celdas_scan": len(o.get("scan") or []),
        "conf_max": round(max([float(x.get("conf", 0)) for x in dets], default=0.0), 2),
    }


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if not args:
        sys.exit(__doc__)
    etiqueta = " ".join(args)
    print("[aviso] herramienta de solo lectura, pero NO la lances con el robot navegando:")
    print("        dos clientes sobre la misma pagina pueden estorbarse. Robot quieto.")
    n = 1
    if "--n" in sys.argv:
        try:
            n = max(1, int(sys.argv[sys.argv.index("--n") + 1]))
        except (IndexError, ValueError):
            pass

    import g1_nav_v2 as g
    # SOLO LECTURA. NO se usa get_cdp(): esa reinstala el driver de captura en la pagina
    # (INSTALL_JS) y eso PISA la conexion de un run en marcha -- lo tiro una vez, con el robot
    # navegando, con un WebSocketConnectionClosedException. Aqui solo se abre la conexion y se
    # pide el fotograma; no se inyecta nada ni se toca window.__cmd.
    print("conectando a la app del robot (solo lectura)...")
    cdp = g.CDP(g.discover_ws())
    try:
        cdp.call("Runtime.enable")
    except Exception:
        pass

    dia = time.strftime("%Y-%m-%d")
    dst = os.path.join(RAIZ, "calib_luz", dia)
    os.makedirs(dst, exist_ok=True)
    reg = os.path.join(dst, "muestras.jsonl")

    hechas = 0
    for i in range(n):
        j = None
        for intento in range(12):                      # el frame puede tardar en estar listo
            j = cdp.eval(g.CAM_JS)
            if j and isinstance(j, str) and j.startswith("data:image"):
                break
            time.sleep(0.4)
        if not j:
            print("  [%d/%d] sin fotograma: la app no esta en la pantalla de camara?" % (i + 1, n))
            continue
        b64 = j.split(",", 1)[1]
        raw = base64.b64decode(b64)
        ts = time.strftime("%H%M%S")
        nom = "%s_%03d.jpg" % (ts, i + 1)
        open(os.path.join(dst, nom), "wb").write(raw)

        fila = {"hora": time.strftime("%Y-%m-%d %H:%M:%S"), "etiqueta": etiqueta,
                "fichero": nom, "imagen": stats(raw)}
        try:
            hq = cdp.eval(CAM_HQ_JS)
            if hq:
                o = json.loads(hq)
                braw = base64.b64decode(o["d"].split(",", 1)[1])
                nomhq = "%s_%03d_hq.jpg" % (ts, i + 1)
                open(os.path.join(dst, nomhq), "wb").write(braw)
                fila["hq"] = {"fichero": nomhq, "video_wh": [o["w"], o["h"]],
                              "imagen": stats(braw)}
        except Exception as e:
            fila["hq"] = {"error": str(e)[:60]}
        try:
            fila["percepcion"] = perceive(b64)
        except Exception as e:
            fila["percepcion"] = {"error": "%s: %s" % (type(e).__name__, e)}

        with open(reg, "a") as w:
            w.write(json.dumps(fila, ensure_ascii=False) + "\n")

        im, pc = fila["imagen"], fila["percepcion"]
        print("  [%d/%d] %s  brillo %.0f  contraste %.1f  grano %.1f  oscuros %.0f%%  ->  %s" % (
            i + 1, n, nom, im["brillo_medio"], im["contraste"], im["grano"],
            100 * im["oscuros"],
            ("%d detecciones (conf max %.2f), %d celdas" % (pc["n_detecciones"], pc["conf_max"],
                                                            pc["n_celdas_scan"]))
            if "error" not in pc else "PERCEPCION: " + pc["error"]))
        hechas += 1
        if n > 1 and i < n - 1:
            time.sleep(1.5)

    print("\n%d muestra(s) de «%s» guardadas en calib_luz/%s/" % (hechas, etiqueta, dia))
    print("Cambia la luz y vuelve a lanzarlo con la etiqueta nueva.")


if __name__ == "__main__":
    main()
