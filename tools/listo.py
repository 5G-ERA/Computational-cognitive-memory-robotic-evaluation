#!/usr/bin/env python3
"""¿Esta la cadena lista para grabar/conducir? Comprueba los eslabones y dice
EXACTAMENTE cual falta. Solo lectura: no mueve el robot.

    python3 tools/listo.py
"""
import json
import os
import sys
import urllib.request

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(RAIZ, "src"))
sys.path.insert(0, os.path.join(RAIZ, "tools"))

OK, MAL = "  OK   ", "  FALTA "


def get(url, t=6):
    with urllib.request.urlopen(url, timeout=t) as r:
        return json.loads(r.read().decode())


def main():
    fallos = []

    # 1) proxy y dispositivo
    try:
        d = get("http://localhost:9221/json")
        if d:
            print(OK + "proxy 9221 ve el dispositivo: %s (iOS %s)"
                  % (d[0].get("deviceName"), d[0].get("deviceOSVersion")))
            puerto = (d[0].get("url") or "localhost:9222").split(":")[-1]
        else:
            print(MAL + "proxy 9221 no ve ningun dispositivo -> cable USB / desbloquea / 'Trust'")
            fallos.append("dispositivo")
            puerto = "9222"
    except Exception as e:
        print(MAL + "proxy 9221 no responde (%s) -> arranca ios_webkit_debug_proxy" % type(e).__name__)
        return 1

    # 2) pagina del WebView
    try:
        ps = get("http://localhost:%s/json" % puerto)
        if ps:
            print(OK + "WebView: %s" % (ps[0].get("title") or "?"))
        else:
            print(MAL + "el dispositivo se ve pero NO lista paginas WebView -> "
                        "Ajustes>Safari>Avanzado>Inspector web, o proxy desactualizado "
                        "(docs/SETUP_UBUNTU.md §2)")
            fallos.append("webview")
    except Exception as e:
        print(MAL + "no puedo listar paginas (%s)" % type(e).__name__)
        fallos.append("webview")

    # 3) percepcion
    try:
        h = get("http://127.0.0.1:8008/health")
        print(OK + "percepcion :8008 (%s)" % h.get("mode"))
    except Exception:
        print(MAL + "percepcion :8008 no responde -> bash tools/arranca_percepcion.sh")
        fallos.append("percepcion")

    # 4) datos vivos del robot
    if "webview" not in fallos:
        import g1_goto as G
        import captura_gemelo as C
        cdp = G.get_live_cdp()
        if not cdp:
            print(MAL + "no engancho con la pagina del robot")
            return 1
        src, p, _ = G.read_pose(cdp)
        if p:
            print(OK + "POSE (%s): x=%.2f y=%.2f yaw=%.0f" % (src, p[0], p[1], G.yaw_of(p)))
            if src != "slam_info":
                print("         ojo: la fuente no es la relocalizada; relocaliza en la app")
        else:
            print(MAL + "SIN POSE -> carga el mapa y RELOCALIZA el robot en la app")
            fallos.append("pose")
        try:
            modo = cdp.eval("String(location.hash||'')")
        except Exception:
            modo = ""
        if modo:
            print("         modo de la app: %s" % modo)
        n = nb = 0
        for var in ("__relocbuf", "__buf"):
            try:
                v = int(cdp.eval("(window.%s||[]).length" % var) or 0)
            except Exception:
                v = 0
            if var == "__relocbuf":
                n = v
            else:
                nb = v
        if max(n, nb) > 30:
            print(OK + "nube del laser: %d puntos (%s)"
                  % (max(n, nb) // 3, "relocbuf" if n >= nb else "buf/mapeo"))
        else:
            print(MAL + "nube del laser VACIA -> el SLAM no esta publicando. "
                        "Operando: relocaliza. Mapeando: dale a empezar el mapa")
            fallos.append("nube")
        uri, w, h_, via = C.foto(cdp)
        if uri:
            print(OK + "video: %sx%s (%s), %d KB por foto" % (w, h_, via, len(uri) / 1024))
        else:
            print(MAL + "sin frame de video -> la app debe estar en la pantalla de operacion")
            fallos.append("video")

    print()
    if fallos:
        print(">>> NO listo. Falta: %s" % ", ".join(fallos))
        return 1
    print(">>> LISTO para grabar y conducir.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
