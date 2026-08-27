#!/usr/bin/env python3
"""Captura para RECONSTRUIR EL GEMELO: fotos + odometria del mapa, sin conducir.

Pensado para la sesion en la que alguien TELEOPERA el robot por todo el sitio:
este programa solo MIRA. No envia un solo comando al robot -- todo lo que hace
son lecturas del WebView (`cdp.eval` de consulta), asi que no compite con quien
conduce ni cambia el comportamiento que se esta grabando.

Que guarda, y por que cada cosa:
  frames/f######.jpg  la foto, a la RESOLUCION NATIVA del video (no la de 320 px
                      con calidad 0.5 que usa la navegacion): para proyectar
                      texturas sobre la geometria hace falta todo el pixel posible.
  frames.jsonl        una linea por foto: instante, fichero, POSE EN EL INSTANTE
                      DE LA FOTO, tamano, y si el canal repitio fotograma.
  poses.jsonl         la odometria del mapa a mas frecuencia que las fotos (5 Hz
                      por defecto): la trayectoria fina, que es lo que alinea
                      las fotos entre si.
  nube.jsonl          celdas del laser + pose, cada pocos segundos: la geometria
                      contra la que se texturiza.
  meta.json           parametros, version del codigo, resolucion y el recuento
                      final. Sin esto la captura no es reconstruible.

FOTOGRAMAS REPETIDOS. El canal de video de la app se congela a ratos. Un frame
identico al anterior NO se guarda dos veces (se anota `dup` y se apunta al
fichero original): duplicar disco es lo de menos, el problema es creerse que hay
mas puntos de vista de los que hay. El resumen final dice cuantos hubo.

Uso tipico (en la maquina que conduce, con el proxy y la app vivos):
    python3 tools/captura_gemelo.py                  # 1 foto/s hasta Ctrl+C
    python3 tools/captura_gemelo.py --hz 2 --minutos 40
    python3 tools/captura_gemelo.py --sin-nube       # solo fotos y odometria

Se para con Ctrl+C y escribe el resumen. Si la conexion se cae, reintenta sola.
"""
import argparse
import base64
import hashlib
import json
import math
import os
import signal
import sys
import time

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(RAIZ, "src"))

import g1_goto as G                      # noqa: E402  (reusa conexion, pose, nube)
import g1_nav_v2 as g                    # noqa: E402

# Captura a resolucion NATIVA y calidad alta. Si la pagina no la sirve (p.ej. el
# gemelo, que responde a la consulta estandar de navegacion), se cae a `g.CAM_JS`.
CAM_NATIVA_JS = (
    "(function(){var v=document.querySelector('video');"
    "if(!v||!v.videoWidth)return '';"
    "var W=v.videoWidth,H=v.videoHeight;"
    "var c=window.__capc||(window.__capc=document.createElement('canvas'));"
    "c.width=W;c.height=H;c.getContext('2d').drawImage(v,0,0,W,H);"
    "try{return JSON.stringify({w:W,h:H,d:c.toDataURL('image/jpeg',0.92)});}"
    "catch(e){return '';}})()"
)

_PARAR = {"si": False}


def _alto(sig, frame):
    _PARAR["si"] = True


def foto(cdp):
    """(data_uri, w, h, via) a la mejor resolucion que sirva la pagina."""
    try:
        s = cdp.eval(CAM_NATIVA_JS)
        if s and isinstance(s, str) and s.startswith("{"):
            d = json.loads(s)
            if d.get("d", "").startswith("data:image"):
                return d["d"], d.get("w"), d.get("h"), "nativa"
    except Exception:
        pass
    try:                                   # respaldo: el canal de navegacion
        j = cdp.eval(g.CAM_JS)
        if j and isinstance(j, str) and j.startswith("data:image"):
            return j, None, None, "navegacion"
    except Exception:
        pass
    return None, None, None, None


def pose_de(cdp):
    """(x, y, yaw_deg, fuente) o None."""
    try:
        src, p, _ = G.read_pose(cdp)
    except Exception:
        return None
    if not p:
        return None
    try:
        return (round(float(p[0]), 4), round(float(p[1]), 4),
                round(G.yaw_of(p), 2), src)
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hz", type=float, default=1.0, help="fotos por segundo (1)")
    ap.add_argument("--pose-hz", type=float, default=5.0, help="muestras de odometria/s (5)")
    ap.add_argument("--nube-cada", type=float, default=5.0, help="segundos entre nubes (5)")
    ap.add_argument("--sin-nube", action="store_true")
    ap.add_argument("--minutos", type=float, default=0.0, help="tope; 0 = hasta Ctrl+C")
    ap.add_argument("--salida", default=None)
    ap.add_argument("--nota", default="", help="se guarda en meta.json (p.ej. 'vuelta sala B')")
    a = ap.parse_args()

    signal.signal(signal.SIGINT, _alto)
    signal.signal(signal.SIGTERM, _alto)

    cdp = G.get_live_cdp()
    if not cdp:
        print("No hay pagina viva del WebView. ¿proxy en 9221 + app abierta + robot relocalizado?")
        return 1
    try:
        cdp.eval(g.LOWSTATE_JS)
    except Exception:
        pass

    print("Esperando pose...", end="", flush=True)
    for _ in range(30):
        if pose_de(cdp):
            break
        time.sleep(0.3)
    else:
        print(" sin pose. ¿Mapa cargado y robot RELOCALIZADO?")
        return 1
    print(" ok.")

    sesion = time.strftime("%Y%m%d_%H%M%S")
    base = a.salida or os.path.join(RAIZ, "dataset", "reconstruccion", sesion)
    os.makedirs(os.path.join(base, "frames"), exist_ok=True)
    f_frames = open(os.path.join(base, "frames.jsonl"), "w")
    f_poses = open(os.path.join(base, "poses.jsonl"), "w")
    f_nube = None if a.sin_nube else open(os.path.join(base, "nube.jsonl"), "w")

    try:
        sha = os.popen("git -C %s rev-parse --short HEAD" % RAIZ).read().strip()
    except Exception:
        sha = None

    t0 = time.time()
    per_foto = 1.0 / max(0.05, a.hz)
    per_pose = 1.0 / max(0.2, a.pose_hz)
    tope = a.minutos * 60.0 if a.minutos > 0 else None

    n_foto = n_dup = n_pose = n_nube = n_fallo = 0
    n_sin_pose = n_nube_vacia = n_nube_error = 0
    fuente_ant = None          # slam_info = relocalizado; otra cosa = a la deriva
    t_sin_pose = None
    ult_hash = None
    ult_fichero = None
    ult_pose = None
    dist = 0.0
    xs, ys = [], []
    t_foto = t_pose = t_nube = 0.0
    res_w = res_h = None
    via_usada = None

    print("GRABANDO en %s" % os.path.relpath(base, RAIZ))
    print("Solo lectura: este programa NO conduce. Ctrl+C para cerrar.\n")

    while not _PARAR["si"]:
        ahora = time.time() - t0
        if tope and ahora >= tope:
            break

        # --- odometria (mas rapida que las fotos: es la que alinea todo) ---
        if ahora - t_pose >= per_pose:
            t_pose = ahora
            p = pose_de(cdp)
            if p is None:
                n_sin_pose += 1
                if t_sin_pose is None:
                    t_sin_pose = ahora
                    print("  [%4.0fs] AVISO: sin pose. ¿La app sigue abierta y con el mapa?"
                          % ahora, flush=True)
            else:
                if t_sin_pose is not None:
                    print("  [%4.0fs] pose recuperada (%.0f s sin ella)"
                          % (ahora, ahora - t_sin_pose), flush=True)
                    t_sin_pose = None
                x, y, yaw, src = p
                # Conduciendo con el mando es facil salirse del mapa: cuando la
                # fuente deja de ser la relocalizada, la pose sigue saliendo pero
                # YA NO ESTA ANCLADA al mapa -- y eso invalida la reconstruccion
                # de ese tramo. Se avisa en el momento, que es cuando se arregla.
                if src != fuente_ant:
                    if fuente_ant is not None:
                        if src == "slam_info":
                            print("  [%4.0fs] OK: pose anclada al mapa otra vez (%s)"
                                  % (ahora, src), flush=True)
                        else:
                            print("  [%4.0fs] *** RELOCALIZACION PERDIDA: pose pasa a '%s'. "
                                  "Para al estudiante y re-relocaliza en la app; este tramo "
                                  "no vale para reconstruir. ***" % (ahora, src), flush=True)
                    fuente_ant = src
                f_poses.write(json.dumps({"t": round(ahora, 3), "x": x, "y": y,
                                          "yaw": yaw, "src": src}) + "\n")
                n_pose += 1
                if ult_pose:
                    d = math.hypot(x - ult_pose[0], y - ult_pose[1])
                    if d < 1.0:            # salto de relocalizacion: no es camino
                        dist += d
                ult_pose = (x, y)
                xs.append(x); ys.append(y)

        # --- foto ---
        if ahora - t_foto >= per_foto:
            t_foto = ahora
            uri, w, h, via = foto(cdp)
            if not uri:
                n_fallo += 1
            else:
                via_usada = via_usada or via
                crudo = base64.b64decode(uri.split(",", 1)[1])
                hsh = hashlib.sha1(crudo).hexdigest()
                p = ult_pose
                pos = pose_de(cdp) or (None, None, None, None)
                if hsh == ult_hash:        # el canal repitio fotograma
                    n_dup += 1
                    fila = {"t": round(ahora, 3), "fichero": ult_fichero, "dup": True,
                            "x": pos[0], "y": pos[1], "yaw": pos[2]}
                else:
                    n_foto += 1
                    nombre = "f%06d.jpg" % n_foto
                    with open(os.path.join(base, "frames", nombre), "wb") as fh:
                        fh.write(crudo)
                    if res_w is None:
                        res_w, res_h = w, h
                    fila = {"t": round(ahora, 3), "fichero": nombre, "dup": False,
                            "x": pos[0], "y": pos[1], "yaw": pos[2],
                            "w": w, "h": h, "bytes": len(crudo)}
                    ult_hash, ult_fichero = hsh, nombre
                f_frames.write(json.dumps(fila) + "\n")
                if n_foto % 20 == 0 and not fila["dup"]:
                    f_frames.flush(); f_poses.flush()
                    print("  %5.0fs  fotos %4d (rep %3d)  odom %5d  recorrido %5.1f m"
                          % (ahora, n_foto, n_dup, n_pose, dist), flush=True)

        # --- nube del laser: CRUDA, con z ---
        # No se usa reloc_cells: eso devuelve celdas 2D discretizadas a 0.2 m y
        # filtradas a la banda del torso, que es lo que necesita NAVEGAR. Para
        # reconstruir hace falta el punto crudo con su altura. El buffer viene
        # PLANO ([x,y,z,x,y,z,...]) y ya en el frame del mapa.
        if f_nube is not None and ahora - t_nube >= a.nube_cada:
            t_nube = ahora
            try:
                cru = cdp.eval("JSON.stringify(window.__relocbuf||[])")
                buf = json.loads(cru) if cru else []
                pts = []
                for i in range(0, len(buf) - 2, 3):
                    pts.append([round(float(buf[i]), 3), round(float(buf[i + 1]), 3),
                                round(float(buf[i + 2]), 3)])
                if pts and ult_pose:
                    paso = max(1, len(pts) // 3000)      # tope por muestra
                    f_nube.write(json.dumps({
                        "t": round(ahora, 3), "x": ult_pose[0], "y": ult_pose[1],
                        "n": len(pts), "paso": paso,
                        "pts": pts[::paso]}) + "\n")
                    n_nube += 1
                else:
                    n_nube_vacia += 1
            except Exception as e:
                # Fallar callado aqui seria perder la geometria entera sin
                # enterarse hasta el analisis. Se cuenta y se avisa una vez.
                n_nube_error += 1
                if n_nube_error == 1:
                    print("  [%4.0fs] AVISO: no puedo leer la nube del laser (%s). "
                          "Las fotos y la odometria siguen grabandose."
                          % (ahora, type(e).__name__), flush=True)

        time.sleep(0.05)

    dur = time.time() - t0
    meta = {
        "sesion": sesion, "nota": a.nota, "git": sha,
        "solo_lectura": True,
        "duracion_s": round(dur, 1),
        "fotos": n_foto, "fotos_repetidas": n_dup, "fotos_fallidas": n_fallo,
        "muestras_odometria": n_pose, "nubes": n_nube,
        "nubes_vacias": n_nube_vacia, "nubes_con_error": n_nube_error,
        "hz_fotos": a.hz, "hz_odometria": a.pose_hz,
        "resolucion": [res_w, res_h], "canal_camara": via_usada,
        "recorrido_m": round(dist, 2),
        "muestras_sin_pose": n_sin_pose,
        "ultima_fuente_pose": fuente_ant,
        "cobertura_bbox": ([round(min(xs), 2), round(min(ys), 2),
                            round(max(xs), 2), round(max(ys), 2)] if xs else None),
        "aviso": "las poses son del SLAM de la app; un salto de relocalizacion "
                 ">1 m no se suma al recorrido pero SI queda en poses.jsonl",
    }
    json.dump(meta, open(os.path.join(base, "meta.json"), "w"), indent=1)
    for fh in (f_frames, f_poses, f_nube):
        if fh:
            fh.close()

    print("\n=== CAPTURA CERRADA ===")
    print("  carpeta        %s" % os.path.relpath(base, RAIZ))
    print("  duracion       %.1f min" % (dur / 60.0))
    print("  fotos          %d  (repetidas por el canal: %d, fallidas: %d)"
          % (n_foto, n_dup, n_fallo))
    print("  resolucion     %sx%s  (canal: %s)" % (res_w, res_h, via_usada))
    print("  odometria      %d muestras, recorrido %.1f m  (sin pose: %d)"
          % (n_pose, dist, n_sin_pose))
    if not a.sin_nube:
        print("  nubes laser    %d  (vacias: %d, con error: %d)"
              % (n_nube, n_nube_vacia, n_nube_error))
        if n_nube == 0:
            print("  AVISO: NINGUNA nube guardada -- la reconstruccion se queda sin "
                  "geometria; revisa que la app publique la nube 'location'.")
    if meta["cobertura_bbox"]:
        b = meta["cobertura_bbox"]
        print("  cobertura      x[%.1f, %.1f]  y[%.1f, %.1f]" % (b[0], b[2], b[1], b[3]))
    if n_foto and n_dup > n_foto:
        print("  AVISO: mas fotogramas repetidos que nuevos -- el canal de video iba "
              "atascado; la cobertura visual real es menor de lo que parece.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
