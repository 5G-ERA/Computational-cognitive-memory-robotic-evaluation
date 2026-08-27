#!/usr/bin/env python3
"""Prueba del capturador con un CDP de DOBLE: sin robot y sin gemelo.

Simula lo que de verdad pasa en el laboratorio y que hay que ver funcionar antes
de la sesion: el robot avanzando, el canal de video congelandose a ratos
(fotogramas repetidos) y un tramo con la RELOCALIZACION PERDIDA.
"""
import base64
import io
import json
import math
import os
import subprocess
import sys
import time

RAIZ = "/home/ros/Documents/G1_UNITREE_ROBOT_META_REASONING"
sys.path.insert(0, os.path.join(RAIZ, "src"))
sys.path.insert(0, os.path.join(RAIZ, "tools"))

from PIL import Image


def jpeg(color, w=640, h=360):
    im = Image.new("RGB", (w, h), color)
    b = io.BytesIO()
    im.save(b, "JPEG", quality=88)
    return "data:image/jpeg;base64," + base64.b64encode(b.getvalue()).decode()


class CDPDoble:
    """Robot que avanza en linea recta; el video se congela entre t=4 y t=6 s;
    la relocalizacion se pierde entre t=7 y t=9 s."""

    def __init__(self):
        self.t0 = time.time()

    def eval(self, e):
        t = time.time() - self.t0
        if "videoWidth" in e and "__capc" in e:                 # camara nativa
            congelado = 4.0 <= t < 6.0
            tono = 40 if congelado else int(40 + (t * 37) % 180)
            return json.dumps({"w": 640, "h": 360, "d": jpeg((tono, 90, 120))})
        if "window.__pose" in e:                                # pose
            x, y = 1.0 + 0.25 * t, 0.5 + 0.05 * t
            yaw = math.radians(20.0)
            q = [x, y, 0.0, 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]
            if 7.0 <= t < 9.0:                                  # relocalizacion perdida
                return json.dumps({"pose": None, "reloc": None, "map": q,
                                   "pcd": "", "pt": 0, "rt": 0})
            return json.dumps({"pose": q, "reloc": None, "map": None,
                               "pcd": "m", "pt": t, "rt": t})
        if "__relocbuf" in e and "length" in e:
            return 120
        if "__relocbuf" in e:                                   # nube: PLANA [x,y,z,...]
            pts = []
            for i in range(60):                                 # cambia con t: barrido nuevo
                pts += [round(1.0 + 0.1 * i + 0.01 * t, 3), round(0.4 * i, 3), 0.2]
            return json.dumps(pts)
        return ""


import g1_goto as G                                            # noqa: E402
G.get_live_cdp = lambda *a, **k: CDPDoble()

salida = "/tmp/prueba_captura"
subprocess.run(["rm", "-rf", salida])
sys.argv = ["captura_gemelo.py", "--hz", "2", "--pose-hz", "5",
            "--nube-cada", "3", "--minutos", str(11.0 / 60.0),
            "--salida", salida, "--nota", "prueba con doble"]

import captura_gemelo                                          # noqa: E402
rc = captura_gemelo.main()

print("\n--- COMPROBACIONES ---")
meta = json.load(open(os.path.join(salida, "meta.json")))
frames = [json.loads(l) for l in open(os.path.join(salida, "frames.jsonl"))]
poses = [json.loads(l) for l in open(os.path.join(salida, "poses.jsonl"))]
nubes = [json.loads(l) for l in open(os.path.join(salida, "nube.jsonl"))]
jpgs = os.listdir(os.path.join(salida, "frames"))

fuentes = {}
for p in poses:
    fuentes[p["src"]] = fuentes.get(p["src"], 0) + 1

ok = True
def chk(cond, txt):
    global ok
    print(("  OK   " if cond else "  FALLO ") + txt)
    ok = ok and cond

chk(rc == 0, "sale con codigo 0")
chk(len(jpgs) == meta["fotos"], "ficheros jpg (%d) == fotos declaradas (%d)" % (len(jpgs), meta["fotos"]))
chk(meta["fotos_repetidas"] >= 1, "detecto fotogramas repetidos (%d)" % meta["fotos_repetidas"])
chk(len(frames) == meta["fotos"] + meta["fotos_repetidas"],
    "frames.jsonl tiene una linea por captura, repetidas incluidas (%d)" % len(frames))
chk(all(f["fichero"] for f in frames), "toda linea apunta a un fichero (las repetidas, al original)")
chk(meta["resolucion"] == [640, 360], "resolucion nativa registrada: %s" % meta["resolucion"])
chk("map_odom" in fuentes and "slam_info" in fuentes,
    "registro ambas fuentes de pose: %s" % fuentes)
chk(meta["recorrido_m"] > 1.0, "recorrido acumulado %.2f m" % meta["recorrido_m"])
chk(len(nubes) >= 2, "guardo nubes del laser (%d)" % len(nubes))
chk(bool(nubes) and len(nubes[0]["pts"][0]) == 3, "la nube lleva z (punto crudo 3D)")
chk(meta.get("nubes_con_error", 0) == 0, "sin errores de nube (%d)" % meta.get("nubes_con_error", 0))
chk(meta["solo_lectura"] is True, "meta declara solo_lectura")
prim = json.loads(open(os.path.join(salida, "frames.jsonl")).readline())
chk(prim.get("x") is not None, "la foto lleva su pose (x=%s, y=%s)" % (prim.get("x"), prim.get("y")))
tam = sum(os.path.getsize(os.path.join(salida, "frames", f)) for f in jpgs) / max(1, len(jpgs))
print("  info  jpg medio %.0f KB -> ~%.0f MB/hora a 1 Hz" % (tam / 1024, tam * 3600 / 1e6))
print("\n%s" % ("TODO OK" if ok else "HAY FALLOS"))
sys.exit(0 if ok else 1)
