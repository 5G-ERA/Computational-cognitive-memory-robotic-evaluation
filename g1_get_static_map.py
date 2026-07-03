#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
g1_get_static_map.py — Saca ENTERO el mapa ESTATICO que usa el robot para navegar.

Tres fuentes (tres modos), de mas "verdad-robot" a mas "verdad-navegador":

  1) pcd      El .pcd guardado DENTRO del G1 (/unitree/data/unitree_slam/<nombre>.pcd),
              el que usa el firmware para relocalizarse. Descarga por WebRTC puro con
              slam_operate api_id=1934 (getBigFile), por chunks.
              REQUISITOS: app del iPhone CERRADA (sesion WebRTC unica), Mac en el AP del
              robot (192.168.12.1), venv con unitree_webrtc_connect.
              NOTA: el formato exacto de los chunks de 1934 no esta confirmado en este
              firmware -> el modo es un PROBE robusto: imprime la respuesta cruda si no
              la entiende, y reensambla si reconoce el patron (offset/base64).

  2) webview  La nube del mapa CARGADO que la app decodifica y pinta en su WebView
              (mismo truco que cmd_mapgrab de g1_goto.py, pero standalone). Captura la
              nube MAS GRANDE que pase por los workers (= el mapa entero, decenas de
              miles de puntos, no el laser vivo de ~1-3k).
              REQUISITOS: iPhone por USB + ios_webkit_debug_proxy corriendo + app en la
              pantalla del mapa con el MAPA CARGADO. Mueve/rota la vista del mapa (o
              re-localiza) durante la captura para forzar el redibujado.

  3) local    El mapa estatico que usa TU navegacion como plan global (G1_GLOBALMAP=hard),
              tal como esta en disco: refmap (G1_REFMAP: 'summit' = summit/ref_map_g1.json
              alineado, 'g1' = dataset/map_full.json) + nav_map.json (celdas OCELL
              acumuladas). Sin robot. (La capa 'hard' en runtime suma ademas celdas
              saturadas por score y colisiones de la sesion: eso es estado vivo, no disco.)

Todos los modos exportan a maps_out/:  <base>.json  +  <base>.pcd  +  <base>.png

USO (desde la carpeta G1 ROBOT):
  python g1_get_static_map.py local                      # sin robot, ahora mismo
  python g1_get_static_map.py webview [--secs 40]        # iPhone USB + mapa cargado en la app
  python g1_get_static_map.py pcd [--name Qw_20260625]   # app CERRADA, WebRTC directo

Opciones comunes: --out DIR (defecto maps_out) · --no-png
  webview: --secs N (defecto 40) · --frame auto|yup|zup (defecto auto)
  pcd:     --name NOMBRE (defecto: el 'pcd' de waypoints.json) · --address RUTA_COMPLETA
           --chunk BYTES (defecto 65536) · --ip IP (defecto 192.168.12.1)
"""
import argparse
import base64
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
WP_FILE = os.path.join(HERE, "waypoints.json")


# ----------------------------------------------------------------------------- utilidades
def load_waypoints():
    try:
        return json.load(open(WP_FILE))
    except Exception:
        return {}


def default_pcd_name():
    wps = load_waypoints()
    for w in wps.values():
        if isinstance(w, dict) and w.get("pcd"):
            return w["pcd"]
    return "Qw_20260625"


def write_pcd(path, pts):
    """PCD ASCII x y z (pts = lista de [x,y,z] o [x,y] -> z=0)."""
    rows = [(p[0], p[1], (p[2] if len(p) > 2 else 0.0)) for p in pts]
    with open(path, "w") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        f.write(f"WIDTH {len(rows)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(rows)}\nDATA ascii\n")
        for x, y, z in rows:
            f.write(f"{x:.4f} {y:.4f} {z:.4f}\n")


def write_png(path, layers, title):
    """layers = [(pts2d, kwargs_scatter), ...]. Dibuja waypoints A/B/C encima si existen."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("  (sin matplotlib -> me salto el PNG; pip install matplotlib)")
        return
    fig, ax = plt.subplots(figsize=(10, 10))
    for pts, kw in layers:
        if pts:
            ax.scatter([p[0] for p in pts], [p[1] for p in pts], **kw)
    for name, w in load_waypoints().items():
        try:
            ax.plot(w["x"], w["y"], "r*", ms=18, zorder=10)
            ax.annotate(name, (w["x"], w["y"]), fontsize=14, color="red",
                        xytext=(6, 6), textcoords="offset points", zorder=10)
        except Exception:
            pass
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(title)
    if any(pts for pts, _ in layers):
        ax.legend(loc="best", fontsize=9)
    fig.savefig(path, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"  PNG  -> {path}")


def save_all(outdir, base, pts3d, meta, layers_png=None, title="", no_png=False):
    os.makedirs(outdir, exist_ok=True)
    pj = os.path.join(outdir, base + ".json")
    pp = os.path.join(outdir, base + ".pcd")
    meta = dict(meta)
    meta["npts"] = len(pts3d)
    meta["points"] = pts3d
    json.dump(meta, open(pj, "w"))
    print(f"  JSON -> {pj}  ({len(pts3d)} puntos)")
    write_pcd(pp, pts3d)
    print(f"  PCD  -> {pp}")
    if not no_png:
        if layers_png is None:
            layers_png = [([(p[0], p[1]) for p in pts3d],
                           dict(s=2, c="k", alpha=0.6, label="mapa"))]
        write_png(os.path.join(outdir, base + ".png"), layers_png, title or base)


# =============================================================================
# MODO 1: local — el mapa estatico del plan global de g1_goto (disco, sin robot)
# =============================================================================
def mode_local(args):
    print(">>> MODO local: mapa estatico del plan global (G1_GLOBALMAP=hard), desde disco.\n")
    OCELL = 0.2
    choice = os.environ.get("G1_REFMAP", "summit").lower()

    # --- refmap (mismas rutas y prioridad que ref_points() de g1_goto.py) ---
    ref = []
    src_ref = None
    if choice != "g1":
        p = os.path.join(HERE, "summit", "ref_map_g1.json")
        if os.path.exists(p):
            try:
                ref = [(q[0], q[1]) for q in json.load(open(p)).get("points", [])]
                src_ref = "summit/ref_map_g1.json (alineado A/B)"
            except Exception as e:
                print("  aviso: no pude leer", p, repr(e))
    if not ref:
        p = os.path.join(HERE, "dataset", "map_full.json")
        if os.path.exists(p):
            try:
                d = json.load(open(p))
                for q in d.get("points", []):
                    if len(q) >= 3 and -0.5 <= q[2] <= 0.6:
                        ref.append((q[0], q[1]))
                    elif len(q) == 2:
                        ref.append((q[0], q[1]))
                ref = [(a, b) for (a, b) in ref if -15 <= a <= 15 and -15 <= b <= 15]
                src_ref = "dataset/map_full.json (mapa propio G1 — puede estar desalineado)"
            except Exception as e:
                print("  aviso: no pude leer", p, repr(e))
    print(f"  refmap  : {len(ref)} puntos  ({src_ref or 'NO ENCONTRADO'})")

    # --- nav_map.json (celdas OCELL acumuladas por waypoint/sweep) ---
    nav = []
    OC = OCELL
    p = os.path.join(HERE, "nav_map.json")
    if os.path.exists(p):
        try:
            d = json.load(open(p))
            OC = d.get("OCELL", OCELL)
            nav = [(c[0] * OC, c[1] * OC) for c in d.get("cells", [])]
            nav = [(a, b) for (a, b) in nav if -15 <= a <= 15 and -15 <= b <= 15]
        except Exception as e:
            print("  aviso: no pude leer nav_map.json:", repr(e))
    print(f"  nav_map : {len(nav)} celdas (OCELL={OC})")

    if not ref and not nav:
        print("\n  Nada que exportar (ni refmap ni nav_map). ¿Estas en la carpeta G1 ROBOT?")
        return 1

    # union en celdas OCELL = exactamente lo que ve el A* global como estatico
    cells = set((round(x / OC), round(y / OC)) for x, y in ref)
    cells |= set((round(x / OC), round(y / OC)) for x, y in nav)
    pts3d = [[cx * OC, cy * OC, 0.0] for cx, cy in sorted(cells)]

    meta = {"source": "static_global_plan (refmap + nav_map, celdas OCELL)",
            "refmap_src": src_ref, "refmap_pts": len(ref),
            "nav_map_cells": len(nav), "OCELL": OC,
            "frame": "G1/map (Z-up, 2D en z=0)", "G1_REFMAP": choice,
            "note": "la capa hard en runtime suma ademas celdas score-saturadas y colisiones (estado vivo)"}
    layers = [(ref, dict(s=2, c="0.55", alpha=0.7, label=f"refmap ({len(ref)})")),
              (nav, dict(s=6, c="tab:blue", alpha=0.8, label=f"nav_map ({len(nav)})"))]
    save_all(args.out, "map_static_local", pts3d, meta, layers,
             "Mapa ESTATICO del plan global (refmap + nav_map) + waypoints", args.no_png)
    return 0


# =============================================================================
# MODO 2: webview — el mapa cargado que la app decodifica (iPhone USB + CDP)
# =============================================================================
PROXY = "http://localhost:9221"

# El mismo gancho que cmd_mapgrab de g1_goto.py: guarda la nube MAS GRANDE que
# pase por los workers (el mapa cargado es la mayor, decenas de miles de puntos).
MAPGRAB_JS = r"""(function(){
  if(!window.__mapHook){ window.__mapHook=1; window.__mapbuf=[]; window.__mapinfo={n:0,type:'',t:0};
    var seen=new WeakSet();
    var o=Worker.prototype.postMessage;
    Worker.prototype.postMessage=function(m){
      try{ if(!seen.has(this)){ seen.add(this);
        this.addEventListener('message',function(ev){ try{
          var d=ev.data; if(!d||typeof d!=='object') return;
          var ty=(d.type!=null?(''+d.type):''); var dd=d.data; var arr=null;
          if(dd&&typeof dd==='object'){ arr=dd.directOutput||dd.points||dd.cloud||dd.positions||dd.data; }
          if(!arr && (d.points||d.positions)) arr=d.points||d.positions;
          if(arr){ var n=arr.length||(arr.byteLength?arr.byteLength/4:0);
            if(n>window.__mapinfo.n){
              var a=(ArrayBuffer.isView(arr))?Array.prototype.slice.call(arr):Object.values(arr);
              window.__mapbuf=a.slice(0,900000);
              window.__mapinfo={n:n,type:ty,t:Date.now()};
            }
          }
        }catch(e){} });
      } }catch(e){}
      return o.apply(this,arguments);
    };
  } return JSON.stringify(window.__mapinfo);
})()"""


def _discover_ws():
    import requests
    devs = requests.get(PROXY + "/json", timeout=5).json()
    if not devs:
        raise RuntimeError("No hay dispositivos. ¿iPhone por USB y ios_webkit_debug_proxy corriendo?")
    durl = devs[0]["url"]
    pages = requests.get(f"http://{durl}/json", timeout=5).json()
    if not pages:
        raise RuntimeError("No hay paginas. ¿App en la pantalla del mapa? ¿Inspector de Safari CERRADO?")
    page = next((p for p in pages if "slam" in (p.get("url", "").lower())), pages[0])
    print("  Pagina:", page.get("title"), page.get("url"))
    return page["webSocketDebuggerUrl"]


class CDP:
    """Cliente minimo para ios_webkit_debug_proxy (envuelve en Target.sendMessageToTarget).
    Copiado de g1_inspector_bridge.py para no arrancar el driver de g1_nav_v2."""
    def __init__(self, url):
        import websocket
        self.ws = websocket.create_connection(url, max_size=None)
        self.id = 0
        self.target = None
        end = time.time() + 6
        while time.time() < end and not self.target:
            self.ws.settimeout(max(0.1, end - time.time()))
            try:
                msg = json.loads(self.ws.recv())
            except Exception:
                continue
            if msg.get("method") == "Target.targetCreated":
                ti = msg["params"]["targetInfo"]
                if ti.get("type") == "page":
                    self.target = ti["targetId"]
        if not self.target:
            raise RuntimeError("No llego Target.targetCreated (¿pagina del mapa abierta?)")

    def call(self, method, params=None, timeout=15):
        self.id += 1
        iid = self.id
        inner = json.dumps({"id": iid, "method": method, "params": params or {}})
        self.ws.send(json.dumps({"id": iid, "method": "Target.sendMessageToTarget",
                                 "params": {"targetId": self.target, "message": inner}}))
        end = time.time() + timeout
        while time.time() < end:
            self.ws.settimeout(max(0.1, end - time.time()))
            try:
                msg = json.loads(self.ws.recv())
            except Exception:
                continue
            if msg.get("method") == "Target.dispatchMessageFromTarget":
                im = msg["params"]["message"]
                im = json.loads(im) if isinstance(im, str) else im
                if im.get("id") == iid:
                    if "error" in im:
                        raise RuntimeError(im["error"])
                    return im
        raise TimeoutError(method)

    def eval(self, expr):
        r = self.call("Runtime.evaluate", {"expression": expr, "returnByValue": True})
        res = (r.get("result") or {}).get("result") or {}
        return res.get("value")


def _frame_convert(buf, frame):
    """buf plano [a,b,c,...] -> lista [[x,y,z],...] en frame del MAPA (Z-up).
    'zup'  : nube location / mapa (idx0=x, idx1=y, idx2=altura) -> tal cual.
    'yup'  : nube Three.js de mapeo (idx1=altura)               -> x=idx0, y=-idx2, z=idx1.
    'auto' : el eje VERTICAL es el de MENOR rango (altura interior ~2-3 m vs sala 8-15 m)."""
    tr = [buf[i:i + 3] for i in range(0, len(buf) - 2, 3)]
    if not tr:
        return [], frame
    if frame == "auto":
        rng = [max(t[k] for t in tr) - min(t[k] for t in tr) for k in (0, 1, 2)]
        frame = "yup" if rng[1] == min(rng) else "zup"
        print(f"  frame auto: rangos x/y/z = {rng[0]:.1f}/{rng[1]:.1f}/{rng[2]:.1f} m -> '{frame}'")
    if frame == "yup":
        return [[t[0], -t[2], t[1]] for t in tr], frame
    return [[t[0], t[1], t[2]] for t in tr], frame


def mode_webview(args):
    print(">>> MODO webview: capturo el mapa CARGADO desde la WebView de la app (USB).")
    print("    Pre: ios_webkit_debug_proxy corriendo · app en la pantalla del mapa con el mapa CARGADO")
    print("    Durante la captura: MUEVE/ROTA la vista del mapa en la app (fuerza el redibujado).\n")
    cdp = CDP(_discover_ws())
    try:
        cdp.call("Runtime.enable")
    except Exception:
        pass
    cdp.eval(MAPGRAB_JS)
    t0 = time.time()
    best = 0
    try:
        while time.time() - t0 < args.secs:
            info = json.loads(cdp.eval(MAPGRAB_JS) or "{}")
            best = info.get("n", 0)
            print(f"  nube max: {best // 3} puntos (type='{info.get('type','')}')  "
                  f"t={time.time()-t0:.0f}/{args.secs}s  (Ctrl+C fija antes)", end="\r")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print()
    buf = json.loads(cdp.eval("JSON.stringify(window.__mapbuf||[])") or "[]")
    info = json.loads(cdp.eval(MAPGRAB_JS) or "{}")
    print(f"\n  capturados {len(buf)//3} puntos (mensaje type='{info.get('type','')}')")
    if len(buf) < 300:
        print("  MUY POCOS puntos. ¿Mapa cargado y visible? Mueve la vista del mapa y repite.")
        return 1
    pts3d, frame = _frame_convert(buf, args.frame)
    # dedup a voxel de 5 cm (el mapa se redibuja varias veces)
    vox = {}
    for x, y, z in pts3d:
        vox[(round(x / 0.05), round(y / 0.05), round(z / 0.05))] = (x, y, z)
    pts3d = [list(v) for v in vox.values()]
    meta = {"source": "app_loaded_map (WebView worker, nube mas grande)",
            "msg_type": info.get("type", ""), "frame_in": args.frame, "frame_used": frame,
            "frame": "map (Z-up) tras conversion", "voxel_dedup": 0.05}
    pts2d = [(p[0], p[1]) for p in pts3d if -0.5 <= p[2] <= 0.6]   # banda de obstaculos para el PNG
    layers = [([(p[0], p[1]) for p in pts3d], dict(s=1, c="0.75", alpha=0.4, label="todas las alturas")),
              (pts2d, dict(s=2, c="k", alpha=0.8, label="banda obstaculos [-0.5,0.6]m"))]
    save_all(args.out, "map_webview", pts3d, meta, layers,
             "Mapa CARGADO capturado del WebView (frame mapa)", args.no_png)
    return 0


# =============================================================================
# MODO 3: pcd — descarga del .pcd del robot por WebRTC (getBigFile api 1934)
# =============================================================================
def _find_chunk(resp):
    """Busca en la respuesta el payload del chunk + metadatos, sea cual sea el anidamiento.
    Devuelve (bytes|None, meta_dict). Reconoce base64 en campos tipicos y offset/total/seq."""
    meta = {}
    payload = None

    def walk(o, depth=0):
        nonlocal payload
        if depth > 6 or o is None:
            return
        if isinstance(o, dict):
            for k, v in o.items():
                lk = str(k).lower()
                if isinstance(v, (int, float)) and lk in ("offset", "total", "size", "seq",
                                                          "index", "count", "filesize",
                                                          "total_size", "chunks", "len", "length"):
                    meta[lk] = v
                if isinstance(v, str) and lk in ("data", "filedata", "content", "chunk",
                                                 "payload", "file", "b64", "buf") and len(v) > 32:
                    try:
                        raw = base64.b64decode(v, validate=True)
                        if payload is None or len(raw) > len(payload):
                            payload = raw
                    except Exception:
                        pass
                walk(v, depth + 1)
        elif isinstance(o, list):
            for v in o[:50]:
                walk(v, depth + 1)

    walk(resp)
    return payload, meta


def mode_pcd(args):
    print(">>> MODO pcd: descarga del .pcd del robot por WebRTC (slam_operate api 1934, getBigFile).")
    print("    Pre: app del iPhone CERRADA (sesion unica) · Mac en el AP del robot · venv de")
    print("    unitree_webrtc_connect activo (cd ~/unitree_webrtc_connect && source .venv/bin/activate)\n")
    try:
        import asyncio
        from unitree_webrtc_connect.webrtc_driver import (
            UnitreeWebRTCConnection, WebRTCConnectionMethod)
    except ImportError:
        print("  FALTA unitree_webrtc_connect. Activa el venv:")
        print("    cd ~/unitree_webrtc_connect && source .venv/bin/activate")
        return 1

    name = args.name or default_pcd_name()
    address = args.address or f"/unitree/data/unitree_slam/{name}.pcd"
    print(f"  fichero objetivo: {address}")

    SLAM_OPERATE = "rt/api/slam_operate/request"

    async def run():
        conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP, ip=args.ip)
        await conn.connect()
        print("  Conectado al robot.\n")
        ps = conn.datachannel.pub_sub

        async def req(data, timeout=15):
            opt = {"api_id": 1934, "parameter": {"data": data}}
            try:
                return await asyncio.wait_for(ps.publish_request_new(SLAM_OPERATE, opt),
                                              timeout=timeout)
            except asyncio.TimeoutError:
                return None

        # ---- probe inicial: solo address (algunas firmwares devuelven el fichero
        #      entero o el primer chunk + total) ----
        blob = bytearray()
        offset = 0
        total = None
        attempts = [
            {"address": address},
            {"address": address, "offset": 0, "length": args.chunk},
            {"address": address, "offset": 0, "size": args.chunk},
            {"address": address, "seq": 0},
        ]
        first = None
        used = None
        for cand in attempts:
            print(f"  probe -> {cand}")
            r = await req(cand)
            if r is None:
                print("     (timeout)")
                continue
            code = None
            try:
                code = r["data"]["header"]["status"]["code"]
            except Exception:
                pass
            payload, meta = _find_chunk(r)
            print(f"     RESP code={code} meta={meta} payload={len(payload) if payload else 0}B")
            if payload:
                first = (payload, meta)
                used = cand
                break
            if code == 0 and first is None:
                # respondio OK sin payload reconocible: enseñar la respuesta cruda
                print("     respuesta sin payload reconocible; muestra cruda:")
                print("     " + json.dumps(r, default=str)[:800])
        if first is None:
            print("\n  No he conseguido un chunk reconocible con api 1934 en este firmware.")
            print("  Pistas: (a) mira la respuesta cruda de arriba y adapta _find_chunk/attempts;")
            print("  (b) esnifa la app cuando descarga/carga el mapa (webview_live_inspect_steps.md);")
            print("  (c) plan B sin firmware: modo 'webview' de este mismo script (mapa entero).")
            return 1

        payload, meta = first
        blob += payload
        total = meta.get("total") or meta.get("filesize") or meta.get("total_size") or meta.get("size")
        offset = len(blob)
        print(f"\n  chunk 0: {len(payload)}B  (total anunciado: {total})")

        # ---- bucle de chunks si hace falta ----
        while total is None or offset < int(total):
            nxt = dict(used)
            if "seq" in nxt:
                nxt["seq"] = nxt.get("seq", 0) + (offset // max(1, len(payload)))
            else:
                nxt["offset"] = offset
                nxt.setdefault("length", args.chunk)
            r = await req(nxt)
            if r is None:
                print(f"  timeout en offset={offset} -> paro (guardo lo que hay)")
                break
            p2, m2 = _find_chunk(r)
            if not p2:
                if total is None:
                    print(f"  sin mas payload en offset={offset} -> asumo fichero completo")
                else:
                    print(f"  sin payload en offset={offset} (total={total}) -> paro")
                break
            blob += p2
            offset = len(blob)
            if total:
                print(f"  {offset}/{total} bytes ({100.0*offset/float(total):.0f}%)", end="\r")
            else:
                print(f"  {offset} bytes...", end="\r")
            if len(p2) < 16:
                break

        os.makedirs(args.out, exist_ok=True)
        raw_path = os.path.join(args.out, f"{name}.pcd")
        open(raw_path, "wb").write(bytes(blob))
        print(f"\n  RAW -> {raw_path}  ({len(blob)} bytes)")

        # si es un PCD legible, exporta tambien JSON/PNG
        try:
            pts = parse_pcd_bytes(bytes(blob))
        except Exception as e:
            print("  (no parsea como PCD:", repr(e), "- se queda el RAW; puede ser otro contenedor)")
            pts = []
        if pts:
            meta_out = {"source": f"robot getBigFile api1934 {address}", "frame": "map (Z-up)"}
            pts2d = [(p[0], p[1]) for p in pts if -0.5 <= p[2] <= 0.6]
            layers = [([(p[0], p[1]) for p in pts], dict(s=1, c="0.75", alpha=0.4, label="todas las alturas")),
                      (pts2d, dict(s=2, c="k", alpha=0.8, label="banda obstaculos [-0.5,0.6]m"))]
            save_all(args.out, f"map_robot_{name}", pts, meta_out, layers,
                     f"Mapa del robot {name}.pcd (firmware)", args.no_png)
        return 0

    return asyncio.run(run())


def parse_pcd_bytes(raw):
    """Parser minimo de PCD (ascii o binary, campos x y z al principio). -> [[x,y,z],...]"""
    import struct
    head, _, rest = raw.partition(b"DATA")
    if not rest:
        raise ValueError("sin cabecera DATA")
    lines = head.decode("ascii", "replace").splitlines()
    fields, sizes, types, npts = [], [], [], 0
    for ln in lines:
        t = ln.split()
        if not t:
            continue
        if t[0] == "FIELDS":
            fields = t[1:]
        elif t[0] == "SIZE":
            sizes = [int(v) for v in t[1:]]
        elif t[0] == "TYPE":
            types = t[1:]
        elif t[0] == "POINTS":
            npts = int(t[1])
    mode_line, _, body = rest.partition(b"\n")
    mode = mode_line.strip().decode("ascii", "replace")
    ix = [fields.index(k) for k in ("x", "y", "z") if k in fields]
    if len(ix) < 3:
        raise ValueError(f"faltan campos xyz en {fields}")
    pts = []
    if mode == "ascii":
        for ln in body.splitlines():
            t = ln.split()
            if len(t) >= len(fields):
                try:
                    pts.append([float(t[ix[0]]), float(t[ix[1]]), float(t[ix[2]])])
                except ValueError:
                    pass
    elif mode == "binary":
        stride = sum(sizes)
        offs = [sum(sizes[:i]) for i in range(len(sizes))]
        fmt = {("F", 4): "f", ("F", 8): "d", ("I", 4): "i", ("U", 4): "I"}
        for i in range(min(npts, len(body) // stride)):
            rec = body[i * stride:(i + 1) * stride]
            p = []
            for j in ix:
                f = fmt.get((types[j], sizes[j]), "f")
                p.append(struct.unpack_from("<" + f, rec, offs[j])[0])
            pts.append(p)
    else:
        raise ValueError(f"DATA {mode} no soportado (¿binary_compressed? usa open3d/pypcd)")
    return pts


# ----------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["local", "webview", "pcd"],
                    help="local=disco (refmap+nav_map) · webview=mapa cargado via app/USB · pcd=descarga del robot")
    ap.add_argument("--out", default=os.path.join(HERE, "maps_out"))
    ap.add_argument("--no-png", action="store_true")
    ap.add_argument("--secs", type=int, default=40, help="webview: segundos de captura")
    ap.add_argument("--frame", choices=["auto", "yup", "zup"], default="auto",
                    help="webview: frame de la nube capturada (auto detecta el eje vertical)")
    ap.add_argument("--name", default=None, help="pcd: nombre del mapa (defecto: pcd de waypoints.json)")
    ap.add_argument("--address", default=None, help="pcd: ruta completa en el robot (anula --name)")
    ap.add_argument("--chunk", type=int, default=65536, help="pcd: tamaño de chunk pedido")
    ap.add_argument("--ip", default="192.168.12.1", help="pcd: IP del robot (AP local)")
    args = ap.parse_args()
    fn = {"local": mode_local, "webview": mode_webview, "pcd": mode_pcd}[args.mode]
    sys.exit(fn(args) or 0)


if __name__ == "__main__":
    main()
