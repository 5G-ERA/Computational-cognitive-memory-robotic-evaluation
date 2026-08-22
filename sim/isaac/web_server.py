#!/usr/bin/env python3
"""Servidor web de la vista de Isaac. Proceso APARTE: solo lee /ws/live.jpg y escribe cam.json."""
import json, math, os, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

WS = "/home/ros/isaac_ws"
CAM = os.path.join(WS, "cam.json")
IMG = os.path.join(WS, "live.jpg")
E = {"x": 3.0, "y": -3.0, "z": 3.0, "tx": -3.5, "ty": 2.0, "tz": 0.9}

def guarda():
    tmp = CAM + ".tmp"
    json.dump(E, open(tmp, "w")); os.replace(tmp, CAM)
guarda()

PAGINA = """<!doctype html><meta charset=utf-8><title>Oficina G1 · Isaac</title>
<style>body{margin:0;background:#111;color:#ddd;font:14px system-ui;text-align:center}
img{max-width:100%;height:auto;display:block;margin:0 auto;background:#000;min-height:200px}
.c{padding:6px}button{font-size:15px;margin:2px;padding:7px 12px;background:#2a2a2a;color:#eee;
border:1px solid #444;border-radius:6px;cursor:pointer}button:hover{background:#3a3a3a}
small{color:#888}</style>
<div class=c><b>Oficina del G1 · Isaac Sim</b> &middot; <small id=f>—</small></div>
<img id=v>
<div class=c>
<button onclick="m('fwd')">avanzar</button><button onclick="m('back')">atras</button>
<button onclick="m('left')">izq</button><button onclick="m('right')">der</button>
<button onclick="m('up')">subir</button><button onclick="m('down')">bajar</button>
<button onclick="m('yawl')">girar &larr;</button><button onclick="m('yawr')">girar &rarr;</button>
<button onclick="m('pup')">mirar arriba</button><button onclick="m('pdn')">mirar abajo</button></div>
<div class=c><button onclick="m('top')">CENITAL</button><button onclick="m('door')">PUERTA</button>
<button onclick="m('sofas')">SOFAS</button><button onclick="m('robot')">JUNTO AL G1</button></div>
<div class=c><small id=s></small></div>
<script>
// Cada fotograma es una peticion independiente: si una se pierde, la siguiente lo arregla.
// Nada de streams largos que se puedan atascar.
let t0=Date.now(), n=0;
function tick(){
  const img=new Image();
  img.onload=()=>{document.getElementById('v').src=img.src; n++;
    if(n%10==0){const f=10000/(Date.now()-t0); t0=Date.now();
      document.getElementById('f').textContent=f.toFixed(1)+' fps';}
    setTimeout(tick, 40);};
  img.onerror=()=>setTimeout(tick, 400);
  img.src='/f.jpg?'+Date.now();
}
function m(a){fetch('/mov?a='+a).then(r=>r.json()).then(d=>{
  document.getElementById('s').textContent='cam ('+d.x.toFixed(1)+','+d.y.toFixed(1)+','+d.z.toFixed(1)+')';});}
document.addEventListener('keydown',e=>{const k={w:'fwd',s:'back',a:'left',d:'right',q:'down',
e:'up',ArrowLeft:'yawl',ArrowRight:'yawr',ArrowUp:'pup',ArrowDown:'pdn'}[e.key];
if(k){m(k);e.preventDefault();}});
tick();
</script>"""

def mover(a):
    dx, dy = E["tx"]-E["x"], E["ty"]-E["y"]
    n = math.hypot(dx, dy) or 1.0
    ux, uy = dx/n, dy/n; P = 0.9
    if a=="fwd":    E["x"]+=ux*P; E["y"]+=uy*P; E["tx"]+=ux*P; E["ty"]+=uy*P
    elif a=="back": E["x"]-=ux*P; E["y"]-=uy*P; E["tx"]-=ux*P; E["ty"]-=uy*P
    elif a=="left": E["x"]-=uy*P; E["y"]+=ux*P; E["tx"]-=uy*P; E["ty"]+=ux*P
    elif a=="right":E["x"]+=uy*P; E["y"]-=ux*P; E["tx"]+=uy*P; E["ty"]-=ux*P
    elif a=="up":   E["z"]+=0.7; E["tz"]+=0.35
    elif a=="down": E["z"]=max(0.2,E["z"]-0.7); E["tz"]=max(0.0,E["tz"]-0.35)
    elif a=="pup":  E["tz"]+=0.7
    elif a=="pdn":  E["tz"]-=0.7
    elif a in ("yawl","yawr"):
        g=math.radians(18 if a=="yawl" else -18); c_,s_=math.cos(g),math.sin(g)
        E["tx"]=E["x"]+(dx*c_-dy*s_); E["ty"]=E["y"]+(dx*s_+dy*c_)
    elif a=="top":  E.update(x=-1.0,y=1.5,z=24.0,tx=-1.0,ty=1.5,tz=0.0)
    elif a=="door": E.update(x=-1.8,y=-0.5,z=1.6,tx=-3.9,ty=1.25,tz=1.0)
    elif a=="sofas":E.update(x=-2.2,y=4.6,z=2.2,tx=-4.3,ty=2.6,tz=0.6)
    elif a=="robot":E.update(x=2.6,y=-1.0,z=1.6,tx=0.99,ty=0.57,tz=0.9)
    guarda(); return E

class H(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    def log_message(self,*a): pass
    def _send(self, code, ctype, body):
        self.send_response(code); self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control","no-store"); self.end_headers()
        self.wfile.write(body)
    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/":
            self._send(200, "text/html; charset=utf-8", PAGINA.encode())
        elif u.path == "/f.jpg":
            try:
                self._send(200, "image/jpeg", open(IMG, "rb").read())
            except Exception:
                self._send(503, "text/plain", b"sin fotograma")
        elif u.path == "/mov":
            self._send(200, "application/json",
                       json.dumps(mover((parse_qs(u.query).get("a") or ["nada"])[0])).encode())
        else:
            self._send(404, "text/plain", b"no")

s = ThreadingHTTPServer(("0.0.0.0", 8899), H)
s.daemon_threads = True
print("web en :8899", flush=True)
s.serve_forever()
