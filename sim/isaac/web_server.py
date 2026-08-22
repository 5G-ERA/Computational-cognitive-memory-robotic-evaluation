#!/usr/bin/env python3
"""Vista web de la oficina en Isaac — navegacion con RATON (camara orbital).

Proceso APARTE del render (sin GIL compartido): solo lee /ws/live.jpg y escribe /ws/cam.json.

Camara ORBITAL, que es como navega cualquier visor 3D:
    estado = punto de mira (tx,ty,tz) + distancia + yaw + pitch
    arrastrar        -> orbitar        rueda          -> acercar/alejar
    Shift+arrastrar  -> desplazar      doble click    -> mirar ahi
La posicion se deriva: cam = mira - dist * adelante(yaw,pitch).
"""
import json, math, os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

WS = "/home/ros/isaac_ws"
CAM, IMG = os.path.join(WS, "cam.json"), os.path.join(WS, "live.jpg")

# mira + orbita
S = {"tx": -2.0, "ty": 1.5, "tz": 0.9, "dist": 9.0, "yaw": 25.0, "pitch": 22.0}

def guarda():
    y, p = math.radians(S["yaw"]), math.radians(S["pitch"])
    fx, fy, fz = math.cos(p)*math.cos(y), math.cos(p)*math.sin(y), -math.sin(p)
    d = S["dist"]
    e = {"x": S["tx"] - d*fx, "y": S["ty"] - d*fy, "z": S["tz"] - d*fz,
         "tx": S["tx"], "ty": S["ty"], "tz": S["tz"]}
    tmp = CAM + ".tmp"
    json.dump(e, open(tmp, "w")); os.replace(tmp, CAM)
    return e
guarda()

PRESETS = {
 "top":   {"tx": -1.0, "ty": 1.5, "tz": 0.0, "dist": 22.0, "yaw": 90.0, "pitch": 88.0},
 "door":  {"tx": -3.9, "ty": 1.25, "tz": 1.0, "dist": 5.0, "yaw": 155.0, "pitch": 8.0},
 "sofas": {"tx": -4.3, "ty": 2.8, "tz": 0.7, "dist": 5.5, "yaw": 300.0, "pitch": 22.0},
 "robot": {"tx": 0.99, "ty": 0.57, "tz": 0.9, "dist": 3.2, "yaw": 200.0, "pitch": 12.0},
 "sala":  {"tx": -2.0, "ty": 1.5, "tz": 0.9, "dist": 11.0, "yaw": 25.0, "pitch": 25.0},
}

PAGINA = """<!doctype html><meta charset=utf-8><title>Oficina G1 · Isaac Sim</title>
<style>
html,body{margin:0;height:100%;background:#0e0e10;color:#ddd;font:13px system-ui;overflow:hidden}
#wrap{position:relative;width:100vw;height:100vh;display:flex;align-items:center;justify-content:center}
#v{max-width:100%;max-height:100%;cursor:grab;user-select:none;-webkit-user-drag:none;background:#000}
#v.drag{cursor:grabbing}
#hud{position:fixed;top:8px;left:10px;background:rgba(0,0,0,.55);padding:6px 10px;border-radius:8px;
line-height:1.5;pointer-events:none}
#bar{position:fixed;bottom:10px;left:50%;transform:translateX(-50%);background:rgba(0,0,0,.6);
padding:6px;border-radius:10px;display:flex;gap:6px}
button{font:13px system-ui;padding:6px 11px;background:#2b2b30;color:#eee;border:1px solid #45454c;
border-radius:6px;cursor:pointer}button:hover{background:#3a3a42}
b{color:#fff}small{color:#8a8a92}
</style>
<div id=wrap><img id=v draggable=false></div>
<div id=hud><b>Oficina del G1 · Isaac Sim</b><br>
<small>arrastrar: orbitar &middot; rueda: zoom &middot; Shift+arrastrar: desplazar &middot; doble clic: centrar</small><br>
<small id=st>—</small></div>
<div id=bar>
<button onclick="pre('sala')">sala</button><button onclick="pre('top')">cenital</button>
<button onclick="pre('door')">puerta</button><button onclick="pre('sofas')">sofás</button>
<button onclick="pre('robot')">junto al G1</button></div>
<script>
const v=document.getElementById('v'), st=document.getElementById('st');
let t0=Date.now(), n=0, fps=0;

// --- fotogramas: cada uno es una peticion independiente (nada que se atasque) ---
function tick(){
  const im=new Image();
  im.onload=()=>{v.src=im.src; n++;
    if(n%10==0){fps=10000/(Date.now()-t0); t0=Date.now(); info();}
    setTimeout(tick,35);};
  im.onerror=()=>setTimeout(tick,400);
  im.src='/f.jpg?'+Date.now();
}
let cam={};
function info(){st.textContent=(fps?fps.toFixed(1)+' fps · ':'')+
  'mira ('+(cam.tx||0).toFixed(1)+', '+(cam.ty||0).toFixed(1)+') · dist '+(cam.dist||0).toFixed(1)+' m';}

// --- envio de camara: acumula y manda a 25 Hz como mucho ---
let acc={dy:0,dp:0,dd:0,px:0,py:0}, pend=false;
function envia(){
  if(!pend) return;
  const a=acc; acc={dy:0,dp:0,dd:0,px:0,py:0}; pend=false;
  fetch(`/orb?dy=${a.dy.toFixed(3)}&dp=${a.dp.toFixed(3)}&dd=${a.dd.toFixed(3)}&px=${a.px.toFixed(3)}&py=${a.py.toFixed(3)}`)
    .then(r=>r.json()).then(d=>{cam=d; info();});
}
setInterval(envia,40);
function push(o){Object.keys(o).forEach(k=>acc[k]+=o[k]); pend=true;}

// --- raton ---
let arr=false, lx=0, ly=0, sh=false;
v.addEventListener('mousedown',e=>{arr=true; sh=e.shiftKey||e.button===2; lx=e.clientX; ly=e.clientY;
  v.classList.add('drag'); e.preventDefault();});
window.addEventListener('mouseup',()=>{arr=false; v.classList.remove('drag');});
window.addEventListener('mousemove',e=>{
  if(!arr) return;
  const dx=e.clientX-lx, dy=e.clientY-ly; lx=e.clientX; ly=e.clientY;
  if(sh) push({px:-dx*0.012, py:dy*0.012});
  else   push({dy:-dx*0.30, dp:dy*0.30});
});
v.addEventListener('wheel',e=>{push({dd:e.deltaY*0.006}); e.preventDefault();},{passive:false});
v.addEventListener('contextmenu',e=>e.preventDefault());
v.addEventListener('dblclick',()=>fetch('/pre?p=sala').then(r=>r.json()).then(d=>{cam=d;info();}));
function pre(p){fetch('/pre?p='+p).then(r=>r.json()).then(d=>{cam=d; info();});}

// --- teclado, por si acaso ---
addEventListener('keydown',e=>{
  const k={ArrowLeft:{dy:6},ArrowRight:{dy:-6},ArrowUp:{dp:-5},ArrowDown:{dp:5},
           '+':{dd:-0.8},'-':{dd:0.8},w:{dd:-0.8},s:{dd:0.8}}[e.key];
  if(k){push(k); e.preventDefault();}});
tick();
</script>"""

class H(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    def log_message(self, *a): pass
    def _s(self, code, ctype, body):
        self.send_response(code); self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store"); self.end_headers()
        self.wfile.write(body)
    def _estado(self):
        e = guarda()
        return json.dumps(dict(S, **{"x": e["x"], "y": e["y"], "z": e["z"]})).encode()
    def do_GET(self):
        u = urlparse(self.path); q = parse_qs(u.query)
        def f(k):
            try: return float((q.get(k) or ["0"])[0])
            except ValueError: return 0.0
        if u.path == "/":
            self._s(200, "text/html; charset=utf-8", PAGINA.encode())
        elif u.path == "/f.jpg":
            try: self._s(200, "image/jpeg", open(IMG, "rb").read())
            except Exception: self._s(503, "text/plain", b"sin fotograma")
        elif u.path == "/orb":
            S["yaw"] = (S["yaw"] + f("dy")) % 360.0
            S["pitch"] = max(-85.0, min(89.0, S["pitch"] + f("dp")))
            if f("dd"):
                S["dist"] = max(0.6, min(60.0, S["dist"] * math.exp(f("dd"))))
            px, py = f("px"), f("py")
            if px or py:
                # desplazar en el plano de la camara, escalado con la distancia
                y = math.radians(S["yaw"]); k = S["dist"] * 0.35
                rx, ry = -math.sin(y), math.cos(y)          # derecha de la camara
                fx, fy = math.cos(y), math.sin(y)           # adelante (plano)
                S["tx"] += (rx*px + fx*py) * k
                S["ty"] += (ry*px + fy*py) * k
            self._s(200, "application/json", self._estado())
        elif u.path == "/pre":
            p = (q.get("p") or ["sala"])[0]
            if p in PRESETS: S.update(PRESETS[p])
            self._s(200, "application/json", self._estado())
        else:
            self._s(404, "text/plain", b"no")

s = ThreadingHTTPServer(("0.0.0.0", 8899), H)
s.daemon_threads = True
print("web orbital en :8899", flush=True)
s.serve_forever()
