#!/usr/bin/env python3
"""Sonda del API de manipulacion DESDE la pestana SLAM (sin cambiar de pestana, sin UI).

LA HIPOTESIS (21-ago). La app bloquea gestos en su pestana de manipulacion, pero eso es UI:
el API de gestos (topic rt/api/arm/request; 7107 listar, 7108 reproducir, 7113 parar -- ver
docs/notes/G1_Air_SLAM_SOLVED.md par.4) viaja por el MISMO RTCDataChannel que nuestro driver ya
captura como window.__dc. Si un request inyectado ahi recibe respuesta, el cambio navegacion ->
manipulacion con robot parado se hace SIN abandonar la WebView SLAM: la localizacion no se toca.

QUE HACE CADA PASO:
  list   (INOFENSIVO: solo lectura, cero movimiento) prueba sobres candidatos del request 7107
         y muestra que respuesta llega. Si alguno devuelve la lista de gestos: hipotesis
         confirmada a nivel de canal.
  play   (MUEVE EL BRAZO; pide confirmacion) reproduce un gesto con el robot PARADO en zona
         despejada. Las piernas se bloquean durante la accion (FSM de accion: comportamiento
         conocido). Mide pose ANTES y DESPUES via window.__odom para ver si la localizacion
         sobrevive al gesto.

REGLAS: NUNCA con un goto en marcha (un solo cliente CDP: lo mataria). Robot de pie, quieto,
mando en mano. Play solo con zona despejada alrededor de los brazos.

USO:  python3 tools/sonda_brazo.py list
      python3 tools/sonda_brazo.py play "Cafe"
"""
import json
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 2)[0])
import g1_nav_v2 as g

# tap minimo: engancha el canal 'data' (como INSTALL_JS) + captura RESPUESTAS entrantes
TAP_JS = r"""
(function(){
  if(!window.__dcHook){ window.__dcHook = 1;
    var S = RTCDataChannel.prototype.send;
    RTCDataChannel.prototype.send = function(d){
      try{ if((this.label||'')==='data') window.__dc = this; }catch(e){}
      return S.apply(this, arguments);
    };
  }
  if(window.__dc && !window.__rxHook){ window.__rxHook = 1; window.__rx = [];
    window.__dc.addEventListener('message', function(ev){ try{
      var s = (typeof ev.data === 'string') ? ev.data : '';
      if(s && s.length < 4000) { window.__rx.push(s); if(window.__rx.length > 40) window.__rx.shift(); }
    }catch(e){} });
  }
  return JSON.stringify({dc: !!window.__dc, rx: !!window.__rxHook});
})()"""


def envia(cdp, obj):
    js = "(function(){if(!window.__dc)return 'no-dc';try{window.__dc.send(%s);return 'ok';}catch(e){return 'err:'+e;}})()" \
         % json.dumps(json.dumps(obj))
    return cdp.eval(js)


def lee_rx(cdp):
    s = cdp.eval("JSON.stringify(window.__rx||[])")
    try:
        return json.loads(s) if s else []
    except Exception:
        return []


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ("list", "play"):
        sys.exit(__doc__)
    modo = sys.argv[1]
    cdp = g.CDP(g.discover_ws())          # cliente ligero; NO reinstala el driver de navegacion
    print("tap:", cdp.eval(TAP_JS))
    time.sleep(1.0)
    print("tap2:", cdp.eval(TAP_JS))     # segundo intento: __dc aparece cuando la app envia algo

    if modo == "list":
        # sobres candidatos para el request 7107 (solo LECTURA). El que responda, gana.
        candidatos = [
            {"type": "msg", "topic": "rt/api/arm/request", "data": {"api_id": 7107}},
            {"type": "req", "topic": "rt/api/arm/request", "data": {"api_id": 7107}},
            {"type": "request", "topic": "rt/api/arm/request", "data": {"api_id": 7107}},
            {"type": "msg", "topic": "rt/api/arm/request",
             "data": {"header": {"identity": {"id": int(time.time()), "api_id": 7107}},
                      "parameter": ""}},
        ]
        for i, c in enumerate(candidatos):
            antes = len(lee_rx(cdp))
            r = envia(cdp, c)
            time.sleep(1.5)
            rx = lee_rx(cdp)[antes:]
            arm = [m for m in rx if "arm" in m or "action" in m]
            print("\n[%d] type=%-8s -> envio:%s  respuestas nuevas:%d  con-pinta-de-arm:%d"
                  % (i, c["type"], r, len(rx), len(arm)))
            for m in arm[:2]:
                print("    ", m[:220])
        print("\nSi ningun sobre respondio: pegame TODO lo de window.__rx (puede que el 'type'")
        print("sea otro). Si uno listo gestos: hipotesis CONFIRMADA a nivel de canal.")

    else:
        nombre = sys.argv[2] if len(sys.argv) > 2 else "Cafe"
        print("\n>>> VA A MOVER EL BRAZO (gesto '%s'), robot PARADO, zona despejada." % nombre)
        if input("    escribe SI para continuar: ").strip().upper() != "SI":
            sys.exit("abortado")
        p0 = cdp.eval("JSON.stringify(window.__odom||null)")
        print("pose antes:", p0)
        r = envia(cdp, {"type": "msg", "topic": "rt/api/arm/request",
                        "data": {"api_id": 7108, "parameter": json.dumps({"action_name": nombre})}})
        print("envio:", r, "(esperando 8s el gesto...)")
        time.sleep(8.0)
        p1 = cdp.eval("JSON.stringify(window.__odom||null)")
        print("pose despues:", p1)
        print("respuestas:", [m[:160] for m in lee_rx(cdp)[-4:]])
        print("\nSi la pose apenas cambio y el robot volvio a estar de pie normal: el cambio")
        print("nav->manipulacion sin perder localizacion queda DEMOSTRADO.")


if __name__ == "__main__":
    main()
