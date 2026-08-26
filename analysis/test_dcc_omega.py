import os
# -*- coding: utf-8 -*-
"""Controles del puntuador A_Omega: positivo, NEGATIVO y de frontera.

POR QUE EXISTE. En todo el material medido hasta hoy A_Omega = A_meta (escenificaciones de
un solo fundamento: un acierto solo puede citar lo correcto). Un discriminador que nunca ha
discriminado esta sin validar. Aqui se le da, con evaluadores sinteticos, el caso que el
material real aun no ha producido: la respuesta CORRECTA citando el fundamento EQUIVOCADO
-- A_meta debe puntuar 1 y A_Omega debe puntuar 0. Si este test pasa, la maquinaria que
distingue "reconstruido desde los fundamentos" de "acertado por accidente" funciona; cuando
el material real produzca el caso, la lectura sera fiable.
"""
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src"))
from dcc_omega import puntua_run

SEG = [{"desde": 0.0, "hasta": 100.0, "delta": "illumination", "estado": {"luz": 116}}]
RUN = {"samples": [{"t": float(t), "x": 0.0, "y": 0.0, "yaw": 0.0} for t in range(10, 20)]}

def evaluador(z, razon):
    def f(m, usa_pose=True):
        return {"C4": {"Z": z, "razon": razon, "authority": "controller",
                       "estado_incumbente": None}}
    return f

def caso(nombre, z, razon, meta_esp, omega_esp):
    r = puntua_run(RUN, SEG, evaluador(z, razon), True)["C4"]
    m, o = r["meta"] / r["n"], r["omega"] / r["n"]
    ok = (m == meta_esp and o == omega_esp)
    print("%-52s A_meta=%.0f A_Omega=%.0f  %s" % (nombre, m, o, "OK" if ok else "**FALLA**"))
    return ok

todo = True
# positivo: respuesta correcta, fundamento correcto
todo &= caso("illumination citando el contrato (correcto)",
             "illumination", "contrato:illum=116>99 RGB inadmisible", 1, 1)
# NEGATIVO: respuesta correcta, fundamento equivocado -> A_meta 1, A_Omega 0
todo &= caso("illumination citando la BATERIA (acierto por accidente)",
             "illumination", "bat=55<60 replanificar", 1, 0)
todo &= caso("illumination citando cobertura (acierto por accidente)",
             "illumination", "cobertura:cov_blind=0.5>=0.30", 1, 0)
# respuesta incorrecta: ambos 0 (A_Omega no puede superar A_meta jamas)
todo &= caso("motion (respuesta incorrecta)",
             "motion", "defecto:sin fundamento activo", 0, 0)
# frontera de FUNDAMENTO: prefijo que casi coincide no debe colar
todo &= caso("illumination con prefijo casi-correcto 'contratos'",
             "illumination", "contratos falsos:123", 1, 0)
print("\n%s" % ("TODOS LOS CONTROLES PASAN" if todo else "HAY FALLOS"))
sys.exit(0 if todo else 1)
