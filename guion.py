# -*- coding: utf-8 -*-
"""Ejecutor de guiones de escenificacion: convierte una configuracion T1-T12 en una run.

    python3 guion.py T3 --destino B

Un guion es una lista de (instante, estado declarado). El ejecutor escribe cada estado en
el fichero que vigila el adaptador, y ese acto ES el registro independiente: el §3 de la
pre-registracion exige que Omega_t venga del guion y no de la telemetria del sistema bajo
prueba, y aqui el guion es literalmente quien mueve la escena. delta_t no se infiere de
nada: se declara aqui, antes de correr.

Cubre hoy las siete configuraciones que dependen de luz y cristal. Las que faltan y por que:
  T5/T6  objeto      -> el emulador de deteccion carga sus objetos al arrancar; falta darle
                        el mismo canal (mismo patron, trabajo pequeno)
  T7     bateria     -> el gemelo NO tiene modelo de bateria (bat=100 siempre). Es la unica
                        que no puede escenificarse en el gemelo sin construir el modelo.
  T10    bloqueo     -> exige geometria nueva en el USD, no un cambio de escena
  T12    sucesor     -> propiedad del REGISTRO, no del mundo: no necesita canal
"""
import argparse, json, math, os, re, subprocess, sys, time

_POS_RE = re.compile(r"pos=\((?P<x>[-+0-9.]+),(?P<y>[-+0-9.]+)\)")

LUZ_ON, LUZ_OFF = 116.0, 85.0            # los dos estados declarados, medidos el 21-ago
CRISTAL_VANO = "-4.3,0.6,-3.5,1.9"       # rectangulo del flanco del vano (frame del mapa)

# (instante en s desde el arranque, estado declarado, delta_t esperado DESPUES del cambio)
GUIONES = {
    # T1/T2 con luz BAJA (cazado por el puntuador: con luz alta el RGB queda inadmisible por
    # contrato y la insuficiencia es CONJUNTA, no de solo-lidar -- el resolutor respondia
    # defer/illumination correctamente y el certificado exigia lidar_quality. Familia D2.
    "T1":  [(0.0, {"luz": LUZ_OFF, "cristal": CRISTAL_VANO}, "lidar_quality")],
    "T2":  [(0.0, {"luz": LUZ_OFF, "cristal": CRISTAL_VANO}, "lidar_quality"),
            (25.0, {"luz": LUZ_OFF, "cristal": ""},           "motion")],
    # T3/T4 EN LA DIRECCION MEDIDA (D2): en este sistema es la luz PLENA la que inadmite
    # el RGB (contrato congelado), asi que illumination gobierna con luz alta y motion con
    # luz baja -- el inverso del supuesto clasico de la tabla. La renumeracion es de Renxi.
    "T3":  [(0.0, {"luz": LUZ_OFF, "cristal": ""},           "motion"),
            (20.0, {"luz": LUZ_ON, "cristal": ""},            "illumination")],
    "T4":  [(0.0, {"luz": LUZ_ON,  "cristal": ""},           "illumination"),
            (20.0, {"luz": LUZ_OFF, "cristal": ""},           "motion")],
    # T8/T9 EN NUESTRO SISTEMA (cazado por A_Omega, 24-ago: C4=0% con el guion clasico).
    # El supuesto clasico dice "oscuro degrada el RGB"; nuestro contrato congelado mide lo
    # contrario (la luz plena lo inadmite). Insuficiencia conjunta AQUI = luz PLENA (RGB
    # inadmisible) + cristal (laser degradado). El retorno de T9: luz baja y cristal fuera.
    # transicion a t=28: el robot esta ya EN la aproximacion al vano, donde el cristal
    # domina la banda de cobertura (a t=18 aun estaba lejos y cov_n no caia)
    "T8":  [(0.0, {"luz": LUZ_OFF, "cristal": ""},           "motion"),
            ({"zona": [-3.90, 1.25, 3.0]}, {"luz": LUZ_ON, "cristal": CRISTAL_VANO}, "defer")],
    "T9":  [(0.0, {"luz": LUZ_ON,  "cristal": CRISTAL_VANO}, "defer"),
            (22.0, {"luz": LUZ_OFF, "cristal": ""},           "motion")],
    # T5/T6: el objeto entra/sale de la escena via el canal del emulador. Luz BAJA a
    # proposito: en nuestro sistema es la condicion con RGB admisible, y el objeto testigo
    # es la silla en su marca real validada (-2.98, 1.29), con linea de vista desde la
    # aproximacion al vano. delta object exige deteccion >=0.55 con el contrato admitiendo.
    "T5":  [(0.0, {"luz": LUZ_OFF, "cristal": "", "objetos": []}, "motion"),
            (15.0, {"luz": LUZ_OFF, "cristal": "", "objetos": [["chair", -2.98, 1.29]]}, "object")],
    "T6":  [(0.0, {"luz": LUZ_OFF, "cristal": "", "objetos": [["chair", -2.98, 1.29]]}, "object"),
            (20.0, {"luz": LUZ_OFF, "cristal": "", "objetos": []}, "motion")],
    # T7: bateria como TRAYECTORIA DECLARADA (D3). Cruza la banda de energia (60) dentro
    # de la run, con luz baja para que el RGB sea admisible y no interfiera la iluminacion.
    "T7":  [(0.0, {"luz": LUZ_OFF, "cristal": "", "bat": 75}, "motion"),
            (20.0, {"luz": LUZ_OFF, "cristal": "", "bat": 55}, "energy")],
    # T10: paso BLOQUEADO (mundo aparte: office3d_bloqueada.usd, ver campana). La silla
    # esta declarada e identificable; el bloqueo se descubre al llegar. delta se deriva por
    # zona de observacion en dcc_omega (no_use tras descubrir); aqui el guion es estatico.
    "T10": [(0.0, {"luz": LUZ_OFF, "cristal": "", "objetos": [["chair", -2.98, 1.29]], "bloqueo": [-3.90, 1.25]}, "no_use")],
    "T11": [(0.0, {"luz": LUZ_OFF, "cristal": ""},            "motion"),
            (20.0, {"luz": LUZ_ON, "cristal": ""},            "illumination")],  # ver D5
}

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("config", choices=sorted(GUIONES))
    ap.add_argument("--destino", default="B", choices=["A", "B"])
    ap.add_argument("--escena", default="/tmp/g1_escena.json")
    ap.add_argument("--registro", default="/tmp/g1_escena_registro.jsonl")
    ap.add_argument("--seco", action="store_true", help="solo imprime el guion")
    a = ap.parse_args()

    guion = GUIONES[a.config]
    print("== %s -> %s ==" % (a.config, a.destino))
    for t, est, d in guion:
        cuando = ("zona %.1f m de (%.2f, %.2f)" % (t["zona"][2], t["zona"][0], t["zona"][1])
                  if isinstance(t, dict) else "t=%5.1fs" % t)
        print("   %-28s %-46s  delta_t esperado: %s" % (cuando, json.dumps(est), d))
    if a.seco:
        return

    # estado inicial ANTES de arrancar el robot: el guion fija el mundo, no lo hereda
    json.dump(guion[0][1], open(a.escena, "w"))
    # CERTIFICADO DE REFERENCIA (Omega_t): se escribe EN EL MISMO ACTO que escenifica.
    # El §3 exige que la referencia venga del guion y no de la telemetria; aqui el guion es
    # quien mueve el mundo, asi que certificado y evento no pueden divergir -- no son dos
    # fuentes. Cada segmento: instante de pared, delta_t declarada y el estado que la
    # justifica. dcc_omega.carga_referencia lo mapea al tiempo de la run via `started`.
    ref_f = "/tmp/g1_omega_ref_%s.json" % a.config
    ref = {"config": a.config, "destino": a.destino,
           "segmentos": [{"t_pared": time.time(), "delta": guion[0][2],
                          "estado": guion[0][1]}]}
    json.dump(ref, open(ref_f, "w"))
    time.sleep(0.5)

    env = dict(os.environ,
               G1_SIM_ESCENA=a.escena, G1_SIM_ESCENA_LOG=a.registro,
               G1_SIM_URL=os.environ.get("G1_SIM_URL", "ws://localhost:8766"),
               G1_SIM_LUZ=str(guion[0][1]["luz"]),
               G1_SIM_GLASS=guion[0][1].get("cristal", ""),
               G1_DOOR_VIS="1", G1_DOOR_VIS_GATE="1", G1_METASM="1",
               G1_SIM_PERC="1", G1_NOVIS="0",
               G1_PERC=os.environ.get("G1_PERC", "127.0.0.1:8010"), G1_SIM_NOISE="1")

    t0 = time.time()
    proc = subprocess.Popen([sys.executable, "g1_sim_adapter.py", "goto", a.destino],
                            env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, bufsize=1)
    fichero = None
    pendientes = [g for g in guion[1:]]
    pos = None
    def _disparado(trig, ahora):
        # zona: la transicion se dispara por POSICION. Motivo (medido): el arranque del
        # adaptador varia varios segundos entre runs, y un instante fijo cae donde quiere
        # respecto a la geometria -- T8 a t=28 acabo aplicandose con el robot ya al otro
        # lado del vano. Una condicion ligada a la posicion se dispara por posicion; el
        # registro de pared sigue siendo el certificado.
        if isinstance(trig, dict) and "zona" in trig:
            if pos is None:
                return False
            zx, zy, zr = trig["zona"]
            return math.hypot(pos[0] - zx, pos[1] - zy) <= zr
        return ahora >= float(trig)
    try:
        for linea in proc.stdout:
            if "travesia guardada" in linea:
                fichero = linea.strip().split("-> ")[-1]
            m_ = _POS_RE.search(linea)
            if m_:
                pos = (float(m_.group("x")), float(m_.group("y")))
            ahora = time.time() - t0
            while pendientes and _disparado(pendientes[0][0], ahora):
                t, est, d = pendientes.pop(0)
                json.dump(est, open(a.escena, "w"))
                ref["segmentos"].append({"t_pared": time.time(), "delta": d, "estado": est})
                json.dump(ref, open(ref_f, "w"))
                print("  [GUION] t=%.1fs declarado %s (delta_t -> %s)" % (ahora, est, d), flush=True)
            if proc.poll() is not None:
                break
    finally:
        proc.wait()
    # el certificado viaja JUNTO a la run que certifica (paquete de reproducibilidad §15):
    # un fichero por config en /tmp se sobrescribia con cada repeticion de la campana.
    if fichero:
        import shutil
        destino_ref = fichero.replace(".json", "_omega_ref.json")
        try:
            shutil.copy(ref_f, destino_ref)
            print("certificado junto a la run: %s" % destino_ref)
        except Exception as e:
            print("AVISO: no se pudo copiar el certificado (%s)" % e)
    print("\nrun: %s" % (fichero or "(sin fichero)"))
    print("registro independiente: %s" % a.registro)
    print("certificado de referencia: %s" % ref_f)

if __name__ == "__main__":
    main()
