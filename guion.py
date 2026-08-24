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
import argparse, json, os, subprocess, sys, time

LUZ_ON, LUZ_OFF = 116.0, 85.0            # los dos estados declarados, medidos el 21-ago
CRISTAL_VANO = "-4.3,0.6,-3.5,1.9"       # rectangulo del flanco del vano (frame del mapa)

# (instante en s desde el arranque, estado declarado, delta_t esperado DESPUES del cambio)
GUIONES = {
    "T1":  [(0.0, {"luz": LUZ_ON,  "cristal": CRISTAL_VANO}, "lidar_quality")],
    "T2":  [(0.0, {"luz": LUZ_ON,  "cristal": CRISTAL_VANO}, "lidar_quality"),
            (25.0, {"luz": LUZ_ON, "cristal": ""},            "motion")],
    "T3":  [(0.0, {"luz": LUZ_ON,  "cristal": ""},            "motion"),
            (20.0, {"luz": LUZ_OFF, "cristal": ""},           "illumination")],
    "T4":  [(0.0, {"luz": LUZ_OFF, "cristal": ""},            "illumination"),
            (20.0, {"luz": LUZ_ON, "cristal": ""},            "motion")],
    "T8":  [(0.0, {"luz": LUZ_ON,  "cristal": ""},            "motion"),
            (18.0, {"luz": LUZ_OFF, "cristal": CRISTAL_VANO}, "defer")],
    "T9":  [(0.0, {"luz": LUZ_OFF, "cristal": CRISTAL_VANO},  "defer"),
            (22.0, {"luz": LUZ_ON, "cristal": ""},            "motion")],
    # T7: bateria como TRAYECTORIA DECLARADA (D3). Cruza la banda de energia (60) dentro
    # de la run, con luz baja para que el RGB sea admisible y no interfiera la iluminacion.
    "T7":  [(0.0, {"luz": LUZ_OFF, "cristal": "", "bat": 75}, "motion"),
            (20.0, {"luz": LUZ_OFF, "cristal": "", "bat": 55}, "energy")],
    "T11": [(0.0, {"luz": LUZ_OFF, "cristal": ""},            "motion"),
            (20.0, {"luz": LUZ_ON, "cristal": ""},            "motion")],   # el ROL no cambia
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
        print("   t=%5.1fs  %-46s  delta_t esperado: %s" % (t, json.dumps(est), d))
    if a.seco:
        return

    # estado inicial ANTES de arrancar el robot: el guion fija el mundo, no lo hereda
    json.dump(guion[0][1], open(a.escena, "w"))
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
    try:
        for linea in proc.stdout:
            if "travesia guardada" in linea:
                fichero = linea.strip().split("-> ")[-1]
            ahora = time.time() - t0
            while pendientes and ahora >= pendientes[0][0]:
                t, est, d = pendientes.pop(0)
                json.dump(est, open(a.escena, "w"))
                print("  [GUION] t=%.1fs declarado %s (delta_t -> %s)" % (ahora, est, d), flush=True)
            if proc.poll() is not None:
                break
    finally:
        proc.wait()
    print("\nrun: %s" % (fichero or "(sin fichero)"))
    print("registro independiente: %s" % a.registro)

if __name__ == "__main__":
    main()
