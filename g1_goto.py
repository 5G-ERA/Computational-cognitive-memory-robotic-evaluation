#!/usr/bin/env python3
"""
g1_goto.py  -  NAVEGACION A->B sobre un MAPA CARGADO en la app (relocalizacion).

Reusa todo g1_nav_v2 (A*, DWA, costmap, camara, contacto IMU). Flujo:
  1) reloccheck   -> con el mapa cargado y relocalizado en la app, ver que datos llegan (pose/nube/camara)
  2) waypoint A   -> conduces el robot al destino y Ctrl+C; guarda la ULTIMA pose como 'A' en waypoints.json
                     (a la vez acumula el mapa 2D de obstaculos en nav_map.json)
  3) (el mapa 2d se va guardando solo en waypoint; tambien 'sweep' para mapear sin guardar waypoint)
  4) goto         -> menu en vivo: pides A/B/C..., A*+DWA te lleva y para

PRE: app de Unitree con el MAPA CARGADO y el robot RELOCALIZADO (de pie, en modo nav), ios_webkit_debug_proxy.
"""
import sys
import os
import time
import json
import math
import threading
from collections import deque
import g1_nav_v2 as g                      # reusa conexion + A* + DWA + costmap + camara + helpers
try:
    import g1_perception                    # cliente del servidor GPU offboard (opcional; via G1_PERC=host:port)
except Exception:
    g1_perception = None
import g1_metrics                            # metricas SEI: clearance (percepcion) + progression (rendimiento)
try:
    import g1_meta2_bridge                   # (para _resolve_path: config/ y state/ tras la reorg)
    from g1_meta2_bridge import Meta2Bridge  # DCE runtime (Meta-Reasoner 2.0 de Renxi), opcional
except Exception:
    Meta2Bridge = None
META2_MODE = os.environ.get("G1_META2", "0") # 0=off | 1=SHADOW (decide+loguea, no toca control) | 2=ACTIVO (techo de velocidad por analogia)
VCAP = (float(os.environ["G1_VCAP"]) if os.environ.get("G1_VCAP") else None)   # techo fijo M1 (sin gobernanza)
# --- EXT E: LIMITADOR DE JERK DE COMANDO (politica de bajo nivel / Worker; Adrian 2026-07-21).
# Mineria de 133 marcas reales: saltos de AVANCE >=0.30 en los 1.5s previos a una marca son
# 5.7x mas frecuentes que en baseline (22% vs 4%); los saltos de giro NO muestran asociacion
# (0.6x) — el giro sostenido ya lo doma el turn cap (Ext B); lo que derrama es el ESCALON.
# Asimetrico: acelerar/invertir en rampa; REDUCIR hacia cero pasa INSTANTANEO (las frenadas
# de HARD-GUARD/COLOR-BRAKE/HELP no se suavizan). G1_SLEW=1 activa (default OFF para pares
# con/sin en el laboratorio). Limites por tick (~0.1s): 0.08 avance / 0.12 giro.
SLEW_ON = os.environ.get("G1_SLEW", "") == "1"
SLEW_LIN = float(os.environ.get("G1_SLEW_LIN", "0.08"))
SLEW_ANG = float(os.environ.get("G1_SLEW_ANG", "0.12"))
# --- ENTORNO de la prueba (pedido del tutor 2026-07-03, campaña de simulacion): separa las runs de
# SIMULACION de las de robot REAL en dataset/log/CSV para que jamas se mezclen en el analisis.
#   G1_ENV=real (defecto) | sim      ·  G1_SIM_ID=<etiqueta libre del contenedor/escenario> (opcional)
RUN_ENV = os.environ.get("G1_ENV", "real").strip().lower() or "real"
if RUN_ENV not in ("real", "sim"):
    print(f"  AVISO: G1_ENV='{RUN_ENV}' no es 'real' ni 'sim'; se registra tal cual.")
# --- ESCALADA DE EXPERIENCIA (supervisor, 2026-07-03, run 100927: 584s en bucle en la puerta con
# FALLBACK 68% + HELP 14% y nadie abortaba): "the experience should inform the robot that all
# actions are not good and abort". Si la gobernanza sostiene que NINGUNA analogia sirve Y no hay
# progreso, la decision correcta es ABORTAR (HELP del paper = stop/request intervention), no
# reintentar para siempre. En SHADOW solo avisa (META2-ABORT-SHADOW); en ACTIVO aborta de verdad.
META2_ABORT = (os.environ.get("G1_M2_ABORT", "1") == "1")     # G1_M2_ABORT=0 desactiva la escalada
M2_ABORT_WIN = float(os.environ.get("G1_M2_ABORT_WIN", "75"))   # s de ventana de experiencia
M2_ABORT_BAD = float(os.environ.get("G1_M2_ABORT_BAD", "0.6"))  # fraccion de decisiones FALLBACK/HELP firmes
M2_ABORT_PROG = float(os.environ.get("G1_M2_ABORT_PROG", "0.4"))  # m de acercamiento minimo al goal en la ventana
M2_HELP_S = float(os.environ.get("G1_M2_HELP_S", "8"))          # s de HELP firme CONTINUO -> abort directo

_DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")   # reorg 2026-08-07
WP_FILE = os.path.join(_DATA, "waypoints.json")
MAP_FILE = os.path.join(_DATA, "nav_map.json")
GOTO_LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "goto.log")
GOTO_LOG_MAX_MB = float(os.environ.get("G1_GOTO_LOG_MAX_MB", "25"))  # umbral de rotacion; GitHub avisa a partir de 50 MB por fichero

def _open_goto_log():
    """Abre goto.log en append, rotando ANTES si supera G1_GOTO_LOG_MAX_MB (MB).
    La rotacion comprime el log a archive/goto_hasta_<ts>.log.gz (gitignored, queda
    solo en esta maquina) y arranca un goto.log nuevo con una linea de cabecera.
    OJO: goto.log es append-only compartido Mac/GPUEDGE (merge=union en
    .gitattributes). Tras una rotacion, commitea el goto.log pequeno y haz pull
    en la OTRA maquina antes de que vuelva a escribir, o el union-merge lo
    re-mezclara con su copia grande. Protocolo: docs/GOTO_LOG_ROTATION.md"""
    try:
        if os.path.exists(GOTO_LOG) and os.path.getsize(GOTO_LOG) >= GOTO_LOG_MAX_MB * 1024 * 1024:
            import gzip, shutil
            adir = os.path.join(os.path.dirname(GOTO_LOG), "archive")
            os.makedirs(adir, exist_ok=True)
            dst = os.path.join(adir, f"goto_hasta_{time.strftime('%Y%m%d_%H%M%S')}.log.gz")
            with open(GOTO_LOG, "rb") as fin, gzip.open(dst, "wb") as fout:
                shutil.copyfileobj(fin, fout)
            with open(GOTO_LOG, "w") as f:
                f.write(f"=== ROTADO {time.strftime('%Y-%m-%d %H:%M:%S')} -> archive/{os.path.basename(dst)} ===\n")
            print(f"  [goto.log] >{GOTO_LOG_MAX_MB:.0f} MB: rotado a archive/{os.path.basename(dst)}")
            print("  [goto.log] AVISO: commitea la rotacion y haz pull en la otra maquina (Mac/GPUEDGE) antes de su proximo run")
    except Exception as e:
        print(f"  [goto.log] rotacion fallida (sigo con el log actual): {e!r}")
    return open(GOTO_LOG, "a")

# --- nube 'location' (frame del MAPA, Z-up): idx0=x, idx1=y, idx2=altura. CONFIRMADO con reloc_cloud.json ---
HBAND_LO = float(os.environ.get("G1_HBAND_LO", "-0.5"))   # borde INFERIOR banda de altura (m) para OBSTACULOS.
                                 # SUBIDO de -0.9 a -0.5: a -0.9 el suelo (~-1.0) entraba como obstaculo al cabecear
                                 # la marcha -> cientos de celdas falsas. -0.5 lo evita. (override: G1_HBAND_LO)
HBAND_HI = float(os.environ.get("G1_HBAND_HI", "0.6"))    # borde SUPERIOR (excluye techo ~+1.3)
NAV_REACH = 0.35                 # m: se considera ALCANZADO el waypoint
NAV_OMAP_TTL = 60.0              # s: la nube es estatica; TTL medio purga obstaculos dinamicos (persona que pasa)
GATE_M = 0.6                     # m: si arrancas a > esto del waypoint mas cercano = relocalizacion dudosa (como la app)
AGGR_AFTER = 12.0                # s atascado sin ACERCARSE al destino -> activa modo AGRESIVO (cruza la puerta)
AGGR_ROBOT_R = float(os.environ.get("G1_AGGR_R", "0.24"))   # m: holgura en modo agresivo (MINIMO de seguridad).
# ^ 0.13->0.20 (Renxi 2026-07-02: "clearing is wrong, it does not cover the width of the hands"): el
#   semiancho FISICO del G1 es ~0.22m de hombros y ~0.28-0.30m con el vaivén de brazos; 0.13 autorizaba
#   huecos imposibles (roces con omap_near 42-81 en 152030/152330/152532). G1_AGGR_R=0.13 para revertir.
# ^ 0.20->0.24 (Renxi 2026-07-02 tarde, run 171431: "increase clearance safety just a little"): col2 fue
#   AGR-DOOR-GO empujando 3.5s con c0 0.62->0.38 hasta atrapar el HOMBRO derecho en el marco (-3.57,1.09);
#   con 0.24 el DWA corta el avance antes de meter el hombro en huecos <~0.45m. G1_AGGR_R=0.20 revierte.
GLOBAL_SRC = os.environ.get("G1_GLOBALMAP", "static")        # plan GLOBAL: static (DEFECTO 2026-07-02 P8b:
                                 # paredes=inf sin inflar + muebles conocidos en coste BLANDO) | hard (P8 v1:
                                 # todo pared dura -- MEDIDO offline: con INFL_HARD=1 sella la puerta de ~0.8m
                                 # y el A* caia SIEMPRE al fallback sin mapa duro) | ref | live
GLOB_SOFT = float(os.environ.get("G1_GLOB_SOFT", "9.0"))     # coste blando por celda de MUEBLE conocido (plan global)
GLOB_HALO = float(os.environ.get("G1_GLOB_HALO", "4.0"))     # coste blando del halo de 1 celda alrededor del mueble
GLOB_WALL_HALO = float(os.environ.get("G1_GLOB_WHALO", "6.0"))  # coste blando 1 celda junto a PARED (run 171431:
                                 # el plan pegaba el carrot a 0.2m de pared/cajonera -> mano dcha a la cajonera
                                 # en (-1,0.5); con halo el plan prefiere el CENTRO del vano y se despega de los
                                 # bordes sin sellar nada (es coste, no pared). G1_GLOB_WHALO=0 revierte.
                                 # Run 122857: c0min=0.16 y roces laterales en pared/marco -> con el bamboleo
                                 # del bipedo, 0.13 es rozar por diseno. Probar G1_AGGR_R=0.16 si repite (A/B).
PERC_PERIOD = 0.3                # s entre consultas al servidor de percepcion GPU (depth->scan virtual de la mesa)
DOOR_CENTER = (os.environ.get("G1_DOOR_CENTER", "1") == "1")   # centrar izq/dcha en la puerta (idea de Renxi): strafe al lado mas libre
DOOR_BAL_TH = 0.22               # |clear_left - clear_right| (normalizado) para considerar el robot DESCENTRADO
DOOR_STRAFE = 0.34               # magnitud del strafe lateral (> deadzone ~0.3, si no el robot no se mueve)
# CENTRADO EN EL VANO (13-ago-2026). UNICO cambio de esta rama respecto a golden-doorvis.
# El servo de strafe al eje ya existia con dos puertas que se comen el margen: solo corrige
# con desvio > 0.14 m y DEJA de corregir a 0.35 m del vano. Medido en 30 travesias reales:
# todas llegan a ~0.35 m del eje a 2 m; las limpias se recentran a ~0.01, las que fallan se
# quedan en 0.19 -- y el margen fisico es +-0.20 m (vano 0.99, semiancho real 0.29), asi que
# una zona muerta de 0.14 se come el 70% del margen. Confirmado tambien en el gemelo.
# Ambas puertas quedan como variables; los DEFAULTS reproducen golden EXACTAMENTE.
DOOR_CTR_TOL = float(os.environ.get("G1_DOOR_CTR_TOL", "0.14"))  # m de desvio tolerado
DOOR_CTR_S = float(os.environ.get("G1_DOOR_CTR_S", "0.35"))      # m antes del vano donde deja de corregir
# --- CENTRADO POR RITMO (14-ago). MEDIDO, no supuesto: en las 8 runs B->A de hoy las cuatro
# que llegaron y las cuatro que fallaron entran al ultimo metro con el MISMO desvio lateral
# (+1.02..+1.10 m a su izquierda). No es una deriva: es la geometria de la aproximacion. Lo que
# las separa es el RITMO al que vuelven al eje, y separa perfecto, sin solape:
#     llegan  0.027 0.028 0.072 0.082 m/s      fallan  0.016 0.017 0.022 0.023 m/s
# Tiene que cerrar ~1.05 m antes del vano; por debajo de ~0.025 m/s no le da tiempo, entra a
# +0.25 y engancha el marco. Por eso apretar G1_DOOR_CTR_TOL no sirvio (3/7 hoy): cambia CUANDO
# se dispara la correccion, no a que velocidad mueve -- la run 153451 disparo 12 correcciones y
# aun asi entro a +0.29.
# LA CAUSA ESTA EN QUE STRAFE Y AVANCE SON EXCLUYENTES: ENG-C mueve de lado con avance 0 y
# ENG-GO avanza sin corregir. Con 4 ticks de ENG-C contra 69 de ENG-GO, el ritmo lateral medio
# es una fraccion minima del strafe real. No hay que correr mas de lado: hay que dejar de gastar
# el 95% de los ticks sin corregir.
# QUE HACE G1_DOOR_CTR2=1 (OFF por defecto):
#   |lat| > HOLD          -> strafe PURO, avance 0: no se entra al vano descentrado.
#   TOL < |lat| <= HOLD   -> strafe Y avance A LA VEZ, en el mismo comando.
#   |lat| <= TOL          -> ENG-GO de siempre.
# Con red de seguridad: si el hold no reduce el desvio en HOLD_S segundos, avanza igualmente,
# porque un robot parado en la puerta es un run perdido y dispara el aborto por falta de progreso.
DOOR_CTR2 = os.environ.get("G1_DOOR_CTR2", "") == "1"
DOOR_CTR_HOLD = float(os.environ.get("G1_DOOR_CTR_HOLD", "0.25"))   # m: por encima, no avanza
DOOR_CTR_VY = float(os.environ.get("G1_DOOR_CTR_VY", "0.28"))       # avance simultaneo (= el de ENG-GO)
DOOR_CTR_HOLD_S = float(os.environ.get("G1_DOOR_CTR_HOLD_S", "6.0"))  # s maximos parado corrigiendo
# --- RUMBO SIN PARAR (G1_DOOR_YAW2, 14-ago). El arreglo lateral funciono: la run 165706 entro
# al vano a lat -0.03 (los fallos entran a -0.23) y aun asi no cruzo. Se quedo DENTRO del vano
# oscilando: avanza, el rumbo deriva, al pasar de DOOR_REALIGN (14 deg) para a realinear, y
# realineando RETROCEDE; vuelve a entrar y se repite -- 8 realineaciones, 0.21 m perdidos, y el
# vigilante de progreso lo mata (0.32 m en 74 s) sin haber acumulado los 0.75 m del cruce.
# NO es un sesgo de giro que se pueda compensar: en 51 tramos la deriva mediana es +0.1 deg/s y
# va a la izquierda el 51% de las veces. Lo que varia es el sesgo POR RUN (-4 a +3 deg/s), asi
# que no hay feed-forward posible; hay que corregir en lazo sin pagar el paron.
# QUE HACE: si esta BIEN CENTRADO y el error de rumbo no pasa de HARD, gira y avanza en el mismo
# comando (fase ENG-RG) en vez de pararse. Fuera de eso, la realineacion de siempre -- el limite
# duro sigue existiendo porque avanzar muy torcido es lo que engancha el marco.
DOOR_YAW2 = os.environ.get("G1_DOOR_YAW2", "") == "1"
DOOR_YAW_HARD = float(os.environ.get("G1_DOOR_YAW_HARD", "25.0"))   # deg: por encima, parar como antes
DOOR_YAW_LAT = float(os.environ.get("G1_DOOR_YAW_LAT", "0.12"))     # m: solo si esta centrado
# --- CENTRADO DE SALIDA (G1_DOOR_EXIT_CTR, 14-ago). La run 170557 cruzo por fin, pero el brazo
# IZQUIERDO rozo el marco al salir. La traza lo explica: entra impecable (lat -0.01..+0.08) y se
# descentra SALIENDO hasta -0.19 (negativo = su izquierda, el brazo que rozo). Y mientras se iba,
# la fase era ENG-GO -- es decir, el robot se creia centrado.
# POR QUE: el servo apunta al centro MEDIDO (door_c_meas), que sale de detectar las dos jambas.
# Ya dentro del vano las jambas quedan a los lados, dentro de la banda ciega del laser, asi que la
# medida se queda vieja o sesgada y el robot servoa a un centro que no existe.
# QUE HACE: pasado el centro del vano, ignora la medida y servoa al eje del MAPA hasta haber
# salido de verdad. No lleva direccion fija: empuja al lado que toque, asi que no puede meterlo
# contra la jamba por equivocarse de signo.
# OJO A LA METRICA: el detector de colisiones dio 0 en esa run y hubo contacto real. Va por
# odometria e IMU y un roce de brazo no perturba la base. ncol=0 NO significa limpio.
DOOR_EXIT_CTR = os.environ.get("G1_DOOR_EXIT_CTR", "") == "1"
# --- COBERTURA DE LIDAR: el campo prospectivo que faltaba en la interfaz (17-ago) --------------
# POR QUE HACE FALTA. La interfaz revisada de Renxi exige "lidar coverage or health state", y
# ninguna senal existente sirve. Medido antes de escribir esto:
#   laser_trust      -> SOLO baja tras colisionar (-0.25 por colision, +0.001/tick de recuperacion):
#                       es validez RETROSPECTIVA, vale 1.00 en toda run limpia y no puede activar
#                       un rol que debe encenderse ANTES del fallo. En el vocabulario del paper es
#                       preservacion de consecuencia, no reconstruccion del rol actual.
#   meta_state=BLIND -> mide PROXIMIDAD, no ceguera: el 97-99% de sus muestras son solo c0<1.0.
#                       Usarlo confundiria "hay algo cerca" con "no puedo ver" -> viola Semantic
#                       Locality, que exige mantener esos fundamentos separados.
#   c0 - c0_hard     -> sin discriminacion: mediana 0.00 y p90 0.00 sobre 214k muestras reales.
# QUE MIDE. Para el sector frontal se trazan rayos sobre el MAPA de referencia; en cada rumbo
# donde el mapa predice retorno dentro de alcance se comprueba si el barrido vivo lo dio. La
# fraccion de retornos PREDICHOS Y AUSENTES es el deficit de cobertura.
#   cristal      -> el mapa dice pared y el barrido no devuelve nada: deficit alto y localizado
#   vano abierto -> el mapa tampoco predice retorno: sin deficit
# Eso separa las dos mitades del par W1 (cristal vs vano), que es exactamente la distincion que el
# Teorema 1 obliga a restaurar en la interfaz.
# SOLO OBSERVA: no entra en ninguna decision de control. La particion de autoridad del paper exige
# que la evidencia no se convierta en permiso de mando por si sola.
COV = os.environ.get("G1_COV", "1") == "1"                      # G1_COV=0 lo apaga
COV_SECT = float(os.environ.get("G1_COV_SECT", "40.0"))         # grados a cada lado del rumbo
COV_R = float(os.environ.get("G1_COV_R", "3.0"))                # m de alcance del trazado
COV_NRAY = int(os.environ.get("G1_COV_NRAY", "25"))             # rayos en el sector
# Referencia de VISIBILIDAD de sesion (21-ago): fichero points (formato ref_map) con lo que el
# G1 ve con fiabilidad -- tools/mapa_visibilidad.py la construye de laser_snapshots. Si se da,
# ademas de cov_def se emite cov_missing: n de celdas de ESA referencia predichas y AUSENTES en
# 2 barridos frescos seguidos (localizado y persistente; el disparador nuevo del resolutor DCC,
# validado con cristal sintetico sobre las runs del 20-ago). SOLO emision: no toca el control.
COV_REF_FILE = os.environ.get("G1_COVREF", "")
DOOR_STRAFE_SIGN = int(os.environ.get("G1_STRAFE_SIGN", "-1"))  # DEFAULT -1 (2026-07-02): MEDIDO en runs
                                 # 123933 (46 ticks orden izq -> 38cm a la DERECHA) y 122857 (51 ticks, 98cm
                                 # contra lo ordenado): el mapeo fisico de lx esta INVERTIDO. DOOR-CTR centraba
                                 # HACIA el obstaculo. Verificar con STRAFE-CAL en el log; G1_STRAFE_SIGN=1 lo revierte.
DOOR_MIN_GOAL = 1.3              # m: por debajo de esta distancia a B NO hay puerta (es el goal con un mueble):
_DOOR_GATEFIX = os.environ.get("G1_DOOR_GATEFIX", "") == "1" or os.environ.get("G1_M2_DOORLIB", "") == "1"
                                 # desactiva la maniobra de puerta y deja que el DWA rodee el obstaculo (si no, empujaba recto)
# --- ENGAGEMENT de puerta ANCLADO AL MAPA ESTATICO (Renxi/Adrian 2026-07-02 tarde) ---
# Runs 171431/172840/173422: TODAS las colisiones en (-3.75,1.22) con yaw 101-119 cuando el eje del vano
# es ~135 -> el robot entra 20-30 grados under-rotated y el HOMBRO DERECHO lidera contra el marco. La
# maniobra DOOR-* solo saltaba "ya en zona estrecha" (c0<0.9), y c0 parpadea a 2.50 en la boca (marco
# LiDAR-ciego) -> aproximaciones en DWA-F a 0.40 sin alinear. Fix: posicion de PRE-ENTRADA fija en el eje
# del vano (conocido del mapa estatico) -> ir alli -> PARAR -> rotar hasta |err|<=8 -> cruzar RECTO
# (re-alinea si el bipedo deriva >14). "No importa pararse un segundo y calcular bien" (Adrian).
DOOR_ENGAGE = (os.environ.get("G1_DOOR_ENGAGE", "1") == "1")   # G1_DOOR_ENGAGE=0 revierte
DOOR_CX = float(os.environ.get("G1_DOOR_X", "-3.90"))    # centro del vano (frame G1, del mapa estatico)
DOOR_CY = float(os.environ.get("G1_DOOR_Y", "1.25"))
# --- FIXES DE CAPACIDADES (2026-07-21, autopsia de las 10 colisiones reales; ver docs) ---
# FIX C: celdas del MARCO DE PUERTA pegajosas. La jamba fina en incidencia rasante devuelve
# pocos puntos y cae dentro de NEAR_BLIND durante el propio cruce -> el filtro K-de-N + el
# decaimiento la hacian PARPADEAR en c0_hard (0.7<->2.5, golpe 20260714_174051). El marco es
# estatico por definicion: celda de la zona de puerta confirmada en >=2 barridos frescos queda
# FIJADA el resto de la run (bypass del descarte de campo cercano). G1_DOORSTICKY=0 revierte.
DOOR_STICKY = os.environ.get("G1_DOORSTICKY", "1") == "1"
# CENTRADO SENSOR-RELATIVO DEL VANO (Adrian 2026-07-21: "garantizar que encuentre el centro").
# Las 2 colisiones reales del dia estan EN la estructura del vano ((-3.74,1.17) y (-4.53,1.80)):
# el enganche apunta al centro DEL MAPA y ±10-15cm de error de localizacion se comen el margen
# (vano ~0.85 - robot ~0.60). Solucion: medir el centro con el LASER (bordes internos de las
# dos jambas en el mapa vivo — DOORSTICKY las mantiene mientras cruzas) y servo al centro
# MEDIDO: el error de localizacion se cancela (robot y jambas en el mismo marco). Correccion
# acotada a ±0.30m. G1_DOOR_CENTER=0 revierte al centro del mapa.
DOOR_CENTER = os.environ.get("G1_DOOR_CENTER", "1") == "1"
DOOR_CENTER_MAX = float(os.environ.get("G1_DOOR_CENTER_MAX", "0.30"))
# SUELO DETERMINISTA DE ZONA-VANO (auditoria 2026-07-22, hallazgo #1 confirmado): tras un
# abort del FSM (cooldown 8s), un A*-fail o con la fragilidad dormida (marcador caido, como
# el 21-jul), el robot puede cruzar el vano con DWA a 0.40 y sin techo (Efficient cap=None).
# Guarda ADITIVA e independiente del razonador: a <1.2m del centro del vano y FUERA de las
# fases de negociacion (ENG/AGR/ESC/recuperaciones), avance <=0.28 y giro <=0.38 (los valores
# del propio ENG-GO/perfil Cautious). El suelo del hueco del veto L2. G1_DOORGUARD=0 revierte.
DOORGUARD = os.environ.get("G1_DOORGUARD", "1") == "1"
DOORGUARD_R = float(os.environ.get("G1_DOORGUARD_R", "1.2"))
# DOOR-VIS ("seguir la linea morada", Adrian 24-jul): en el enganche, si la vision ve el
# vano (detector de puerta del perception_server, bearing fresco y coherente con el mapa),
# el error de rumbo pasa a ser el BEARING VISUAL — el robot apunta a la ABERTURA REAL que
# esta viendo, no a la constante DOOR_AXIS del mapa. Cierra en lazo lo que los sesgos por
# direccion (BiasPlus etc.) compensaban en abierto: la asimetria B->A, los +-10-15cm de
# reloc y el descentrado lateral (por paralaje, apuntar al centro desde fuera del eje YA
# ES la correccion). Evidencia: run 163117 (BiasPlus B->A): 7 oscilaciones del plano en
# 50s culebreando, mientras door_b veia el vano limpio (103 muestras, +-0.9..3.5 grados).
# Opt-in: G1_DOOR_VIS=1.
DOOR_VIS = os.environ.get("G1_DOOR_VIS", "") == "1"
# --- GATE POR ILUMINACION del representante de vision (rama door-vis-illum-gate, 21-ago) ---
# Medido en sesion: con TODA la luz el canal DOOR-VIS produce 2-4x mas observaciones sesgadas
# +9 grados y el cruce acabo en +0.134 con golpe; a oscuras el canal calla y el eje del mapa
# centra (+-0.02). Decision de Renxi: en vez de corregir el sesgo, la ILUMINACION decide que
# representante del centro de puerta puede gobernar (W3 en produccion): con luma del frame por
# encima del umbral, el representante de vision queda SUPRIMIDO y gobierna el eje del mapa.
# La luma se estima en cliente (media del frame que ya se captura para percepcion, EMA 0.8).
# Emite por muestra illum_b (EMA) y dvis_gate (1 = suprimido) para puntuarlo en replay.
DOOR_VIS_GATE = os.environ.get("G1_DOOR_VIS_GATE", "") == "1"
DOOR_VIS_LUX = float(os.environ.get("G1_DOOR_VIS_LUX", "100"))   # medido: ~85 poca luz / ~116 toda
# RETREAT v2 (portado de main, donde esta VALIDADO en gemelo: 6/6 sin falsos + trampa
# determinista con 3 retiradas y P9 pausado). Doctrina de Renxi: "reverse back using the
# EXACT SAME TRAJECTORY as they enter". Disparo SOLO con atasco seguro (>=2 col/20s o
# col+atasco), edad minima 12s, migas >=0.5m, max 3/run; pausa el reloj HELP del P9.
RETREAT = os.environ.get("G1_RETREAT", "1") == "1"
RETREAT_D = float(os.environ.get("G1_RETREAT_D", "0.9"))
RETREAT_V = float(os.environ.get("G1_RETREAT_V", "0.22"))
# MAQUINA DE ESTADOS META (feedback Renxi: "state machine in the meta level" + "is my
# perception reliable" + "quality of the interface" + "human assistance if still stuck").
# Estados: NORMAL -> DEGRADED (laser_trust<0.6 o door_contra>=3 o iface_q<0.5: percepcion
# poco fiable -> techo 0.24) | BLIND (obstaculo conocido a <1.0m, la frontera del robot
# hipermetrope -> techo 0.28) | RECOVERY (retirada en curso) | ASSIST (presupuesto de
# retiradas agotado y aun atascado -> PARAR y esperar 'm <cm>' del humano; al recibirla,
# presupuestos re-armados y vuelta a NORMAL). Transiciones = evento meta_state{de,a};
# estado por muestra. G1_METASM=0 lo apaga (queda todo en NORMAL).
# INTEGRACION 17-ago: el DEFAULT pasa de "1" a "0". En la rama -sim venia encendido porque
# alli la maquina META era el objeto del trabajo; aqui convive con el nivel objeto CONGELADO
# (tag golden-doorcross), y si viniera encendida la rama fusionada dejaria de reproducir esa
# conducta -- rompiendo la regla del proyecto (los defaults reproducen lo anterior) y la premisa
# de la preregistracion (el nivel objeto es constante e identico en las cuatro condiciones).
# Sin riesgo: todas las campanas pasan G1_METASM explicitamente, nada dependia del implicito.
METASM = os.environ.get("G1_METASM", "0") == "1"
# RETREAT-EN-SALIDA (ataque al bolsillo de B, 31-jul): los encajes del bolsillo nacen EN el
# arranque (run 113617: 1a colision a t=1.1s, dmax 0.44m) y la guardia de span >=0.5m del
# RETREAT (correcta contra el desastre v1) le impedia armarse justo ahi -> molienda sin
# recuperacion. Diseno: si el encaje es a <1.5m del punto de ARRANQUE, el span minimo baja
# a 0.15m (retroceder hacia la pose de arranque es seguro: era valida hace un segundo).
# El techo de salida 0.22 se PROBO Y SE DESCARTO (empeoro el encaje: sin momento, el giro
# del engagement arrastra el hombro contra la pared). G1_EXIT_RETREAT=0 lo desactiva.
EXIT_RT = os.environ.get("G1_EXIT_RETREAT", "1") == "1"
# Cadencia de laser_snapshots (s). Para sesiones de CALIBRACION DE COVARIANZA: G1_LASER_SNAP=0.5
# (el fabricante no expone covarianza del lidar; la estimamos offline de estos snapshots, que
# desde 2026-07-21 llevan pose+fase de movimiento para separar parado/andando/girando).
LASER_SNAP = float(os.environ.get("G1_LASER_SNAP", "2.0"))
DOORSTICK_R = float(os.environ.get("G1_DOORSTICK_R", "1.4"))   # m alrededor del centro del vano
# FIX B: FRENO POR CAMARA — la mitad no implementada del principio de Renxi 2026-07-02 ("el
# LiDAR decide, la vision APOYA: los clamps solo MODERAN la velocidad"). El canal clamp del
# perception_server ("lo tengo encima", NEAR_CLAMP=0.7, solo columnas centrales con obstruccion
# alta) veia el sofa/mueble BAJO el plano del laser (colisiones 103218 t=22.6 y 174051:
# color_near 30+ constante hasta el impacto) y el cliente solo lo logueaba. Actua UNICAMENTE
# si el laser dice via libre (c0 > CB_C0): en la boca de la puerta el laser ya ve las jambas
# y mandan HARD-GUARD/DWA (evita el 'muro fantasma' de las runs 143511/143646). Nunca veta:
# modera el avance; giros/retrocesos intactos. G1_COLORBRAKE=0 revierte.
COLOR_BRAKE = os.environ.get("G1_COLORBRAKE", "1") == "1"
# Politica calibrada por REPLAY sobre las 60 runs reales (2026-07-21): (1) SUPRIMIDO a menos
# de CB_DOOR_R del vano — alli el marco llena la camara con el laser viendo via libre a traves
# del hueco (2.50) y pararse = el muro fantasma de las runs 143511/143646 (los runs limpios
# disparaban 4-10 veces, todos en la boca); (2) aviso sostenido (>=CB_NPTS) -> ARRASTRE lento
# CB_CREEP, no parada (pasar cerca del sofa es legitimo; 104244 paso limpio con n=20-22);
# (3) camara LLENA (>=CB_STOP, solo visto de verdad delante del mueble: n=47 en el roce del
# arranque 103218 t=2) -> parada. G1_COLORBRAKE=0 revierte todo.
CB_NPTS = int(os.environ.get("G1_CB_NPTS", "8"))       # columnas clamp para el ARRASTRE (sostenidas)
CB_STOP = int(os.environ.get("G1_CB_STOP", "24"))      # columnas clamp para PARADA total
CB_TICKS = int(os.environ.get("G1_CB_TICKS", "4"))     # ticks seguidos antes de actuar (~0.4-1s)
CB_C0 = float(os.environ.get("G1_CB_C0", "0.9"))       # solo si el laser ve >= esto de holgura
CB_CREEP = float(os.environ.get("G1_CB_CREEP", "0.10"))  # avance de arrastre con aviso (m/s)
CB_DOOR_R = float(os.environ.get("G1_CB_DOOR_R", "2.0"))  # m del vano donde el freno se SUPRIME
DOOR_AXIS = float(os.environ.get("G1_DOOR_AXIS", "135.0"))     # deg: direccion de cruce lado A -> lado B
DOOR_ENG_D = float(os.environ.get("G1_DOOR_ENG_D", "0.85"))    # m del centro al punto de engagement
DOOR_ENG_TOL = 0.22              # m: engagement alcanzado
DOOR_ALIGN_TOL = 8.0             # deg: alineado para cruzar (2 ticks seguidos)
DOOR_REALIGN = 14.0              # deg: deriva durante el cruce -> parar y re-alinear
DOOR_EXIT_D = 0.75               # m pasado el centro = cruzado (vuelve el control normal)
# --- Filtro de PERSISTENCIA (K-de-N barridos frescos) ---
# OJO (investigado 2026-07-02): el Mid-360 usa escaneo NO REPETITIVO (Livox) -> dos barridos consecutivos
# muestrean DIRECCIONES DISTINTAS del FOV. El parpadeo celda-a-celda entre barridos es INHERENTE al sensor,
# no solo ruido de la marcha; integrar barridos antes de creer una celda es la forma correcta de consumir
# un Livox. Contrapartida: un obstaculo REAL fino (pata de mesa/silla) tambien parpadea legitimamente.
# => NO subir K para "filtrar mas" (retrasa la entrada de obstaculos finos reales); si el ruido gana,
#    subir N manteniendo K=2 (ventana mas larga, mismo nº de votos). SAFE_R + vision son la red para
#    lo que el laser muestrea mal de cerca (Livox: deteccion NO garantizada a 0.1-1m en superficies
#    oscuras/pulidas/finas -> la mesa). Ajustables por entorno para A/B en el robot sin tocar codigo.
PERSIST_N = int(os.environ.get("G1_PERSIST_N", "3"))   # ventana de barridos FRESCOS recientes
PERSIST_K = int(os.environ.get("G1_PERSIST_K", "2"))   # una celda que SOLO ve el laser (no esta en el mapa) cuenta si aparece en >=K de los ultimos N
STALE_WARN_TICKS = 10            # ticks seguidos con la nube SIN refrescar antes de avisar en el log (~3s a 0.3s/tick)
FILM_PERIOD = float(os.environ.get("G1_FILM", "3.0"))   # s entre frames de la PELICULA de la run (0 = off).
                                 # ~30 jpgs de 5KB por run: la aproximacion a CUALQUIER incidente queda grabada.
# --- GUARDIA ANTI-DIVERGENCIA de relocalizacion (fix 2 del handoff; se perdio en el rollback) ---
# Run 134458: 78 reloc_jumps encadenados, path_m=582, pose final a 538m -> el robot CAMINABA en coordenadas
# de fantasia. Una correccion legitima de reloc es UN salto y luego estable; divergencia = saltos repetidos.
# El guardia SOLO PARA (STOP + aborta la run): nunca dirige. Falso positivo = run abortada, robot quieto = seguro.
RELOC_GUARD = (os.environ.get("G1_RELOCGUARD", "1") == "1")   # G1_RELOCGUARD=0 lo desactiva (A/B estricto)
RELOC_STOP_N = int(os.environ.get("G1_RELOC_N", "4"))         # nº de saltos >0.5m...
RELOC_STOP_WIN = float(os.environ.get("G1_RELOC_WIN", "10.0"))  # ...dentro de esta ventana (s) = divergencia
# --- OBSTACULOS DE ALTA CONFIANZA (Renxi): pared del mapa / score saturado / colision. ---
HARD_GUARD = (os.environ.get("G1_HARDGUARD", "1") == "1")   # ON por defecto (replay 29 runs: 0 activaciones
# en runs limpias -> coste cero; 4 STOPs pre-colision en 150440. Paredes NO negociables. =0 revierte).
HARD_STOP = float(os.environ.get("G1_HARD_STOP", "0.22"))    # m: no avanzar con pared a menos de esto
HARD_SLOW = float(os.environ.get("G1_HARD_SLOW", "0.45"))    # m: acercarse a pared -> velocidad reducida
# --- ESCAPE al arranque (HANDOFF 8.8): el waypoint B quedo grabado con la nariz a ~40cm del sofa -> cada
# vuelta B->A NACE encajonada: sin sitio para girar, choca en el propio punto de inicio (150440: golpes en
# t=4s y t=12.5s sin salir del punto de arranque; 143039: 39/54s atascado). Si al arrancar nace encajonado,
# retrocede ~ESC_DIST en RECTO antes de planificar. Actua UNA sola vez, solo en los primeros 5s de la run.
# DISPARO doble (medido en las 3 vueltas fallidas + 4 arranques buenos):
#   laser:  c0 < ESC_TRIG  — PERO el sofa a 40cm cae en la banda ciega del Mid-360: en 150440 c0=1.92
#           "libre" hasta el golpe (c0 0.22 recien EN t=4.4s) y en 143039 nunca bajo de 1.0. Solo, no basta.
#   vision: carpet_pct < ESC_CARPET — la camara SI lo ve: enterrada en el sofa lee 0.00-0.43 (el sofa crema
#           ademas CLASIFICA como moqueta e infla el valor, por eso 0.50 y no 0.12); arranques buenos leen
#           0.86-0.95. Es el caso meta-reasoning de Renxi: LiDAR dice "libre", la vision desempata.
ESCAPE_ON = (os.environ.get("G1_ESCAPE", "1") == "1")        # G1_ESCAPE=0 lo desactiva (A/B estricto)
ESC_TRIG = float(os.environ.get("G1_ESC_TRIG", "0.45"))      # m: c0 al arranque por debajo = nace encajonado
ESC_CARPET = float(os.environ.get("G1_ESC_CARPET", "0.50"))  # frac: menos suelo visible que esto al arrancar = encajonado
ESC_DIST = float(os.environ.get("G1_ESC_DIST", "0.50"))      # m: cuanto retrocede antes de planificar
# --- Mapa de obstaculos con SCORE/DECAY (anti acumulacion de ruido) ---
# Antes: celda confirmada -> entra al instante y dura 60s (TTL). Con el robot parado en la puerta y el LiDAR
# de cabeza vibrando, el mapa = UNION de todo lo visto en 60s -> cientos de celdas falsas (obs 142->521 en 18s).
# Ahora: cada celda tiene un SCORE que sube al verse y baja si esta EN RANGO y no se ve. Con SC_MISS>=SC_HIT una
# celda solo se mantiene si se ve en la MAYORIA de barridos recientes: el ruido intermitente decae y desaparece;
# las paredes/mesa reales (vistas casi siempre) saturan al tope y se quedan. NO añade paredes fantasma.
OLDMAP   = (os.environ.get("G1_OLDMAP") == "1")            # =1 vuelve al mapa TTL antiguo (para A/B en el robot)
SC_HIT   = float(os.environ.get("G1_SC_HIT",   "1.0"))    # +score al ver la celda
SC_MISS  = float(os.environ.get("G1_SC_MISS",  "1.0"))    # -score si esta EN RANGO y no se ve (>=HIT => exige mayoria de barridos)
SC_CAP   = float(os.environ.get("G1_SC_CAP",   "6.0"))    # tope de score (da inercia a paredes reales ante oclusiones breves)
SC_OBST  = float(os.environ.get("G1_SC_OBST",  "2.0"))    # umbral de score para contar como obstaculo
SC_RANGE = float(os.environ.get("G1_SC_RANGE", "2.6"))    # m: solo se penaliza (decay) dentro de este radio (ventana de plan); fuera se conserva por TTL
# GATE DE ROTACION: al girar rapido la nube 'location' se proyecta con la pose RETRASADA -> los barridos se
# ensucian (laser_noise +65% girando, observado). No es fiable meterlos en el mapa. Si |yaw_rate|>YAW_GATE
# CONGELAMOS el mapa (ni inserta ni decae): usamos lo capturado yendo recto/lento. 0 = desactiva. (G1_YAWGATE)
# --- RESOLUCION DE ROL (protocolo DCC, paso 2 del §8) ---
# Emite role / role_reason / authority por muestra: es Z_t, sin el cual A_meta = 1[Z_t = delta_t]
# no se puede computar. El modulo OBSERVA, no actua: no toca ninguna orden (§5.3, Authority
# Partitioning). Import defensivo a proposito -- que falte el fichero no puede tumbar una sesion.
try:
    from dcc_roles import resuelve_rol as _dcc_rol
    from dcc_roles import EstabilizadorRol as _DccEstab
except Exception as _e_rol:
    _dcc_rol = None; _DccEstab = None
    print("  [DCC] sin resolucion de rol (%s)" % _e_rol)

# --- MEMORIA DE VOXELS EN LA BANDA CIEGA (Renxi 14-ago) + BARRIDO POR RAYOS (24-ago) ---
# MEDIDO: 107 de 193 colisiones reales ocurrieron con el laser declarando libre >0.6 m, y 29
# estuvieron precedidas de ceguera medible (mediana 2.2 s, p90 4.1 s): el laser vio el
# obstaculo y lo perdio al acercarse. El costmap local borra en el mismo tick lo que deja de
# verse, asi que el planificador traza contra un obstaculo invisible.
# LA MITAD QUE FALTABA: la v1 solo caducaba por TTL, y cada barrido que reconfirma una celda
# le refresca el sello -> una celda que fue real UNA VEZ (una silla que se movio) no caduca
# nunca mientras el robot ronde cerca. Campana del gemelo: 5/6 neutral y 1/6 CATASTROFICO,
# 7 impactos en el mismo punto, 41 celdas de mediana y 67 de pico contra 17-24 sanas.
# Un TTL dice "ha pasado tiempo"; un rayo dice "he MIRADO y no hay nada". vox_rayos.despeja
# suelta en el acto toda celda que un rayo del barrido actual atraviesa terminando mas alla,
# y NUNCA dentro de NEAR_BLIND, que es justo la zona para la que la memoria existe.
VOXMEM = os.environ.get("G1_VOXMEM", "") == "1"                  # OFF por defecto
VOXMEM_RAYS = os.environ.get("G1_VOXMEM_RAYS", "1") == "1"       # la mitad negativa (ablacion: =0)
VOXMEM_TTL = float(os.environ.get("G1_VOXMEM_TTL", "3.0"))       # s (3 s cubre el 69% de las cegueras)
VOXMEM_R = float(os.environ.get("G1_VOXMEM_R", "1.2"))           # m: radio donde aplica la memoria
VOXMEM_K = int(os.environ.get("G1_VOXMEM_K", "2"))               # confirmaciones sanas para memorizar
VOXMEM_MAX = int(os.environ.get("G1_VOXMEM_MAX", "400"))         # tope de celdas recordadas
# DECISION PROVISIONAL (D1, tasks/DECISIONES_PENDIENTES_RENXI.md): por defecto la memoria
# se EXPONE (campo de I1: vox_inj/vox_ray por muestra) pero NO actua sobre el planificador.
# El §5.3 exige que la confianza del sensor no se convierta en autoridad de control
# automaticamente, y si inyecta incondicionalmente C1 y C3 reciben lidar historico en
# silencio y el factor de interfaz deja de ser separable del de proceso. La rama que ACTUA
# (el beneficio de seguridad, validado en la campana de 45 runs) queda a un interruptor.
VOXMEM_ACT = os.environ.get("G1_VOXMEM_ACT", "") == "1"          # 1 = ademas inyecta
try:
    from vox_rayos import despeja as _vox_despeja
except Exception as _e_vox:
    _vox_despeja = None

YAW_GATE = float(os.environ.get("G1_YAWGATE", "30.0"))    # >0 = gate ACTIVO (G1_YAWGATE=0 lo desactiva). yaw_rate solo se loguea (yr=)
# El gate mira el giro COMANDADO (|rx|), NO el yaw medido: el bamboleo de la marcha mete 30-66 deg/s de yaw
# yendo RECTO (medido, rx=0) -> con umbral por yaw medido el gate congelaba caminando de frente (run 164456).
RX_GATE  = float(os.environ.get("G1_RXGATE",  "0.20"))    # |rx| comandado que cuenta como GIRO real (giro de puerta ~0.45)
# SEGURIDAD > anti-ruido: obstaculo de laser CONFIRMADO a < SAFE_R de frente entra YA al mapa (salta gate y umbral).
SAFE_R   = float(os.environ.get("G1_SAFE_R",  "1.20"))    # m: radio de override de seguridad de campo cercano
VIS_OBST_LABELS = {"table", "diningtable", "dining table", "desk", "chair", "couch", "sofa",
                   "bench", "refrigerator", "person"}     # clases YOLO que son obstaculo (para el log [VIS])
DATASET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dataset")


class RunRecorder:
    """Graba una travesia completa a un JSON estructurado en dataset/ (dataset-ready, mismo esquema para
    nuestra nav y el firmware): metadatos + trayectoria + eventos (colisiones) + snapshots del laser +
    metricas resumen. Apto para subir/comparar/entrenar (tesis: meta-cognicion firmware vs nuestro)."""

    def __init__(self, mode, label, goal, pcd=""):
        try:
            os.makedirs(DATASET_DIR, exist_ok=True)
        except Exception:
            pass
        self.t0 = time.time()
        self.mode = mode
        self.fname = os.path.join(DATASET_DIR, time.strftime("%Y%m%d_%H%M%S") + f"_{mode}_{label}.json")
        self.rec = {"schema": "g1_goto_run/v1", "mode": mode, "label": label,
                    "env": RUN_ENV,                      # 'real' | 'sim' (G1_ENV): separa robot fisico de simulacion
                    "sim_id": os.environ.get("G1_SIM_ID", "") or None,   # etiqueta libre del contenedor/escenario sim
                    "goal": {"x": goal[0], "y": goal[1]}, "pcd": pcd, "OCELL": g.OCELL,
                    "hband": [HBAND_LO, HBAND_HI], "started": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "samples": [], "events": [], "laser_snapshots": [], "telemetry": [], "summary": {}}
        # SNAPSHOT DE CONFIGURACION (auditoria 22-jul): la tabla del paper no puede depender
        # de memoria humana. Env G1_* + SHA de git + brazo declarado (G1_ARM=C2-GOV...).
        try:
            self.rec["env_g1"] = {k: v for k, v in sorted(os.environ.items()) if k.startswith("G1_")}
            self.rec["arm"] = os.environ.get("G1_ARM") or None
            import subprocess as _sp
            _here = os.path.dirname(os.path.abspath(__file__))
            self.rec["git"] = {
                "sha": _sp.run(["git", "rev-parse", "--short", "HEAD"], cwd=_here, timeout=2,
                               capture_output=True, text=True).stdout.strip() or None,
                "dirty": bool(_sp.run(["git", "status", "--porcelain", "-uno"], cwd=_here, timeout=2,
                                      capture_output=True, text=True).stdout.strip())}
        except Exception:
            pass
        # PAYLOAD legible por maquina (G1_FILL_G / G1_CUP_G): el endpoint primario del paper
        # son GRAMOS; hoy vivian solo en el chat.
        try:
            _fg = os.environ.get("G1_FILL_G"); _cg = os.environ.get("G1_CUP_G")
            if _fg or _cg:
                self.rec["payload"] = {"fill_g": float(_fg) if _fg else None,
                                       "cup_g": float(_cg) if _cg else None}
        except Exception:
            pass
        self._laser_t = 0.0
        self._telem_t = -9.0
        # --- GROUND TRUTH de derrames (condicion payload): un humano marca cada derrame.
        # Canal UDP (sin dependencias): cualquier datagrama al puerto = 1 derrame.
        #   robot real:  python spill_mark.py   (o: echo x | nc -u 127.0.0.1 7777)
        #   sim:         ros2 topic pub --once /spill_event std_msgs/msg/Empty
        #                (el adaptador reenvia /spill_event a este mismo puerto)
        # G1_SPILL_GT_PORT=0 lo desactiva.
        self._gt_spills = 0
        self._gt_hb_t = None       # ultimo latido del marcador (heartbeat spill_mark v2)
        self._gt_hb_seen = False
        self._gt_lost = False      # canal caido en este momento (para alarma/evento)
        self._gt_dropouts = 0
        _port = int(os.environ.get("G1_SPILL_GT_PORT", "7777") or 0)
        if _port:
            def _gt_listener():
                import socket
                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    s.bind(("0.0.0.0", _port))
                except Exception:
                    return
                unknown_logged = False
                while True:
                    try:
                        data, _addr = s.recvfrom(128)
                    except Exception:
                        return
                    msg = data.decode("utf-8", errors="replace").strip().lower()
                    t = time.time() - self.t0
                    # HEARTBEAT (auditoria 22-jul): distingue "cero derrames" de "marcador
                    # muerto" — el fallo del 21-jul es irreproducible sin esto.
                    if msg == "hb":
                        self._gt_hb_t = time.time(); self._gt_hb_seen = True
                        continue
                    last = self.rec["samples"][-1] if self.rec["samples"] else None
                    lx = last["x"] if last else 0.0; ly = last["y"] if last else 0.0
                    if msg.startswith("weigh:"):
                        try:
                            g_ = float(msg.split(":", 1)[1])
                            self.event("weigh", t, lx, ly, extra={"grams": g_, "src": "manual"})
                            print(f"  [spill-GT] PESO registrado: {g_:.0f} g t={t:.1f}s")
                        except Exception:
                            pass
                        continue
                    if msg.startswith("invalid"):
                        why = msg.split(":", 1)[1] if ":" in msg else ""
                        self.event("run_invalid", t, lx, ly, extra={"reason": why, "src": "manual"})
                        self.rec["summary"]["valid"] = False
                        self.rec["summary"]["invalid_reason"] = why
                        print(f"  [spill-GT] RUN MARCADA INVALIDA ({why or 'sin motivo'})")
                        continue
                    if msg == "hcol":
                        # HUMANO reporta colision que el robot NO detecto (Renxi: "human can
                        # tell the robot for feedback"). Alimenta la maquinaria de colision
                        # existente via flag consumido por el bucle de control.
                        self._h_col_t = time.time()
                        self.event("human_collision", t, lx, ly, extra={"src": "human"})
                        print(f"  [HUMANO] COLISION reportada por el operador t={t:.1f}s")
                        continue
                    if msg.startswith("hmove:"):
                        # HUMANO movio el robot a mano (Renxi: "move the robot 10 cm... it
                        # should remember I was asked to move 10 cm. Event recorded.")
                        try:
                            _cm = float(msg.split(":", 1)[1])
                        except Exception:
                            _cm = 0.0
                        self._h_assist_t = time.time()
                        self.event("human_assist", t, lx, ly, extra={"cm": _cm, "src": "human"})
                        try:
                            _amf = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                "assist_memory.json")
                            try:
                                _am = json.load(open(_amf))
                            except Exception:
                                _am = []
                            _am.append({"x": round(lx, 2), "y": round(ly, 2), "cm": _cm,
                                        "when": time.strftime("%Y-%m-%d %H:%M:%S"),
                                        "run": os.path.basename(self.fname)})
                            json.dump(_am[-200:], open(_amf, "w"))
                        except Exception:
                            pass
                        print(f"  [HUMANO] asistencia registrada: movido {_cm:g} cm en ({lx:+.2f},{ly:+.2f}) — MEMORIZADO")
                        continue
                    if not msg.startswith("spill"):
                        # anti-falsos (trampa cazada por el verificador: antes CUALQUIER
                        # datagrama contaba como derrame)
                        if not unknown_logged:
                            print(f"  [spill-GT] datagrama desconocido ignorado: {msg[:24]!r}")
                            unknown_logged = True
                        continue
                    self.event("spill_human", t, lx, ly, extra={"src": "manual"})
                    self._gt_spills += 1
                    print(f"  [spill-GT] DERRAME humano #{self._gt_spills} t={t:.1f}s")
            threading.Thread(target=_gt_listener, daemon=True).start()

    def sample(self, t, x, y, yaw, d, spd, c0, nobs, cmd=None, phase="", extra=None):
        # vigilancia del canal spill-GT (solo si el marcador v2 dio senal de vida alguna vez)
        if self._gt_hb_seen:
            _dhb = time.time() - (self._gt_hb_t or 0)
            if not self._gt_lost and _dhb > 6.0:
                self._gt_lost = True; self._gt_dropouts += 1
                self.event("gt_lost", t, x, y, extra={"since_s": round(_dhb, 1)})
                print(f"  [spill-GT] *** MARCADOR CAIDO (sin latido {_dhb:.0f}s) ***")
            elif self._gt_lost and _dhb < 3.0:
                self._gt_lost = False
                self.event("gt_back", t, x, y, None)
                print("  [spill-GT] marcador recuperado")
        rec = {"t": round(t, 2), "x": round(x, 3), "y": round(y, 3),
               "yaw": round(yaw, 1), "d": round(d, 3), "spd": round(spd, 3),
               "c0": round(c0, 2), "nobs": nobs, "phase": phase,
               "cmd": [round(float(v), 2) for v in cmd] if cmd else None}
        if extra:
            rec.update({k: v for k, v in extra.items() if v is not None})
        if _dcc_rol is not None:
            try:
                # Se emiten LOS DOS: `role` estabilizado (confirmacion + permanencia, para
                # que el tableteo no fabrique fronteras de decision falsas) y `role_crudo`
                # sin filtrar. Guardar el crudo mantiene el filtro auditable y permite medir
                # su coste a posteriori en vez de tener que fiarse de el.
                if _DccEstab is not None:
                    if getattr(self, "_estab", None) is None:
                        self._estab = _DccEstab()
                    _r, _why, _aut, _crudo = self._estab.paso(rec)
                    rec["role_crudo"] = _crudo
                else:
                    _r, _why, _aut = _dcc_rol(rec)
                rec["role"], rec["role_reason"], rec["authority"] = _r, _why, _aut
            except Exception:
                pass                       # jamas romper la grabacion por el resolutor
        self.rec["samples"].append(rec)

    def event(self, kind, t, x, y, extra=None):
        e = {"kind": kind, "t": round(t, 2), "x": round(x, 3), "y": round(y, 3)}
        if extra:
            e.update(extra)
        self.rec["events"].append(e)
        if kind in ("spill", "spill_human"):       # canal payload -> fragility del bridge META2
            self.last_spill_t = time.time()
            self.spill_count = getattr(self, "spill_count", 0) + 1

    def maybe_laser(self, t, pts, every=None, ctx=None):
        ev = LASER_SNAP if every is None else every
        if t - self._laser_t >= ev:
            snap = {"t": round(t, 2), "pts": [[round(a, 2), round(b, 2)] for a, b in pts]}
            if ctx:
                snap.update(ctx)               # pose + fase de movimiento (covarianza offline)
            self.rec["laser_snapshots"].append(snap)
            self._laser_t = t

    def telem(self, t, row, every=1.0):
        """Pista de telemetria completa (bateria/cpu/motores/IMU) a ~1Hz, separada de las muestras de nav."""
        if row and t - self._telem_t >= every:
            self.rec["telemetry"].append(dict(t=round(t, 2), **row))
            self._telem_t = t

    def save_cloud(self, tag, pose, points):
        """Guarda una nube 3D CRUDA (todas las alturas) a un fichero aparte y la referencia en el run.
        Util en colisiones: con la nube 3D se ve si era una mesa (tablero a media altura + hueco debajo)."""
        if not points:
            return None
        fn = self.fname[:-5] + f"_{tag}.json"
        try:
            json.dump({"tag": tag, "pose": pose, "npts": len(points) // 3, "points": points,
                       "OCELL": g.OCELL, "hband": [HBAND_LO, HBAND_HI],
                       "frame": "map (idx0=x, idx1=y, idx2=altura)"}, open(fn, "w"))
            self.rec.setdefault("clouds", []).append(os.path.basename(fn))
            return fn
        except Exception:
            return None

    def save_cam(self, tag, jpg):
        """Guarda la foto de la camara (prueba visual de lo que habia, p.ej. una mesa) a un .jpg aparte."""
        if not jpg or not jpg.startswith("data:image"):
            return None
        import base64
        fn = self.fname[:-5] + f"_{tag}.jpg"
        try:
            with open(fn, "wb") as f:
                f.write(base64.b64decode(jpg.split(",", 1)[1]))
            self.rec.setdefault("cams", []).append(os.path.basename(fn))
            return fn
        except Exception:
            return None

    def finish(self, result, summary):
        summary = dict(summary or {})
        if getattr(self, "_gt_spills", 0):
            summary["spills_human"] = self._gt_spills
        # estado del canal de verdad de campo (auditoria 22-jul)
        if getattr(self, "_gt_hb_seen", False):
            summary["gt_hb_seen"] = True
            summary["gt_alive_at_end"] = (time.time() - (self._gt_hb_t or 0)) < 6.0
            summary["gt_dropouts"] = self._gt_dropouts
        if self.rec["summary"].get("valid") is False:      # marcado invalido in-situ
            summary["valid"] = False
            summary["invalid_reason"] = self.rec["summary"].get("invalid_reason", "")
        self.rec["result"] = result
        self.rec["summary"] = summary
        self.rec["duration_s"] = round(time.time() - self.t0, 2)
        try:
            json.dump(self.rec, open(self.fname, "w"))
            print(f"  [dataset] travesia guardada -> {self.fname}")
        except Exception as e:
            print("  [dataset] no se pudo guardar:", repr(e))
        try:
            self._append_stats()                       # fila estadistica AUTOMATICA (Adrian: estudios stats)
        except Exception as e:
            print("  [stats] no se pudo escribir runs_stats.csv:", repr(e))
        return self.fname

    def _append_stats(self):
        """Al terminar CUALQUIER run (llegada/Ctrl+C/aborto/guardia) anade UNA fila a
        dataset/runs_stats.csv con todo lo relevante para estudios estadisticos. Autocontenido:
        no hay que acordarse de correr summarize_runs; el CSV crece run a run."""
        import csv as _csv
        s = self.rec.get("samples", []); ev = self.rec.get("events", []); sm = self.rec.get("summary", {}) or {}

        def _mean(k):
            v = [x.get(k) for x in s if isinstance(x.get(k), (int, float))]
            return round(sum(v) / len(v), 3) if v else ""

        def _minv(k):
            v = [x.get(k) for x in s if isinstance(x.get(k), (int, float))]
            return round(min(v), 3) if v else ""

        def _maxv(k):
            v = [x.get(k) for x in s if isinstance(x.get(k), (int, float))]
            return round(max(v), 3) if v else ""

        def _first(k):
            for x in s:
                if isinstance(x.get(k), (int, float)):
                    return x[k]
            return ""

        def _last(k):
            for x in reversed(s):
                if isinstance(x.get(k), (int, float)):
                    return x[k]
            return ""

        # episodios de ATASCO: >8 s sin acercarse >=0.10 m al objetivo (mismo criterio que AGGR_AFTER)
        stuck_n = 0; stuck_s = 0.0; best_d = None; last_imp = None; in_stuck = False; prev_t = None
        for x in s:
            t, dd = x.get("t", 0.0), x.get("d")
            if dd is None:
                continue
            if best_d is None or dd < best_d - 0.10:
                best_d = dd; last_imp = t; in_stuck = False
            elif last_imp is not None and (t - last_imp) > 8.0:
                if not in_stuck:
                    stuck_n += 1; in_stuck = True
            if in_stuck and prev_t is not None:
                stuck_s += max(0.0, t - prev_t)
            prev_t = t
        # tiempo por familia de fase (%)
        fam = {"DWA": 0, "DOOR": 0, "BRK": 0, "SEEK": 0, "R": 0, "STOP": 0, "OTRO": 0}
        for x in s:
            p = (x.get("phase") or "").replace("AGR-", "")
            k = ("DWA" if p.startswith("DWA") else "DOOR" if p.startswith("DOOR") else
                 "BRK" if p.startswith("BRK") else "SEEK" if p.startswith("SEEK") else
                 "R" if p.startswith("R-") or p.startswith("R") and "-" in p else
                 "STOP" if p.startswith("STOP") else "OTRO")
            fam[k] += 1
        nt = max(1, len(s))
        # detecciones YOLO por etiqueta (json compacto en una columna)
        dets = {}
        for x in s:
            for dd in (x.get("dets") or []):
                if dd and dd[0]:
                    dets[dd[0]] = dets.get(dd[0], 0) + 1
        # densidad de obstaculos por area recorrida (celdas medias / m2 del bbox de la trayectoria)
        xs = [x["x"] for x in s]; ys = [x["y"] for x in s]
        area = max(1.0, (max(xs) - min(xs) + 1.0) * (max(ys) - min(ys) + 1.0)) if s else 1.0
        row = {
            "run": os.path.basename(self.fname)[:-5],
            "started": self.rec.get("started", ""), "mode": self.mode,
            "goal": self.rec.get("label", ""), "result": self.rec.get("result", ""),
            "time_s": sm.get("time_s", self.rec.get("duration_s", "")),
            "path_m": sm.get("path_m", ""), "efficiency": sm.get("efficiency", ""),
            "collisions": sum(1 for e in ev if e.get("kind") == "collision"),
            "stuck_episodes": stuck_n, "stuck_time_s": round(stuck_s, 1),
            "astar_fails": sum(1 for e in ev if e.get("kind") == "astar_fail"),
            "aggressive_on": sum(1 for e in ev if e.get("kind") == "aggressive_on"),
            "reloc_jumps": sm.get("reloc_jumps", ""),
            "pct_dwa": round(100.0 * fam["DWA"] / nt, 1), "pct_door": round(100.0 * fam["DOOR"] / nt, 1),
            "pct_brk": round(100.0 * fam["BRK"] / nt, 1), "pct_seek": round(100.0 * fam["SEEK"] / nt, 1),
            "pct_recovery": round(100.0 * fam["R"] / nt, 1), "pct_stop": round(100.0 * fam["STOP"] / nt, 1),
            "spd_mean": _mean("spd"), "c0_mean": _mean("c0"), "c0_min": sm.get("c0min", _minv("c0")),
            "c0_hard_mean": _mean("c0_hard"), "c0_hard_min": _minv("c0_hard"),
            "near_wall_ticks": sum(1 for x in s if isinstance(x.get("c0_hard"), (int, float)) and x["c0_hard"] < 0.35),
            "obs_mean": _mean("nobs"), "obs_max": sm.get("obs_max", ""),
            "obs_per_m2": round((_mean("nobs") or 0) / area, 2) if s else "",
            "n_hard_mean": _mean("n_hard"), "colmap_cells": sm.get("colmap_cells", ""),
            "perc_n_mean": _mean("perc_n"), "color_pts_mean": _mean("color_pts"),
            "carpet_pct_mean": _mean("carpet_pct"), "color_near_mean": _mean("color_near"),
            "dets_by_label": json.dumps(dets, ensure_ascii=False) if dets else "",
            "laser_noise_mean": sm.get("laser_noise_mean", _mean("laser_noise")),
            "filt_rej_mean": sm.get("filt_rej_mean", ""), "scan_hz": sm.get("scan_hz", ""),
            "stale_pct": sm.get("stale_pct", ""), "gated_pct": sm.get("gated_pct", ""),
            "safer_inserts": sm.get("safer_inserts", ""),
            "map_adds": sm.get("map_adds", ""), "map_dels": sm.get("map_dels", ""),
            "tick_ms_p95": sm.get("tick_ms_p95", ""),
            "clearance_mean": _mean("clearance"), "clearance_min": _minv("clearance"), "clearance_max": _maxv("clearance"),
            "progression_mean": _mean("progression"), "progression_min": _minv("progression"),
            "progression_max": _maxv("progression"),
            # confianza de sensores (Renxi): fiabilidad global, confianza de localizacion, ruido laser
            "reliability_mean": _mean("reliability"), "reliability_min": _minv("reliability"),
            "reliability_max": _maxv("reliability"),
            "loc_conf_mean": _mean("loc_conf"), "loc_conf_min": _minv("loc_conf"), "loc_conf_max": _maxv("loc_conf"),
            "laser_noise_min": _minv("laser_noise"), "laser_noise_max": sm.get("laser_noise_max", _maxv("laser_noise")),
            # min/max de las lecturas principales (Renxi: ademas de la media)
            "spd_max": _maxv("spd"), "c0_max": _maxv("c0"), "c0_hard_max": _maxv("c0_hard"),
            "perc_n_min": _minv("perc_n"), "perc_n_max": _maxv("perc_n"),
            "color_pts_max": _maxv("color_pts"), "carpet_pct_min": _minv("carpet_pct"),
            "carpet_pct_max": _maxv("carpet_pct"),
            # bateria (Renxi): nivel inicial/final, consumo de la travesia, minimo
            "bat_start": _first("bat"), "bat_end": _last("bat"),
            "bat_used": (round(_first("bat") - _last("bat"), 1)
                         if isinstance(_first("bat"), (int, float)) and isinstance(_last("bat"), (int, float)) else ""),
            "bat_min": _minv("bat"),
        }
        path = os.path.join(DATASET_DIR, "runs_stats.csv")
        new_file = not os.path.exists(path)
        with open(path, "a", newline="") as fo:
            w = _csv.DictWriter(fo, fieldnames=list(row.keys()))
            if new_file:
                w.writeheader()
            w.writerow(row)
        print(f"  [stats] fila anadida -> {path}")

# Hook independiente: captura la pose de RELOCALIZACION (mapa cargado) de rt/slam_info (currentPose sobre el
# .pcd) y de slam_relocation/odom. NO depende del hook de mapeo (slam_mapping/odom).
RELOC_JS = r"""(function(){
  if(!window.__relocHook){ window.__relocHook=1;
    var jp=JSON.parse;
    JSON.parse=function(s){ var v=jp.apply(this,arguments);
      try{ if(v && v.topic){ var tp=''+v.topic;
        if(tp.indexOf('slam_info')>=0){
          var d=(typeof v.data==='string')?jp(v.data):v.data;
          if(d && d.data && d.data.currentPose){ var p=d.data.currentPose;
            window.__pose=[p.x,p.y,p.z,p.q_x,p.q_y,p.q_z,p.q_w]; window.__pose_t=Date.now();
            if(d.data.pcdName) window.__pcd=d.data.pcdName;
          }
        }
        if(tp.indexOf('slam_relocation/odom')>=0 && v.data && v.data.pose && v.data.pose.pose){
          var pp=v.data.pose.pose;
          window.__relocodom=[pp.position.x,pp.position.y,pp.position.z,
                              pp.orientation.x,pp.orientation.y,pp.orientation.z,pp.orientation.w];
          window.__relocodom_t=Date.now();
          if(v.data.pose.covariance) window.__reloccov=v.data.pose.covariance;   // 6x6 (xx,yy,..); el G1 la manda a 0
        }
      }}catch(e){}
      return v;
    };
  } return 1;
})()"""


# Diagnostico para ENCONTRAR la nube en vivo en modo operacion/relocalizacion (los "puntitos blancos").
# Hookea: (1) TODOS los mensajes que los Workers mandan a la app (tipo + campos + si traen array de puntos),
# (2) la estructura de los topics slam_relocation/points y mapping/points.
CLOUD_DEBUG_JS = r"""(function(){
  if(!window.__cloudDbg){ window.__cloudDbg=1; window.__msgtypes={}; window.__cloudsample=null;
    var seen=new WeakSet();
    var o=Worker.prototype.postMessage;
    Worker.prototype.postMessage=function(m){
      try{ if(!seen.has(this)){ seen.add(this);
        this.addEventListener('message',function(ev){ try{
          var d=ev.data;
          var t=(d&&d.type)?(''+d.type):(d&&d.constructor?d.constructor.name:typeof d);
          var rec=window.__msgtypes[t]||{n:0,keys:'',dkeys:'',pts:0};
          rec.n++;
          if(!rec.keys && d && typeof d==='object'){
            rec.keys=Object.keys(d).slice(0,8).join(',');
            var dd=d.data;
            if(dd && typeof dd==='object'){ rec.dkeys=Object.keys(dd).slice(0,8).join(',');
              var arr=dd.directOutput||dd.points||dd.cloud||dd.data;
              if(arr){ var n=arr.length||(arr.byteLength?arr.byteLength/4:Object.keys(arr).length); rec.pts=n;
                if(!window.__cloudsample && n>30){ window.__cloudsample={type:t, n:n,
                  head:Array.prototype.slice.call(arr,0,9)}; } }
            }
          }
          window.__msgtypes[t]=rec;
        }catch(e){} });
      } }catch(e){}
      return o.apply(this,arguments);
    };
    // tambien: estructura de los topics de puntos por JSON.parse
    var jp=JSON.parse;
    JSON.parse=function(s){ var v=jp.apply(this,arguments);
      try{ if(v && v.topic){ var tp=''+v.topic;
        if(tp.indexOf('points')>=0){ var rec=window.__msgtypes['JSON:'+tp]||{n:0,keys:'',dkeys:'',pts:0};
          rec.n++; if(!rec.keys){ rec.keys=Object.keys(v).join(','); if(v.data) rec.dkeys=(typeof v.data)+':'+(v.data.length||Object.keys(v.data).slice(0,6).join('|')); }
          window.__msgtypes['JSON:'+tp]=rec; }
      }}catch(e){}
      return v;
    };
  } return 1;
})()"""


def cmd_clouddebug():
    """Encuentra la NUBE en vivo en modo operacion (los puntitos blancos). Lanza con el mapa cargado y
    relocalizado, deja que se vean los puntos, y muestra que tipos de mensaje los llevan."""
    cdp = g.get_cdp()
    cdp.eval(CLOUD_DEBUG_JS)
    dbglog = os.path.join(os.path.dirname(os.path.abspath(__file__)), "clouddebug.log")
    print(">>> CLOUD-DEBUG. Con el mapa cargado y los puntitos visibles, mueve el robot un poco.")
    print(f"    Voy guardando el resumen en {dbglog}. Ctrl+C para el volcado final.\n")

    def dump(final=False):
        try:
            mt = json.loads(cdp.eval("JSON.stringify(window.__msgtypes||{})") or "{}")
            samp = cdp.eval("JSON.stringify(window.__cloudsample||null)")
            samp = json.loads(samp) if samp and samp != "null" else None
        except Exception:
            mt = {}; samp = None
        lines = [f"=== CLOUDDEBUG {time.strftime('%H:%M:%S')}{' FINAL' if final else ''} "
                 "(n=cuantos, pts=nº puntos) ==="]
        for t, r in sorted(mt.items(), key=lambda kv: -kv[1].get("n", 0)):
            lines.append(f"  n={r.get('n', 0):<6} pts={r.get('pts', 0):<8} tipo='{t}' "
                         f"campos=[{r.get('keys', '')}] data=[{r.get('dkeys', '')}]")
        if samp:
            lines.append(f"  >>> NUBE: type='{samp.get('type')}' n={samp.get('n')} primeros9={samp.get('head')}")
        else:
            lines.append("  (aun sin muestra de nube con pts>30)")
        txt = "\n".join(lines)
        try:
            with open(dbglog, "w") as f:
                f.write(txt + "\n")
        except Exception:
            pass
        return txt

    try:
        while True:
            txt = dump()
            print("\033[2J\033[H", end=""); print(txt)
            time.sleep(1.0)
    except KeyboardInterrupt:
        dump(final=True)
        print(f"\nFin clouddebug. Guardado en {dbglog}. Di 'mira el clouddebug' y lo leo.")


# Captura la NUBE EN VIVO de operacion/relocalizacion: mensaje worker type='location', data.points =
# array plano [x,y,z,...]. Guarda la ultima en window.__relocbuf.
# ROBUSTO: adjunta el listener (1) a cada worker al que la app hace postMessage Y (2) a cada worker NUEVO
# via el constructor de Worker -> ya no depende del timing (antes a veces salia nobs=0).
RELOC_CLOUD_JS = r"""(function(){
  function grab(ev){ try{
    var d=ev.data;
    if(d && d.type==='location' && d.data && d.data.points){
      var a=d.data.points;
      window.__relocbuf=(ArrayBuffer.isView(a))?Array.prototype.slice.call(a):Object.values(a);
      window.__relocbuf_t=Date.now();
    }
  }catch(e){} }
  function attach(w){ try{ if(w && !w.__rcAtt){ w.__rcAtt=1; w.addEventListener('message',grab); } }catch(e){} }
  if(!window.__relocCloudHook){ window.__relocCloudHook=1;
    if(!window.__relocbuf) window.__relocbuf=[]; window.__relocbuf_t=window.__relocbuf_t||0;
    // 1) cada worker al que la app postea
    var o=Worker.prototype.postMessage;
    Worker.prototype.postMessage=function(m){ attach(this); return o.apply(this,arguments); };
    // 2) cada worker al que la app ESCUCHA mensajes (engancha aunque ya exista, en cuanto re-escucha)
    var oae=Worker.prototype.addEventListener;
    Worker.prototype.addEventListener=function(type,fn,opt){ try{ if(type==='message') attach(this); }catch(e){} return oae.apply(this,arguments); };
    // 3) cada worker NUEVO (constructor) -> coge el que emite 'location' al arrancar nav
    try{ var OW=window.Worker;
      function W(a,b){ var w=new OW(a,b); attach(w); return w; }
      W.prototype=OW.prototype; window.Worker=W;
    }catch(e){}
  }
  return (window.__relocbuf||[]).length;
})()"""


# Busca el PATH PLANIFICADO del firmware: caza mensajes (worker o JSON-topic) que contengan un array con
# pinta de ruta (>=3 elementos {x,y} o [x,y]). Si aparece, lo tenemos.
PATHSNIFF_JS = r"""(function(){
  if(!window.__pathSniff){ window.__pathSniff=1; window.__pathmsgs={}; window.__pathcand=null;
    function looksPath(o){ try{
        if(Array.isArray(o) && o.length>=3){ var a=o[0];
          if(a && typeof a==='object' && ('x' in a) && ('y' in a)) return o.length;
          if(Array.isArray(a) && a.length>=2 && typeof a[0]==='number') return o.length;
        }}catch(e){} return 0; }
    function scan(d,dep){ if(dep>4||!d||typeof d!=='object') return 0;
      var n=looksPath(d); if(n){ if(!window.__pathcand) window.__pathcand={where:'root',n:n,sample:JSON.stringify(d).slice(0,400)}; return n; }
      for(var k in d){ try{ var m=looksPath(d[k]);
        if(m){ if(!window.__pathcand) window.__pathcand={where:k,n:m,sample:JSON.stringify(d[k]).slice(0,400)}; return m; }
        var mm=scan(d[k],dep+1); if(mm) return mm; }catch(e){} } return 0; }
    var seen=new WeakSet(); var o=Worker.prototype.postMessage;
    Worker.prototype.postMessage=function(m){ try{ if(!seen.has(this)){ seen.add(this);
      this.addEventListener('message',function(ev){ try{ var d=ev.data; var t=(d&&d.type)?(''+d.type):typeof d;
        var rec=window.__pathmsgs[t]||{n:0,pathlen:0,keys:''}; rec.n++;
        if(!rec.keys && d&&typeof d==='object') rec.keys=Object.keys(d).slice(0,8).join(',');
        var pl=scan(d,0); if(pl>rec.pathlen) rec.pathlen=pl; window.__pathmsgs[t]=rec;
      }catch(e){} }); } }catch(e){} return o.apply(this,arguments); };
    var jp=JSON.parse; JSON.parse=function(s){ var v=jp.apply(this,arguments);
      try{ if(v&&v.topic){ var tp=''+v.topic;
        if(/path|plan|traj|route|nav|key_info|waypoint|topo/i.test(tp)){
          var d=(typeof v.data==='string')?jp(v.data):v.data;
          var rec=window.__pathmsgs['JSON:'+tp]||{n:0,pathlen:0,keys:''}; rec.n++;
          if(!rec.keys && d && typeof d==='object') rec.keys=Object.keys(d).slice(0,8).join(',');
          var pl=scan(d,0); if(pl>rec.pathlen) rec.pathlen=pl; window.__pathmsgs['JSON:'+tp]=rec;
        }
      }}catch(e){} return v; };
  } return 1;
})()"""


# Sniffer PASIVO: TU lanzas la navegacion desde la app y capturamos TODO lo util:
#  - OUT: lo que la app ENVIA por el datachannel (su comando de nav real + lo que sea)
#  - WORKER IN: mensajes worker->app (laser y posible render de ruta)
#  - TOPICS: todos, marcando los que parecen ruta + candidato de path
APPSNIFF_JS = r"""(function(){
  if(!window.__appSniff){ window.__appSniff=1;
    window.__snOut={}; window.__snWk={}; window.__snTop={}; window.__snSamples=[]; window.__snPath=null;
    window.__snAll={parse:0,worker:0,send:0};
    function push(a,o){ a.push(o); if(a.length>300) a.shift(); }
    function looksPath(o){ try{ if(Array.isArray(o)&&o.length>=3){ var a=o[0];
      if(a&&typeof a==='object'&&('x'in a)&&('y'in a)) return o.length;
      if(Array.isArray(a)&&a.length>=2&&typeof a[0]==='number') return o.length; }}catch(e){} return 0; }
    function scan(d,dep){ if(dep>5||!d||typeof d!=='object') return 0; var n=looksPath(d);
      if(n){ if(!window.__snPath) window.__snPath={where:'root',n:n,sample:JSON.stringify(d).slice(0,600)}; return n; }
      for(var k in d){ try{ var m=looksPath(d[k]);
        if(m){ if(!window.__snPath) window.__snPath={where:k,n:m,sample:JSON.stringify(d[k]).slice(0,600)}; return m; }
        var mm=scan(d[k],dep+1); if(mm) return mm; }catch(e){} } return 0; }
    var S=RTCDataChannel.prototype.send;
    RTCDataChannel.prototype.send=function(d){ try{ window.__snAll.send++; if((this.label||'')==='data' && typeof d==='string'){
      var v=JSON.parse(d); var tp=(v&&v.topic)?(''+v.topic):'?'; var api='';
      try{ api=(v.data&&v.data.header&&v.data.header.identity)?v.data.header.identity.api_id:''; }catch(e){}
      var key=tp+(api!==''?(' api='+api):''); var rec=window.__snOut[key]||{n:0,sample:''}; rec.n++;
      if(!rec.sample) rec.sample=d.slice(0,600); window.__snOut[key]=rec;
      if(tp.indexOf('wirelesscontroller')<0) push(window.__snSamples,{dir:'OUT',topic:tp,api:''+api,t:Date.now(),raw:d.slice(0,900)});
    }}catch(e){} return S.apply(this,arguments); };
    var seen=new WeakSet(); var o=Worker.prototype.postMessage;
    Worker.prototype.postMessage=function(m){ try{ if(!seen.has(this)){ seen.add(this);
      this.addEventListener('message',function(ev){ try{ window.__snAll.worker++; var d=ev.data; var t=(d&&d.type)?(''+d.type):typeof d;
        var rec=window.__snWk[t]||{n:0,pathlen:0,keys:''}; rec.n++;
        if(!rec.keys&&d&&typeof d==='object') rec.keys=Object.keys(d).slice(0,8).join(',');
        var pl=scan(d,0); if(pl>rec.pathlen) rec.pathlen=pl; window.__snWk[t]=rec;
      }catch(e){} }); } }catch(e){} return o.apply(this,arguments); };
    var jp=JSON.parse; JSON.parse=function(s){ var v=jp.apply(this,arguments);
      try{ if(v&&v.topic){ window.__snAll.parse++; var tp=''+v.topic; var rec=window.__snTop[tp]||{n:0,pathlen:0}; rec.n++;
        if(/path|plan|traj|route|nav|key_info|topo|waypoint/i.test(tp)){ var d=(typeof v.data==='string')?jp(v.data):v.data; var pl=scan(d,0); if(pl>rec.pathlen) rec.pathlen=pl; }
        window.__snTop[tp]=rec; } }catch(e){} return v; };
  } return 1;
})()"""


def _all_inspectable_pages():
    """Lista (titulo, url, ws_url) de TODAS las paginas inspeccionables del WebView (no solo la mejor)."""
    import requests
    devs = requests.get(g.PROXY + "/json", timeout=5).json()
    if not devs:
        return []
    durl = devs[0]["url"]
    pages = requests.get(f"http://{durl}/json", timeout=5).json()
    return [(p.get("title", "?"), p.get("url", "?"), p.get("webSocketDebuggerUrl"))
            for p in pages if p.get("webSocketDebuggerUrl")]


def cmd_appsniff(secs=40):
    """PASIVO multi-pagina: se engancha a TODAS las paginas del WebView. TU navegas DESDE LA APP (ir a B)
    y captura TODO en la que tenga trafico (la activa). No envio nada yo."""
    pages = _all_inspectable_pages()
    if not pages:
        print("No hay paginas inspeccionables. ¿ios_webkit_debug_proxy + app en SLAM?"); return
    cdps = []
    for (ti, url, wsu) in pages:
        try:
            c = g.CDP(wsu)
            c.eval(DISABLE_DRV_JS); c.eval(APPSNIFF_JS); c.eval(RELOC_JS)
            cdps.append((ti, url, c))
            print(f"  enganchado: '{ti[:22]}'  {url[:46]}")
        except Exception as e:
            print(f"  (no enganche '{(ti or '')[:22]}': {repr(e)[:50]})")
    if not cdps:
        print("No pude engancharme a ninguna pagina."); return
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "appsniff.log")
    outj = os.path.join(os.path.dirname(os.path.abspath(__file__)), "appsniff.json")
    print(f"\n>>> APPSNIFF en {len(cdps)} paginas. EN LA APP: pon destino B y navega. Capturo ~{secs}s. Ctrl+C para parar.\n")

    def traffic(c):
        try:
            a = json.loads(c.eval("JSON.stringify(window.__snAll||{})") or "{}")
            return sum(a.values()), a
        except Exception:
            return 0, {}

    def best_cdp():
        bc = None; bn = -1; rows = []
        for (ti, url, c) in cdps:
            tot, a = traffic(c); rows.append((ti, tot, a))
            if tot > bn:
                bn = tot; bc = c
        return bc, bn, rows
    try:
        for i in range(int(secs * 2)):
            bc, bn, rows = best_cdp()
            L = [f"=== APPSNIFF {time.strftime('%H:%M:%S')}  ({len(cdps)} paginas) ==="]
            for ti, tot, a in rows:
                L.append(f"  pagina '{ti[:24]:<24}' trafico={tot:<6} (parse={a.get('parse',0)} worker={a.get('worker',0)} send={a.get('send',0)})")
            if bc and bn > 0:
                so = json.loads(bc.eval("JSON.stringify(window.__snOut||{})") or "{}")
                sw = json.loads(bc.eval("JSON.stringify(window.__snWk||{})") or "{}")
                cand = bc.eval("JSON.stringify(window.__snPath||null)")
                L.append("  -- OUT (la APP envia) --")
                for k, r in sorted(so.items(), key=lambda kv: -kv[1]["n"]):
                    L.append(f"     n={r['n']:<4} {k}")
                L.append("  -- WORKER (PATHLEN=puntos tipo ruta) --")
                for k, r in sorted(sw.items(), key=lambda kv: -kv[1].get("pathlen", 0)):
                    L.append(f"     n={r['n']:<4} PATHLEN={r.get('pathlen',0):<4} tipo='{k}' [{r.get('keys','')}]")
                L.append(f"  >>> CANDIDATO DE RUTA: {cand if cand and cand!='null' else '(ninguno aun)'}")
            else:
                L.append("  (sin trafico aun en NINGUNA pagina; navega desde la app)")
            with open(out, "w") as f:
                f.write("\n".join(L) + "\n")
            print("\033[2J\033[H", end=""); print("\n".join(L))
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            bc, bn, _ = best_cdp()
            if bc:
                json.dump({"out": json.loads(bc.eval("JSON.stringify(window.__snOut||{})") or "{}"),
                           "worker": json.loads(bc.eval("JSON.stringify(window.__snWk||{})") or "{}"),
                           "path_candidate": json.loads(bc.eval("JSON.stringify(window.__snPath||null)") or "null"),
                           "samples_out": json.loads(bc.eval("JSON.stringify(window.__snSamples||[])") or "[]")},
                          open(outj, "w"), indent=1)
        except Exception:
            pass
        print(f"\nFin appsniff. Guardado {out} + {outj}. Di 'mira el appsniff' y lo analizo.")


def cmd_pathsniff(label):
    """DESCUBRE si el firmware expone su PATH planificado. Envia el goal nativo a <label> y caza durante
    ~20s cualquier mensaje con pinta de ruta. El robot SE MUEVE (firmware). Mando en mano."""
    wps = _load_wps()
    if not label or label not in wps:
        print(f"uso: python g1_goto.py pathsniff <N>  (waypoints: {list(wps.keys())})"); return
    w = wps[label]
    cdp = g.get_cdp()
    cdp.eval(DISABLE_DRV_JS); cdp.eval(NATIVE_CAP_JS); cdp.eval(RELOC_JS); cdp.eval(PATHSNIFF_JS)
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pathsniff.log")
    # espera dc
    for _ in range(30):
        if cdp.eval("!!window.__dc"):
            break
        time.sleep(0.3)
    cdp.eval(native_avoid_js(True)); r = cdp.eval(native_goal_js(w["x"], w["y"]))
    print(f">>> PATHSNIFF -> {label} ({w['x']:+.2f},{w['y']:+.2f}). Goal enviado: {r}. El robot se movera.")
    print(f"    Cazando mensajes con pinta de ruta ~20s. Ctrl+C para parar. Log -> {out}")
    try:
        for i in range(40):
            mm = json.loads(cdp.eval("JSON.stringify(window.__pathmsgs||{})") or "{}")
            cand = cdp.eval("JSON.stringify(window.__pathcand||null)")
            lines = [f"=== PATHSNIFF {time.strftime('%H:%M:%S')} (pathlen = nº de puntos tipo ruta) ==="]
            for t, rec in sorted(mm.items(), key=lambda kv: -kv[1].get("pathlen", 0)):
                lines.append(f"  n={rec.get('n',0):<5} PATHLEN={rec.get('pathlen',0):<5} tipo='{t}' campos=[{rec.get('keys','')}]")
            if cand and cand != "null":
                lines.append(f"  >>> CANDIDATO DE RUTA: {cand}")
            else:
                lines.append("  (todavia sin candidato de ruta)")
            with open(out, "w") as f:
                f.write("\n".join(lines) + "\n")
            print("\033[2J\033[H", end=""); print("\n".join(lines))
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        cdp.eval(native_cancel_js())
        print(f"\nFin pathsniff. Guardado en {out}. Di 'mira el pathsniff' y lo analizo.")


def cmd_cloudgrab():
    """Captura una nube 'location' en vivo + la pose, y la guarda en reloc_cloud.json para analizar el frame."""
    cdp = g.get_cdp()
    cdp.eval(RELOC_CLOUD_JS)
    cdp.eval(RELOC_JS)
    out = os.path.join(_DATA, "reloc_cloud.json")
    print(">>> CLOUDGRAB. Con el mapa cargado y los puntos visibles, espero a capturar una nube...")
    try:
        for _ in range(40):
            src, p, pcd = read_pose(cdp)
            n = int(cdp.eval("(window.__relocbuf||[]).length") or 0)
            print(f"  pose={'si' if p else 'no'}  puntos_nube={n}", end="\r")
            if p and n > 100:
                buf = json.loads(cdp.eval("JSON.stringify(window.__relocbuf||[])") or "[]")
                json.dump({"pose_src": src, "pose": p, "pcd": pcd, "npts": n, "points": buf[:9000]},
                          open(out, "w"))
                print(f"\n  GUARDADO: {n} valores ({n // 3} puntos), pose=({p[0]:+.2f},{p[1]:+.2f}) -> {out}")
                print("  Di 'mira el reloc_cloud' y analizo el frame para montar la rejilla 2D.")
                return
            time.sleep(0.5)
        print("\n  No capture nube. ¿Se ven los puntos en la app? ¿mapa cargado?")
    except KeyboardInterrupt:
        print("\nCancelado.")


def _install(cdp):
    cdp.eval(g.INSTALL_JS)          # captura mapeo (odom+nube+driver teleop) + grid hook
    cdp.eval(RELOC_JS)              # + pose de relocalizacion
    cdp.eval(RELOC_CLOUD_JS)        # + nube en vivo de operacion (mensaje 'location')
    cdp.eval(HEALTH_JS)             # + errorCode reloc + telemetria (bateria, cpu, motores)
    cdp.eval(IMUFULL_JS)            # + IMU completa (accel/gyro/rpy/par de patas)


def get_live_cdp():
    """Conecta a TODAS las paginas del WebView y devuelve el CDP de la que tiene el SLAM VIVO (pose +
    nube 'location'). Resuelve el problema de que get_cdp() coja una pagina muerta cuando hay varias
    (era la causa de nobs=0). Si solo hay una pagina, la devuelve directamente."""
    try:
        pages = _all_inspectable_pages()
    except Exception:
        pages = []
    if len(pages) <= 1:
        return g.get_cdp()
    cands = []
    for (ti, url, wsu) in pages:
        try:
            c = g.CDP(wsu)
            c.eval(RELOC_JS); c.eval(RELOC_CLOUD_JS)      # sondas ligeras (pose + nube)
            cands.append((ti, c))
        except Exception:
            pass
    if not cands:
        return g.get_cdp()
    if len(cands) == 1:
        return cands[0][1]
    best = None; bestsc = -1
    end = time.time() + 4.0
    while time.time() < end and bestsc < 3:
        for ti, c in cands:
            try:
                sc = (2 if c.eval("!!window.__pose") else 0) + \
                     (1 if int(c.eval("(window.__relocbuf||[]).length") or 0) > 50 else 0)
            except Exception:
                sc = -1
            if sc > bestsc:
                bestsc = sc; best = c
        time.sleep(0.4)
    print(f"  pagina viva elegida (score {bestsc}/3: pose+nube)" + ("" if bestsc >= 2 else "  <-- OJO: poca señal"))
    return best or cands[0][1]


def read_pose(cdp):
    """Pose LOCALIZADA sobre el mapa cargado. Prioriza slam_info.currentPose (autoritativa en relocalizacion),
    luego slam_relocation/odom, luego slam_mapping/odom. Devuelve (src, [x,y,z,qx,qy,qz,qw]) o (None,None)."""
    try:
        s = cdp.eval("JSON.stringify({pose:window.__pose||null, reloc:window.__relocodom||null, "
                     "map:window.__odom||null, pcd:window.__pcd||'', pt:window.__pose_t||0, rt:window.__relocodom_t||0})")
        d = json.loads(s) if s else {}
    except Exception:
        return (None, None, "")
    pcd = d.get("pcd", "")
    if d.get("pose"):
        return ("slam_info", d["pose"], pcd)
    if d.get("reloc"):
        return ("reloc_odom", d["reloc"], pcd)
    if d.get("map"):
        return ("map_odom", d["map"], pcd)
    return (None, None, pcd)


def yaw_of(q):
    """yaw (deg) desde quaternion [.. qx,qy,qz,qw] (indices 3..6)."""
    qx, qy, qz, qw = q[3], q[4], q[5], q[6]
    return math.degrees(math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))


def cmd_reloccheck():
    """PASO 1: con el mapa cargado y el robot relocalizado en la app, muestra que datos llegan."""
    cdp = get_live_cdp()
    _install(cdp)
    print(">>> RELOCCHECK. En la app: carga el mapa y RELOCALIZA el robot. Mueve el robot a mano y mira")
    print("    que la pose CAMBIA (= localizacion viva). Ctrl+C para salir.\n")
    try:
        while True:
            src, p, pcd = read_pose(cdp)
            try:
                extra = json.loads(cdp.eval(
                    "JSON.stringify({buf:(window.__buf||[]).length, grid:Object.keys(window.__grid||{}).length, "
                    "dc:!!window.__dc})") or "{}")
            except Exception:
                extra = {}
            if p:
                print(f"  POSE[{src}] x={p[0]:+.2f} y={p[1]:+.2f} yaw={yaw_of(p):+6.1f}  | "
                      f"nube={extra.get('buf', 0)} pts  grid={extra.get('grid', 0)} celdas  "
                      f"camara={'si' if extra.get('dc') else 'no'}  mapa='{pcd}'")
            else:
                print("  (sin pose localizada todavia; ¿mapa cargado y relocalizado en la app?)")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nFin reloccheck.")


def reloc_cells(cdp, pose=None):
    """Celdas OBSTACULO (OCELL, frame del mapa) desde la nube en vivo 'location' (window.__relocbuf),
    filtrando la banda de altura de torso. La nube ya esta en el frame de la pose -> sin conversiones.
    Si se pasa 'pose' (x,y), descarta el campo cercano (<NEAR_BLIND): el anillo fantasma del cabeceo."""
    try:
        s = cdp.eval("JSON.stringify(window.__relocbuf||[])")
        buf = json.loads(s) if s else []
    except Exception:
        reloc_cells.fresh = False
        return set()
    # --- FRESCURA del barrido: si el buffer no cambio desde la ultima llamada, es el MISMO barrido ---
    # (el topic 'location' puede refrescar mas lento que el tick de control ~0.3s -> sin esto, un unico
    # barrido ruidoso se cuenta 2-3 veces y pasa EL SOLO el filtro de persistencia 2-de-3). hash(str)=O(us).
    # Nube vacia ("[]") repetida tambien cuenta como NO fresca (= sin datos nuevos).
    sig = hash(s)
    reloc_cells.fresh = (sig != reloc_cells.last_sig)
    reloc_cells.last_sig = sig
    px = py = None
    if pose is not None:
        px, py = pose[0], pose[1]
    cells = set()
    for i in range(0, len(buf) - 2, 3):
        z = buf[i + 2]
        if z < HBAND_LO or z > HBAND_HI:
            continue
        cx, cy = buf[i], buf[i + 1]
        if px is not None and math.hypot(cx - px, cy - py) < g.NEAR_BLIND:
            continue
        cells.add((round(cx / g.OCELL), round(cy / g.OCELL)))
    return cells


reloc_cells.last_sig = None                  # firma del ultimo buffer visto (dedup de barrido)
reloc_cells.fresh = False                    # True si la ULTIMA llamada trajo un barrido NUEVO


# Salud/telemetria del robot (robot_data) + estado de relocalizacion (errorCode). El firmware NO da
# covarianza de pose (todo ceros), asi que la CONFIANZA de localizacion la estimamos nosotros (scan-to-map).
HEALTH_JS = r"""(function(){
  if(!window.__healthHook){ window.__healthHook=1;
    var jp=JSON.parse;
    JSON.parse=function(s){ var v=jp.apply(this,arguments);
      try{ if(v && v.topic && (''+v.topic).indexOf('slam_info')>=0){
        var d=(typeof v.data==='string')?jp(v.data):v.data;
        if(d){ if(d.type==='pos_info'){ window.__poseErr=d.errorCode; }
          if(d.type==='robot_data' && d.data){ var m=d.data; var me=0,mm=(m.motorError||[]);
            for(var i=0;i<mm.length;i++){ if(mm[i]) me++; }
            var mtm=0,mhi=-1,mtt=(m.motorTemp||[]); for(var j=0;j<mtt.length;j++){ if(mtt[j]>mtm){mtm=mtt[j];mhi=j;} }
            window.__health={bat:m.batteryPower, vol:m.batteryVol, amp:m.batteryAmp, batT:m.batteryTemp,
              cpuT:m.cpuTemp, cpuU:m.cpuUsage, cpuMem:m.cpuMemory, cpuFreq:m.cpuFrequency,
              motTmax:mtm, motThot:mhi, merr:me, motorTemp:mtt, motorError:mm,
              sport:m.sportMode, gait:m.gaitType, t:Date.now()}; }
        }
      }}catch(e){}
      return v;
    };
  } return 1;
})()"""


# IMU completa de rt/lf/lowstate (~15Hz): quaternion, giroscopo, acelerometro, rpy + par max de patas.
IMUFULL_JS = r"""(function(){
  if(!window.__imuFullHook){ window.__imuFullHook=1;
    var jp=JSON.parse;
    JSON.parse=function(s){ var v=jp.apply(this,arguments);
      try{ if(v && v.topic && (''+v.topic).indexOf('lowstate')>=0 && v.data && v.data.imu_state){
        var im=v.data.imu_state; var ms=v.data.motor_state||[]; var mt=0;
        for(var i=0;i<12&&i<ms.length;i++){ var tq=Math.abs(ms[i].tau_est||0); if(tq>mt) mt=tq; }
        window.__imufull={quat:im.quaternion, gyro:im.gyroscope, accel:im.accelerometer, rpy:im.rpy,
          legtau:mt, t:Date.now()};
      }}catch(e){}
      return v;
    };
  } return 1;
})()"""


def read_telemetry(cdp):
    """TODO lo util: errorCode reloc + robot_data (bateria/cpu/motores) + IMU (accel/gyro/rpy/par). Dict o {}."""
    try:
        s = cdp.eval("JSON.stringify({err:(window.__poseErr==null?null:window.__poseErr), "
                     "h:(window.__health||null), imu:(window.__imufull||null), cov:(window.__reloccov||null)})")
        return json.loads(s) if s else {}
    except Exception:
        return {}


def read_health(cdp):
    """Compat: solo errorCode + robot_data (sin IMU)."""
    t = read_telemetry(cdp)
    return {"err": t.get("err"), "h": t.get("h")}


def _telem_row(hh):
    """Aplana read_telemetry() a una fila de telemetria para el dataset (campos no nulos)."""
    h = dict(hh.get("h") or {}); im = hh.get("imu") or {}
    h.pop("t", None)
    row = dict(h)
    row["err"] = hh.get("err")
    if im:
        row["accel"] = im.get("accel"); row["gyro"] = im.get("gyro")
        row["rpy"] = im.get("rpy"); row["legtau"] = im.get("legtau"); row["quat"] = im.get("quat")
    cov = hh.get("cov")
    if cov and len(cov) >= 15:                # covarianza de pose (6x6). traza de posicion xx+yy+zz
        row["pose_cov"] = cov                 # completa (el G1 la manda a 0)
        row["pose_cov_trace"] = round(cov[0] + cov[7] + cov[14], 5)
    return {k: v for k, v in row.items() if v is not None}


def ref_points():
    """Puntos de pared 2D (frame G1) del mapa de referencia elegido por env G1_REFMAP:
       'summit' (DEFECTO) = mapa del Summit ALINEADO a A/B (bien orientado) -> summit/ref_map_g1.json;
       'g1'               = mapa propio del G1 (dataset/map_full.json) — OJO: puede salir rotado/desalineado."""
    choice = os.environ.get("G1_REFMAP", "summit").lower()
    here = os.path.dirname(os.path.abspath(__file__))
    if choice != "g1":                          # por defecto, el mapa Summit alineado (orientacion correcta)
        p = os.path.join(here, "summit", "ref_map_g1.json")
        try:
            if os.path.exists(p):
                pts = [(q[0], q[1]) for q in json.load(open(p)).get("points", [])]
                if pts:
                    return pts
        except Exception:
            pass
    # 'g1': mapa propio del G1 (puede estar rotado/desalineado)
    def _clip(pts):
        return [(a, b) for (a, b) in pts if -15 <= a <= 15 and -15 <= b <= 15]   # quita outliers de reloc
    p3 = os.path.join(DATASET_DIR, "map_full.json")
    try:
        if os.path.exists(p3):
            d = json.load(open(p3)); pts = []
            for q in d.get("points", []):
                if len(q) >= 3 and HBAND_LO <= q[2] <= HBAND_HI:
                    pts.append((q[0], q[1]))
                elif len(q) == 2:
                    pts.append((q[0], q[1]))
            pts = _clip(pts)
            if pts:
                return pts
    except Exception:
        pass
    try:
        d = json.load(open(MAP_FILE)); OC = d.get("OCELL", g.OCELL)
        return _clip([(c[0] * OC, c[1] * OC) for c in d.get("cells", [])])
    except Exception:
        return []


def load_ref_map():
    """Mapa de referencia (set de celdas OCELL, frame G1) para confianza/plan. Ver ref_points()."""
    return set((round(x / g.OCELL), round(y / g.OCELL)) for x, y in ref_points())


def load_static_map():
    """Celdas OCELL de MUEBLES ESTATICOS conocidos (nav_map.json, acumulado en waypoint/sweep).
    Van al plan GLOBAL como coste BLANDO (G1_GLOBALMAP=static), NUNCA como pared dura: nav_map
    acumula celdas de marco/hoja EN la boca de la puerta (medido 2026-07-02: sellan el vano en
    x[-3.6,-3.2] y[0.8,1.6]) y como inf cerrarian el paso = gotcha de paredes fantasma."""
    try:
        d = json.load(open(MAP_FILE)); OC = d.get("OCELL", g.OCELL)
        return set((c[0], c[1]) for c in d.get("cells", [])
                   if -15 <= c[0] * OC <= 15 and -15 <= c[1] * OC <= 15)
    except Exception:
        return set()


def global_static_costmap(walls, furn):
    """Costmap del plan GLOBAL en modo 'static' (P8b, 2026-07-02):
      - PAREDES (refmap) = inf SIN inflar. Medido offline: la puerta real mide ~4 celdas (0.8m)
        y con INFL_HARD=1 se sella -> el modo 'hard' fallaba el A* en CADA replanificacion y caia
        al fallback (solo paredes, sin hard_set): el mapa duro nunca llegaba de verdad al plan.
      - MUEBLES conocidos + persistentes saturados + colisiones = coste BLANDO (GLOB_SOFT) con
        halo de 1 celda (GLOB_HALO): el plan los RODEA si hay sitio pero JAMAS sellan un paso.
    Validado offline con el mapa real (maps_out/map_static_local): A<->B y C->B cruzan SIEMPRE la
    puerta real (34 celdas, sin ruta fantasma por el hueco sin mapear de arriba), plan determinista
    (mismo mapa -> mismo plan: adios ddir tembloroso), 1 celda de mueble pisada vs 2-4 antes.
    La seguridad fina sigue siendo del DWA local con el laser vivo (arquitectura Nav2)."""
    cm = {c: math.inf for c in walls}
    if GLOB_WALL_HALO > 0:                       # halo blando junto a pared: centra el plan en vanos
        for (ox, oy) in walls:                   # y lo despega de bordes/cajoneras (run 171431, mano dcha)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    c = (ox + dx, oy + dy)
                    if cm.get(c) != math.inf and cm.get(c, 0.0) < GLOB_WALL_HALO:
                        cm[c] = GLOB_WALL_HALO
    for (ox, oy) in furn:
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                c = (ox + dx, oy + dy)
                if cm.get(c) == math.inf:
                    continue
                w = GLOB_SOFT if (dx == 0 and dy == 0) else GLOB_HALO
                if cm.get(c, 0.0) < w:
                    cm[c] = w
    return cm


def frame_check(lg, x0, y0):
    """Compara la pose INICIAL con el waypoint mas cercano. Si arrancas FISICAMENTE en un waypoint pero la
    pose dice que estas lejos de su coordenada guardada => la relocalizacion de esta sesion difiere de la
    de cuando se capturaron los waypoints (deriva de frame). Devuelve dict para el dataset."""
    try:
        wps = json.load(open(WP_FILE))
    except Exception:
        wps = {}
    if not wps:
        return None
    near, w = min(wps.items(), key=lambda kv: math.hypot(kv[1]["x"] - x0, kv[1]["y"] - y0))
    off = math.hypot(w["x"] - x0, w["y"] - y0)
    msg = (f"FRAME-CHECK start=({x0:+.2f},{y0:+.2f}) wp_mas_cercano={near}({w['x']:+.2f},{w['y']:+.2f}) "
           f"offset={off:.2f}m")
    try:
        lg.write(msg + "\n")
    except Exception:
        pass
    if off < 0.4:
        print(f"  {msg}  -> frame OK (pose y waypoints alineados)")
    else:
        print(f"  {msg}\n  >>> OJO: arrancas lejos del waypoint mas cercano. Si fisicamente estas EN un "
              f"waypoint, la relocalizacion difiere de la captura -> re-captura A/B/C en ESTA sesion.")
    return {"start": [round(x0, 2), round(y0, 2)], "nearest_wp": near, "offset_m": round(off, 2)}


def match_score(live_cells, ref_cells):
    """Confianza de localizacion ESTIMADA: fraccion de celdas del laser en vivo que caen sobre (o junto a)
    una celda del mapa conocido. ~1 = bien localizado (el laser encaja con el mapa); bajo = deriva/duda.
    Es la auto-evaluacion meta-cognitiva (el robot no da covarianza)."""
    if not ref_cells or not live_cells:
        return None
    hit = 0
    for (cx, cy) in live_cells:
        if any((cx + dx, cy + dy) in ref_cells for dx in (-1, 0, 1) for dy in (-1, 0, 1)):
            hit += 1
    return round(hit / len(live_cells), 3)


def grab_cam(cdp):
    """Foto actual de la camara del robot (data:image base64) o None. En una colision = prueba VISUAL de
    lo que habia (p.ej. una mesa que el LiDAR no ve)."""
    try:
        j = cdp.eval(g.CAM_JS)
        return j if (j and isinstance(j, str) and j.startswith("data:image")) else None
    except Exception:
        return None


def cam_floor_clear(cam):
    """Decodifica el frame de camara y devuelve (frac_suelo_CENTRO, near_run) con la segmentacion de suelo
    de g1_nav_v2. frac alto + near_run bajo = camino DESPEJADO por delante. Util donde el LASER es RUIDOSO
    (puerta/mesa): la VISION confirma si se puede pasar. None si no hay frame."""
    if not cam or not isinstance(cam, str) or not cam.startswith("data:image"):
        return None, None
    try:
        import base64, io
        from PIL import Image
        img = Image.open(io.BytesIO(base64.b64decode(cam.split(",", 1)[1]))).convert("RGB")
        lf, cf, rf, refS, nrun, mcont = g.floor_free_bands(img)
        return cf, nrun
    except Exception:
        return None, None


def grab_full_cloud(cdp, cap=12000):
    """Nube 'location' CRUDA (todas las alturas, sin filtrar) -> lista plana [x,y,z,...]. Para guardar en
    una colision y poder ver despues si era una MESA (tablero a media altura con hueco debajo) u otro
    obstaculo invisible a la banda de torso del LiDAR."""
    try:
        s = cdp.eval("JSON.stringify(window.__relocbuf||[])")
        buf = json.loads(s) if s else []
    except Exception:
        return []
    return buf[:cap]


def clear_dir(x, y, yaw_deg, off_deg, obs_pts, maxd=2.5, cone=25.0):
    """Distancia (m) al obstaculo mas cercano en un cono de +-cone deg hacia (yaw+off). Sustituye a
    clear_ahead() (que leia la nube de MAPEO, no disponible en modo nav): aqui se calcula de obs_pts."""
    best = maxd
    aim = yaw_deg + off_deg
    for (ox, oy) in obs_pts:
        dx = ox - x; dy = oy - y; d = math.hypot(dx, dy)
        if d < 0.05 or d >= best:
            continue
        ang = abs((math.degrees(math.atan2(dy, dx)) - aim + 180) % 360 - 180)
        if ang < cone:
            best = d
    return best


def global_plan(sx, sy, gx, gy, oset):
    """A* GLOBAL de (sx,sy) -> (gx,gy). Es solo para VISUALIZAR/orientar, asi que usa el costmap SIN inflado
    (marca solo las celdas-pared como bloqueadas) para que pueda cruzar puertas estrechas. Si no halla ruta
    devuelve la RECTA, para que la ventana siempre muestre algo."""
    cm = {c: math.inf for c in oset}
    cells = g.astar((round(sx / g.OCELL), round(sy / g.OCELL)),
                    (round(gx / g.OCELL), round(gy / g.OCELL)), cm, margin=25)
    if cells and len(cells) > 1:
        return [(c[0] * g.OCELL, c[1] * g.OCELL) for c in cells]
    return [(sx, sy), (gx, gy)]    # fallback: recta origen->destino


def _cov_deficit(x, y, yaw, live, refmap, oc, near_blind):
    """Deficit de cobertura del sector frontal (ver cabecera COV).

    Devuelve (deficit, n_predichos, ciego): 'deficit' es la fraccion de rumbos en los que el
    MAPA predice retorno y el barrido vivo NO lo da; 'ciego' la fraccion de esos rumbos cuyo
    retorno predicho cae dentro de la banda que el fabricante recorta, o sea inobservable por
    construccion y no por una perdida de cobertura. Se separan a proposito: son fundamentos
    distintos y agregarlos volveria a confundir "no puedo ver aqui" con "aqui no hay cobertura".
    Sin mapa devuelve (None, 0, None) -- ausencia de referencia no es ausencia de obstaculo."""
    if not refmap:
        return (None, 0, None)
    paso = oc * 0.5
    pred = falt = ciego = 0
    for k in range(COV_NRAY):
        off = -COV_SECT + (2.0 * COV_SECT * k / max(1, COV_NRAY - 1))
        a_ = math.radians(yaw + off)
        ca, sa = math.cos(a_), math.sin(a_)
        r_map = r_live = None
        r = paso
        while r <= COV_R:
            c = (int(round((x + ca * r) / oc)), int(round((y + sa * r) / oc)))
            if r_map is None and c in refmap:
                r_map = r
            if r_live is None and c in live:
                r_live = r
            if r_map is not None and r_live is not None:
                break
            r += paso
        if r_map is None:                      # el mapa no predice retorno: este rumbo no informa
            continue
        pred += 1
        if r_map < near_blind:
            ciego += 1
        elif r_live is None or r_live > r_map + oc:
            falt += 1                          # predicho y AUSENTE -> cobertura perdida
    if not pred:
        return (None, 0, None)
    return (round(falt / pred, 3), pred, round(ciego / pred, 3))


# cov_missing v2 = BASE ACUMULADA: la marcha va contra la union de los ultimos PERSIST_N
# barridos frescos + el actual, con matching EXACTO. Por que (medido 25-ago, gemelo,
# cristal escenificado vs control, ventana de aproximacion):
#   - instantanea exacta (v1):      40% vs 32% de muestras >=3  -> ruido en el umbral
#   - instantanea vecindad 3x3:      0% vs  0%                  -> la tolerancia traga el cristal
#   - acumulada exacta (esta):      77% vs 39% en replay 'op'   -> la unica que separa
# La referencia de sesion se construye sobre 'op' ACUMULADO (tools/mapa_visibilidad.py):
# campo y referencia deben compartir base de evidencia. Apagado por defecto hasta pasar
# la validacion en vivo. VALIDADO 25-ago (piernas GLASS_COVM2B/BASE_COVM2B): cristal con
# racha de 5 ticks >=3 en la aproximacion (2.0->1.2 m del vano) y control con CERO muestras
# >=3 (max 2). Por defecto ON desde entonces; G1_COVM_V2=0 recupera la variante instantanea.
COVM_V2 = os.environ.get("G1_COVM_V2", "1") == "1"


def _cov_missing_celdas(x, y, yaw, live, covref, oc, near_blind, prev):
    """(n_persistentes, celdas_de_este_barrido) contra la referencia de visibilidad de SESION.

    Misma marcha de rayos que _cov_deficit, pero el resultado es POR CELDA: una celda cuenta si
    esta predicha y AUSENTE en este barrido Y en el anterior (prev). El transitorio de un giro
    dura un barrido; una perdida de cobertura, toda la aproximacion."""
    paso = oc * 0.5
    cel = {}
    for k in range(COV_NRAY):
        off = -COV_SECT + (2.0 * COV_SECT * k / max(1, COV_NRAY - 1))
        a_ = math.radians(yaw + off)
        ca, sa = math.cos(a_), math.sin(a_)
        r_map = r_live = None
        c_map = None
        r = paso
        while r <= COV_R:
            c = (int(round((x + ca * r) / oc)), int(round((y + sa * r) / oc)))
            if r_map is None and c in covref:
                r_map, c_map = r, c
            if r_live is None and c in live:
                r_live = r
            if r_map is not None and r_live is not None:
                break
            r += paso
        if r_map is None or r_map < near_blind:
            continue
        falta = r_live is None or r_live > r_map + oc
        cel[c_map] = cel.get(c_map, True) and falta     # verla con UN rayo basta
    n = sum(1 for c, falta in cel.items() if falta and prev.get(c) is True)
    return n, cel


def _cov_deficit_v2(x, y, yaw, live, refcells, oc, near_blind):
    """Instrumento v2 del deficit de cobertura (21-ago, tras la negativa del cristal real).

    El control de pared maciza invalido al v1: cov_def 1.000 con la pared VISIBLE a 2.16 m
    (c0), porque (a) el mapa historico esta desfasado y (b) el march exacto por celdas se salta
    superficies finas en distancia (aliasing rayo-celda). Dos arreglos, ambos declarados:
      - referencia = el mapa de VISIBILIDAD DE SESION (G1_COVREF), no el historico;
      - el matching del retorno vivo acepta VECINDAD 3x3 alrededor de cada paso del rayo
        (una superficie a 0.2 m de la linea exacta del rayo cuenta como presente), y la
        tolerancia de rango sube a 2 celdas.
    Devuelve (deficit, n_predichos, ciego) como el v1. Pendiente de validar en gemelo con
    SIM_GLASS + control de pared antes de usarse en sesion."""
    if not refcells:
        return (None, 0, None)
    paso = oc * 0.5
    pred = falt = ciego = 0
    for k in range(COV_NRAY):
        off = -COV_SECT + (2.0 * COV_SECT * k / max(1, COV_NRAY - 1))
        a_ = math.radians(yaw + off)
        ca, sa = math.cos(a_), math.sin(a_)
        r_map = r_live = None
        r = paso
        while r <= COV_R:
            cx = int(round((x + ca * r) / oc))
            cy = int(round((y + sa * r) / oc))
            if r_map is None and (cx, cy) in refcells:
                r_map = r
            if r_live is None:
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        if (cx + dx, cy + dy) in live:
                            r_live = r
                            break
                    if r_live is not None:
                        break
            if r_map is not None and r_live is not None:
                break
            r += paso
        if r_map is None:
            continue
        pred += 1
        if r_map < near_blind:
            ciego += 1
        elif r_live is None or r_live > r_map + 2.0 * oc:
            falt += 1
    if not pred:
        return (None, 0, None)
    return (round(falt / pred, 3), pred, round(ciego / pred, 3))


def load_cov_ref():
    """Celdas OCELL de la referencia de visibilidad (G1_COVREF). Vacio si no esta configurada."""
    if not COV_REF_FILE:
        return None
    try:
        pts = json.load(open(COV_REF_FILE)).get("points", [])
        return set((round(p[0] / g.OCELL), round(p[1] / g.OCELL)) for p in pts)
    except Exception as e:
        print("  [AVISO] G1_COVREF ilegible (%s): sin cov_missing" % e)
        return None


def navigate_to(cdp, lg, wx, wy, label, vshare=None, lock=None, stop_event=None):
    """NAVEGA A->B sobre el mapa cargado: A* (firmware-like) + DWA local, obstaculos de la nube 'location',
    contacto por IMU/odom y desatasco (reusados del frontier explorer). Para al llegar. Ctrl+C aborta.
    Si se pasan vshare/lock/stop_event, publica el estado para la ventana en vivo (modo viz)."""
    print(f"\n>>> GOTO '{label}' -> ({wx:+.2f},{wy:+.2f}). Mando en mano (L2+B). Ctrl+C aborta.")
    if RUN_ENV != "real":
        print(f"    ENTORNO: {RUN_ENV.upper()}" + (f" (id: {os.environ.get('G1_SIM_ID')})" if os.environ.get("G1_SIM_ID") else "")
              + "  — esta run se registra como SIMULACION, no cuenta como run de robot")
    lg.write(f"\n=== RUN ours '{label}' -> ({wx:+.2f},{wy:+.2f})  {time.strftime('%Y-%m-%d %H:%M:%S')} "
             f"env={RUN_ENV}{('/' + os.environ.get('G1_SIM_ID')) if os.environ.get('G1_SIM_ID') else ''} ===\n"); lg.flush()
    cdp.eval(g.LOWSTATE_JS)                       # contacto rapido por par/accel
    # espera pose + primera nube
    print("  Esperando pose localizada y primera nube...", end="", flush=True)
    for _ in range(30):
        src, p, _ = read_pose(cdp)
        n = int(cdp.eval("(window.__relocbuf||[]).length") or 0)
        if p and n > 50:
            break
        time.sleep(0.3)
    else:
        print(" sin datos. ¿Mapa cargado y robot RELOCALIZADO en la app?"); return False
    print(" ok.")
    # --- GATE de relocalizacion (como la app): pose inicial vs waypoint mas cercano ---
    fc0 = frame_check(lg, p[0], p[1])
    if fc0 and fc0["offset_m"] > GATE_M and os.environ.get("G1_NOGATE") != "1":
        print(f"\n  >>> RELOCALIZACION DUDOSA: arrancas a {fc0['offset_m']}m del waypoint {fc0['nearest_wp']}. NO navego.")
        print("      Re-localiza en la app (que los puntitos encajen con el mapa) y reintenta (override: G1_NOGATE=1).")
        lg.write("GATE-BLOCKED reloc dudosa\n")
        return False

    omap = {}                                     # celda OCELL -> ultimo instante visto (mapa ACTIVO de obstaculos; lo lee el resto del codigo)
    oscore = {}                                   # celda OCELL -> score de confianza (anti-ruido: sube al verse, baja al no verse)
    oseen  = {}                                   # celda OCELL -> ultimo instante visto (para el TTL de respaldo del score)
    yaw_prev = None; yaw_prev_t = None            # para estimar yaw_rate (gate de rotacion)
    yaw_rate = 0.0                                # deg/s (se recalcula cada tick)
    livehist = deque(maxlen=PERSIST_N)            # ultimos N barridos del laser (filtro de ruido por persistencia)
    stale_streak = 0                              # ticks seguidos con la nube 'location' sin refrescar (dedup)
    # --- DIAGNOSTICO de sensado/filtro (solo observacion, no toca el control) ---
    n_ticks = 0; stale_ticks = 0; gated_ticks = 0    # contadores de ticks (total / nube repetida / gate de giro)
    fresh_times = deque(maxlen=64)                # instantes de barridos FRESCOS -> Hz efectivo del topic 'location'
    filt_rej = 0.0; rej_sum = 0.0; rej_n = 0      # fraccion del barrido RECHAZADA por el filtro de persistencia
    nz_sum = 0.0; nz_max = 0.0; nz_n = 0          # laser_noise (SensingMonitor) acumulado por barrido fresco
    map_add = 0; map_del = 0; obs_max = 0         # churn del mapa activo (celdas que entran/salen) + pico
    tick_add = 0; tick_del = 0; prev_oset_diag = set()
    safer_ins = 0                                 # celdas forzadas por el override de seguridad SAFE_R
    njumps = 0; pend_jump = False                 # saltos de reloc (contador + pendiente para el proximo update fresco)
    jump_times = deque(maxlen=32)                 # instantes de reloc_jump (guardia anti-divergencia, cuenta TODOS)
    tick_dts = deque(maxlen=3000); last_tick_t = None   # duracion del ciclo de control (WebView lento = ticks largos)
    ss2 = {"reliability": 1.0, "laser_noise": 0.0, "loc_conf": 1.0, "c0_std": 0.0,
           "scan_churn": 0.0, "reloc_rate10s": 0}      # ultimo update FRESCO del SensingMonitor (se reusa en ticks stale)
    loc = None
    vox_mem = {}; vox_seen = {}   # memoria de voxels: celda -> t de la ultima confirmacion sana
    vox_inj = 0; vox_ray = 0      # (diag) celdas sostenidas y celdas soltadas por rayo en el tick

    def diag_summary():
        """Resumen de diagnostico de sensado para summary del run (todo medido, nada estimado)."""
        shz = (len(fresh_times) - 1) / max(1e-6, fresh_times[-1] - fresh_times[0]) if len(fresh_times) >= 2 else 0.0
        p95 = sorted(tick_dts)[int(0.95 * (len(tick_dts) - 1))] if tick_dts else 0.0
        return {"laser_noise_mean": round(nz_sum / nz_n, 3) if nz_n else None,
                "laser_noise_max": round(nz_max, 3) if nz_n else None,
                "filt_rej_mean": round(rej_sum / rej_n, 3) if rej_n else None,
                "scan_hz": round(shz, 2),
                "stale_pct": round(100.0 * stale_ticks / n_ticks, 1) if n_ticks else None,
                "gated_pct": round(100.0 * gated_ticks / n_ticks, 1) if n_ticks else None,
                "safer_inserts": safer_ins, "map_adds": map_add, "map_dels": map_del,
                "obs_max": obs_max, "reloc_jumps": njumps,
                "colmap_cells": len(colmap),
                "meta2_mode": META2_MODE,                     # 2) tambien en el summary de la run
                "meta2_capped_ticks": m2_ncap,                # nº de ticks donde META2 CAPO de verdad (>0 <=> activo actuando)
                "tick_ms_p95": round(1000.0 * p95, 1) if tick_dts else None}
    colmap = set()                                # colisiones PERMANENTES (no re-chocar en el mismo sitio)
    plan_pts = []; plan_t = 0; carrot = None
    fhist = []; prev_fwd = False; recov = None; ncol = 0; last_col_t = -99; rside = 1
    low_t = 0; last_low = None; lt_base = []; ah_base = []
    dhist = []; brk = None; brk_cool = 0; nbrk = 0; nstop = 0
    esc = None; esc_done = False                  # maniobra de ESCAPE inicial (una sola vez por run)
    pose_t = time.time(); last_pose = None; t0 = time.time(); tprint = 0
    trail = []
    # --- diagnostico: calibracion de giro (signo real vs comandado) + spin ---
    prev_yaw = None; prev_cmd = (0, 0, 0, 0); prev_lt = None; prev_xy = None; strafecal = []
    last_sent = (0.0, 0.0, 0.0, 0)   # lo REALMENTE publicado el tick anterior (prev_cmd se
                                     # reasigna ANTES de las guardas con el cmd de ESTE tick)
    last_ph_sent = ""                # la FASE tal como se ACTUO el tick anterior, CON los
                                     # marcadores de guardia (!H !D !C !c !S !M ~). La muestra
                                     # se graba antes de la cadena de guardas, asi que 'phase'
                                     # jamas lleva marcadores (0 en 33.026 muestras revisadas
                                     # el 20-ago) y W4 quedaba sin campo de autoridad. Espejo
                                     # exacto de 'sent': el tick anterior, post-guardas.
    spin_acc = 0.0; prog_pos = None; prog_t = t0; turncal = []; phcount = {}
    minc0 = 9.9
    rd = RunRecorder("ours", label, (wx, wy))
    refmap = load_ref_map(); health_t = 0; hh = {}; cloud_ok = False; cloud_warned = False
    covref = load_cov_ref(); cov_missing = None; _covm_prev = {}
    illum_ema = None; dvis_gate = 0; _dvg_log = 0   # gate de iluminacion del DOOR-VIS
    staticmap = load_static_map()                            # muebles conocidos (nav_map) -> coste blando del plan global
    gplan = []; gplan_t = 0; cam_t = 0; cam_jpg = None
    aggressive = (os.environ.get("G1_AGGRESSIVE") == "1")    # modo agresivo (forzable; si no, se activa al atascarse)
    eng = {"state": None, "ok": 0, "wt": 0, "cool": 0.0, "ts": 0.0}   # engagement de puerta (mapa estatico)
    # --- META2 (G1_META2=1 shadow / =2 activo): gobernanza DCE con Meta-Reasoner 2.0 ---
    meta2 = None; m2o = None
    if META2_MODE in ("1", "2"):
        if Meta2Bridge is None:
            print("  META2 pedido pero g1_meta2_bridge/meta-reasoner-2.0 no importable -> OFF")
        else:
            try:
                m2cfg = os.environ.get("G1_M2_CFG") or None      # config alternativa (p.ej. payload agua)
                if m2cfg:                                        # config/ tras la reorg (ver bridge)
                    m2cfg = g1_meta2_bridge._resolve_path(m2cfg, "config")
                meta2 = Meta2Bridge(m2cfg)
                print(f"  META2 {'ACTIVO (techo por analogia)' if META2_MODE == '2' else 'SHADOW (solo log)'}: "
                      f"Meta-Reasoner 2.0, analogia inicial {meta2.applied}")
                lg.write(f"META2 mode={META2_MODE} cfg={m2cfg or 'config_meta2_g1door.json'} init={meta2.applied}\n"); lg.flush()
            except Exception as e:
                print("  META2 no disponible:", repr(e))
    m2win = deque(); m2_help_t0 = None; m2_warn_t = 0.0; m2_mode_logged = False   # escalada de experiencia
    m2trk = deque()                                          # (t,x,y,cmd_ly) para el canal de RESISTENCIA (mobility)
    m2_ncap = 0; m2_cap_on = False                           # nº de ticks realmente CAPADOS por META2 (prueba de modo activo)
    # REGISTRO DEL MODO a prueba de dudas (runs 100739/100927: no se pudo verificar si fueron mode=2):
    # 1) cabecera del dataset (autodescriptivo aunque el goto.log no viaje)
    rd.rec["meta2_mode"] = META2_MODE
    rd.rec["meta2_enabled"] = bool(meta2 is not None)
    best_d = 1e9; best_d_t = t0; ROBOT_R0 = g.ROBOT_R        # progreso hacia B + holgura normal (para restaurar)
    vis_center = None; vis_nearrun = None; vis_t = 0         # VISION (suelo despejado) para la puerta (laser ruidoso ahi)
    vis_log_t = 0                                            # throttle del log [VIS] (que ve YOLO y si la nav lo usa)
    # --- PERCEPCION GPU offboard (opcional): depth -> scan virtual que VE la mesa invisible al LiDAR ---
    perc = g1_perception.make_client_from_env(g.OCELL) if g1_perception else None
    perc_worker = g1_perception.PerceptionWorker(perc) if perc else None       # consulta GPU en HILO APARTE (no congela el control)
    # --- GATE DE VISION (duro): sin percepcion VERIFICADA no se navega. ---
    # Runs 164306/164456: 2 colisiones con c0=2.50 y perc_n=0 -> la mesa LiDAR-ciega NO se ve sin vision.
    # No basta con que G1_PERC este definido: se hace un TEST REAL (frame de la camara -> /perceive) para
    # comprobar que YOLO+depth procesan de verdad. Override explicito: G1_NOVIS=1 (corre sin vision, avisa).
    vis_ready = False
    if perc_worker:
        perc_worker.start()
        print("  [perc] test real (frame de camara -> /perceive)...", end="", flush=True)
        for _ in range(10):                           # hasta ~25s (warmup del modelo / primer frame)
            fr = grab_cam(cdp)
            if fr:
                rtest = perc.query(fr, 0.0, 0.0, 0.0)   # pose dummy: solo validamos que el pipeline responde
                if rtest is not None:
                    vis_ready = True
                    # 'color_pts' solo existe si el servidor corre con G1_FLOORCOLOR=1 -> estado visible
                    # desde el robot (run 120438: creiamos que el color estaba ON y perc_n=0 lo desmintio).
                    fcs = "ON" if isinstance(getattr(rtest, "raw", None), dict) and "color_pts" in rtest.raw else "off"
                    print(f" OK ({len(rtest.detections or [])} dets, {len(rtest.cells)} celdas, "
                          f"{rtest.latency_ms:.0f}ms, floorcolor={fcs})")
                    lg.write(f"PERC-TEST OK dets={len(rtest.detections or [])} cells={len(rtest.cells)} "
                             f"lat={rtest.latency_ms:.0f}ms floorcolor={fcs}\n"); lg.flush()
                    break
            time.sleep(0.5)
        if not vis_ready:
            print(" FALLO")
    if not vis_ready:
        why = ("G1_PERC sin definir" if perc is None else
               f"el servidor no proceso el frame de test ({getattr(perc, 'last_err', None)})")
        if os.environ.get("G1_NOVIS") == "1":
            print("\n  " + "!" * 74)
            print(f"  !!  VISION OFF ({why}) pero G1_NOVIS=1: navego SIN vision bajo tu responsabilidad.")
            print("  !!  La MESA invisible al LiDAR no se vera.")
            print("  " + "!" * 74 + "\n")
            lg.write(f"NO-VISION override G1_NOVIS=1 ({why})\n"); lg.flush()
        else:
            print("\n  >>> VISION NO VERIFICADA: NO navego. " + why + ".")
            print("      1) Arranca perception_server.py en el Ubuntu (terminal aparte).")
            print("      2) Lanza con G1_PERC=127.0.0.1:8008 y comprueba '[perc] ... OK'.")
            print("      (override consciente: G1_NOVIS=1 navega sin vision; la mesa sera invisible)")
            lg.write(f"PERC-GATE BLOCKED: {why} {time.strftime('%Y-%m-%d %H:%M:%S')}\n"); lg.flush()
            return False
    perc_t = 0; perc_cells = set(); perc_dets = []; nperc = 0; perc_raw = {}
    perc_rx_t = None; perc_seen = -1              # edad de la ULTIMA respuesta real del server (diag P3)
    cambuf = deque(maxlen=20)                     # (t, jpg) ultimos ~6s de camara (autopsia pre-colision)
    film_t = 0.0                                  # ultimo frame de la pelicula guardado
    hg_log_t = 0.0; c0_hard = 9.9; hard_set = set()   # guardia de alta confianza
    door_seen = {}; door_sticky = set()               # FIX C: confirmaciones/celdas fijadas del marco
    cov_def = None; cov_n = 0; cov_blind = None; cov_ms = None   # cobertura de lidar (cabecera COV)
    cb_hits = 0; cb_on = False                        # FIX B: persistencia del aviso de camara
    # --- FEEDBACK RENXI (rama tutor-feedback): canal humano + validez retrospectiva ---
    h_col_seen = 0.0                                  # ultimo reporte humano de colision consumido
    laser_trust = 1.0; c0_prev = 2.5; lt_prev_ncol = 0   # "use the past data to calculate the
                                                      # validity of the laser reading"
    assist_mem = []                                   # memoria episodica de asistencias humanas
    try:
        assist_mem = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                 "assist_memory.json")))
    except Exception:
        pass
    assist_recall_t = 0.0
    dc_contra = []                                    # timestamps de sugerencias CONTRADICTORIAS
    dc_last = None                                    # ultima sugerencia de centro de puerta
    crumbs = []; retreat = None; retreat_cool = 0.0   # RETREAT v2 (portado de main)
    rt_prev_ncol = 0; rt_col_ts = []; rt_count = 0
    stk_hist = []
    meta_state = "NORMAL"; ms_ev_t = 0.0; iface_q = 1.0   # maquina de estados META
    exit_p0 = None                                    # punto de arranque del run (EXIT-CAUTION)
    h_assist_seen = 0.0                               # ultima asistencia humana consumida
    cdp_lat = 0.05                                    # latencia del ultimo envio CDP (iface_q)
    dg_on = False                                     # DOORGUARD: ya avisado en esta pasada de zona
    omni_log_t = 0.0                                  # OMNI-GUARD: throttle de log/evento
    envch_gone_miss = {}                              # ENV-CHANGE: celda refmap -> barridos frescos sin verse
    envch_new_n = 0; envch_gone_n = 0; envch_onpath_n = 0
    envch_evt_t = 0.0                                 # throttle de eventos env_*
    crumbs = []                                       # RETREAT: migas (x,y) cada 0.15m, ~4.5m de cola
    retreat = None; retreat_cool = 0.0; rt_prev_ncol = 0
    rt_col_ts = []; rt_count = 0                      # v2: colisiones recientes + retiradas por run
    stk_hist = []                                     # (t,x,y) para detectar atasco (4s de ventana)
    vis_lost_t = 0.0; vis_lost = False                # watchdog de vision en caliente
    door_side = None; door_geom_t = 0.0               # detector GEOMETRICO de cruce (independiente del FSM)
    sei = g1_metrics.SEIMetrics()                            # clearance + progression por tick (las 2 metricas del tutor)
    sens = g1_metrics.SensingMonitor()                       # auto-evaluacion de sensado (ruido/fiabilidad) = feedback de capacidad
    m_clear = 0.0; m_prog = 0.0; m_rel = 1.0; m_cl = 0.0; m_cr = 0.0
    print(f"  mapa de referencia: {len(refmap)} celdas" + (" (sin mapa -> confianza N/A)" if not refmap else "")
          + f" | muebles estaticos (nav_map): {len(staticmap)} celdas | plan global: {GLOBAL_SRC}")
    lg.write(f"GLOBALMAP src={GLOBAL_SRC} walls={len(refmap)} static={len(staticmap)} "
             f"soft={GLOB_SOFT}/{GLOB_HALO}\n"); lg.flush()
    try:
        while not (stop_event is not None and stop_event.is_set()):
            now = time.time()
            n_ticks += 1
            if last_tick_t is not None:
                tick_dts.append(now - last_tick_t)    # duracion real del ciclo (diagnostico WebView/CDP lento)
            last_tick_t = now
            src, p, pcd = read_pose(cdp)
            if not p:
                cdp.eval(g.STOP_JS)
                if now - pose_t > 3.0:
                    print("\n  POSE PERDIDA (3s). STOP. Relocaliza en la app y reintenta."); return False
                time.sleep(0.2); continue
            x, y, yaw = p[0], p[1], yaw_of(p)         # yaw en GRADOS
            reloc_flag = False
            if last_pose is not None and math.hypot(x - last_pose[0], y - last_pose[1]) > 0.5:
                jd = math.hypot(x - last_pose[0], y - last_pose[1])     # >0.5m en un ciclo (~0.1s) = salto reloc
                reloc_flag = True
                njumps += 1; pend_jump = True         # (diag) contado en summary + entregado al proximo update fresco
                rd.event("reloc_jump", now - t0, x, y, {"dist": round(jd, 2),
                                                        "from": [round(last_pose[0], 2), round(last_pose[1], 2)]})
                lg.write(f"RELOC-JUMP {jd:.2f}m de ({last_pose[0]:+.2f},{last_pose[1]:+.2f}) a ({x:+.2f},{y:+.2f})\n")
                jump_times.append(now)
                # --- GUARDIA: saltos repetidos = la reloc esta DIVERGIENDO -> STOP ya (no caminar en fantasia) ---
                if RELOC_GUARD:
                    recent_j = sum(1 for tj in jump_times if now - tj <= RELOC_STOP_WIN)
                    if recent_j >= RELOC_STOP_N:
                        cdp.eval(g.STOP_JS); time.sleep(0.2); cdp.eval(g.STOP_JS)
                        print(f"\n  >>> RELOC DIVERGE: {recent_j} saltos en {RELOC_STOP_WIN:.0f}s. STOP y ABORTO. "
                              f"Re-localiza en la app antes de reintentar (G1_RELOCGUARD=0 desactiva el guardia).")
                        lg.write(f"RELOC-DIVERGE STOP {recent_j} saltos/{RELOC_STOP_WIN:.0f}s "
                                 f"pos=({x:+.2f},{y:+.2f}) {time.strftime('%Y-%m-%d %H:%M:%S')}\n"); lg.flush()
                        rd.event("reloc_diverge_stop", now - t0, x, y, {"jumps_in_win": recent_j})
                        rd.save_cloud("relocdiv", [round(x, 3), round(y, 3), round(yaw, 1)], grab_full_cloud(cdp))
                        rd.finish("aborted_reloc_diverge",
                                  {"time_s": round(now - t0, 2), "path_m": round(_path_len(trail), 2),
                                   "collisions": ncol, "c0min": round(minc0, 2), **diag_summary()})
                        return False
            if last_pose is None or abs(x - last_pose[0]) > 1e-4 or abs(y - last_pose[1]) > 1e-4:
                pose_t = now; last_pose = (x, y)
            if not trail:
                rd.rec["frame_check"] = fc0
            if not trail or math.hypot(x - trail[-1][0], y - trail[-1][1]) > 0.05:
                trail.append((x, y))

            d_goal = math.hypot(wx - x, wy - y)
            if d_goal < NAV_REACH:                    # --- LLEGADA ---
                cdp.eval(g.STOP_JS); time.sleep(0.2); cdp.eval(g.STOP_JS)
                print(f"\n  LLEGADO a '{label}' ({wx:+.2f},{wy:+.2f}); error={d_goal:.2f} m, colisiones={ncol}.")
                lg.write(f"REACHED {label} err={d_goal:.2f} ncol={ncol} {time.strftime('%Y-%m-%d %H:%M:%S')}\n"); lg.flush()
                T = now - t0; plen = _path_len(trail)
                straight = math.hypot(wx - trail[0][0], wy - trail[0][1]) if trail else 0.0
                rd.save_cloud("end", [round(x, 3), round(y, 3), round(yaw, 1)], grab_full_cloud(cdp))
                rd.finish("reached", {"time_s": round(T, 2), "path_m": round(plen, 2),
                                      "path_m_k8": round(_path_len(trail, 8), 2),   # escala declarada
                                      "path_scale_note": "path_m nativo: valido DENTRO de un sistema; entre sistemas usar path_m_k8",
                                      "straight_m": round(straight, 2),
                                      "efficiency": round(straight / plen, 2) if plen > 0 else 0.0,
                                      "collisions": ncol, "c0min": round(minc0, 2),
                                      "perc_queries": nperc,
                                      "start": {"x": round(trail[0][0], 3), "y": round(trail[0][1], 3)} if trail else None,
                                      **diag_summary()})
                if vshare is not None:                # marca llegada en la ventana antes de salir
                    with lock:
                        vshare["ph"] = "LLEGADO"; vshare["x"] = x; vshare["y"] = y
                return True

            # --- OBSTACULOS de la nube 'location' (frame mapa) -> mapa con SCORE/DECAY (anti-ruido) ---
            live = reloc_cells(cdp)                   # celdas del barrido ACTUAL (laser en vivo)
            scan_fresh = reloc_cells.fresh            # True = buffer NUEVO (dedup: no re-contar el mismo barrido)
            if not scan_fresh:
                stale_streak += 1; stale_ticks += 1
                if stale_streak == STALE_WARN_TICKS:  # aviso UNA vez por racha (nube congelada, WebRTC caido?)
                    lg.write(f"SCAN-STALE nube 'location' sin refrescar {stale_streak} ticks (mapa congelado)\n")
            else:
                stale_streak = 0; fresh_times.append(now)
            if COV and scan_fresh:                # una vez por BARRIDO, no por tick de control
                _t_cov = time.time()
                cov_def, cov_n, cov_blind = _cov_deficit(x, y, yaw, live, refmap,
                                                         g.OCELL, g.NEAR_BLIND)
                if covref is not None:
                    base_covm = live
                    if COVM_V2:
                        for _h in livehist:
                            base_covm = base_covm | _h
                    cov_missing, _covm_prev = _cov_missing_celdas(
                        x, y, yaw, base_covm, covref, g.OCELL, g.NEAR_BLIND, _covm_prev)
                cov_ms = round((time.time() - _t_cov) * 1000.0, 2)   # coste real (diag)
            if live:
                cloud_ok = True
            elif not cloud_ok and not cloud_warned and now - t0 > 4.0:
                print("\n  [AVISO] no llega la nube 'location' -> NO puedo planificar (sin obstaculos).")
                print("          ¿se ven los PUNTITOS del laser en la app?")
                lg.write("NO-CLOUD warning\n"); cloud_warned = True
            # --- CONFIRMACION de ENTRADA (confiar mas en el mapa) ---
            # Una celda del MAPA estatico (pared conocida) se confirma YA; una celda que SOLO ve el laser
            # necesita verse en >=PERSIST_K de los ultimos PERSIST_N barridos (filtra el parpadeo de 1 barrido:
            # ruido del cabeceo o nube desplazada por un salto de reloc).
            if scan_fresh:                            # solo VOTA un barrido NUEVO (dedup: el mismo buffer
                livehist.append(live)                 # repetido 2-3 ticks ya no se auto-confirma)
            confirmed = set()
            for c in live:
                if refmap and c in refmap:            # confirmado por el MAPA -> instantaneo (confiamos en el mapa)
                    confirmed.add(c)
                elif sum(1 for h in livehist if c in h) >= PERSIST_K:   # confirmado por PERSISTENCIA del laser
                    confirmed.add(c)
            confirmed = {c for c in confirmed         # descarta campo cercano (anillo fantasma del cabeceo)
                         if math.hypot(c[0] * g.OCELL - x, c[1] * g.OCELL - y) >= g.NEAR_BLIND}
            if scan_fresh:                            # (diag) fraccion del barrido RECHAZADA por el filtro:
                lf = sum(1 for c in live              #  celdas del barrido (fuera de NEAR_BLIND) que NO quedaron
                         if math.hypot(c[0] * g.OCELL - x, c[1] * g.OCELL - y) >= g.NEAR_BLIND)
                filt_rej = (1.0 - len(confirmed) / lf) if lf else 0.0
                rej_sum += filt_rej; rej_n += 1
            # --- MEMORIA DE VOXELS + BARRIDO POR RAYOS (ver cabecera) ---
            vox_inj = 0; vox_ray = 0
            if VOXMEM:
                if scan_fresh:
                    for c in confirmed:               # ya filtrado a distancia SANA
                        vox_seen[c] = vox_seen.get(c, 0) + 1
                        if vox_seen[c] >= VOXMEM_K:
                            vox_mem[c] = now
                    # evidencia POSITIVA de ausencia: los rayos del barrido de ESTE tick.
                    # Los extremos son los centros de las celdas vivas; lo que el rayo
                    # atraviesa antes de llegar esta demostrado libre.
                    if VOXMEM_RAYS and _vox_despeja is not None and vox_mem:
                        _pts = [(c[0] * g.OCELL, c[1] * g.OCELL) for c in live]
                        _libres, _, _ = _vox_despeja((x, y), _pts, set(vox_mem),
                                                     g.OCELL, g.NEAR_BLIND, True)
                        for c in _libres:
                            vox_mem.pop(c, None); vox_seen.pop(c, None)
                        vox_ray = len(_libres)
                if vox_mem:
                    _add = []
                    for c, ts in list(vox_mem.items()):
                        if now - ts > VOXMEM_TTL:
                            vox_mem.pop(c, None); vox_seen.pop(c, None); continue
                        if math.hypot(c[0] * g.OCELL - x, c[1] * g.OCELL - y) > VOXMEM_R:
                            continue                  # fuera de la banda: manda el barrido
                        if scan_fresh and c in live:
                            continue                  # el barrido la ve: no hace falta memoria
                        _add.append((ts, c))
                    _add.sort(reverse=True)
                    _keep = {c for _, c in _add[:VOXMEM_MAX]}
                    vox_inj = len(_keep)
                    if VOXMEM_ACT:
                        confirmed |= _keep       # solo la rama que ACTUA toca el planificador
            # --- FIX C: marco de puerta pegajoso (ver cabecera). Cuenta confirmaciones REALES
            # (post-filtro, a distancia sana) y fija la celda; el bypass reinyecta las fijadas
            # aunque el robot este encima (NEAR_BLIND) -> c0_hard estable durante el cruce.
            if DOOR_STICKY:
                if scan_fresh:
                    for c in confirmed:
                        if math.hypot(c[0] * g.OCELL - DOOR_CX, c[1] * g.OCELL - DOOR_CY) <= DOORSTICK_R:
                            # v3 (smoke 20260721_100417, cazado por el gemelo: v1 fijaba ruido
                            # vivo -> 6 celdas fantasma en la boca -> DWA contra el borde real,
                            # 9 colisiones; y la adyacencia (v2) no discrimina en rejilla de
                            # 0.2m con vano de 0.85m). SOLO se fija lo que esta EN el mapa
                            # estatico: la jamba es pared CONOCIDA cuyo score decae al entrar
                            # en NEAR_BLIND durante el cruce (el mecanismo exacto del parpadeo
                            # de c0_hard); el ruido vivo no esta en refmap y jamas se fija.
                            if refmap and c in refmap:
                                door_seen[c] = door_seen.get(c, 0) + 1
                                if door_seen[c] >= 2 and len(door_sticky) < 240:
                                    door_sticky.add(c)
                if door_sticky:
                    confirmed |= door_sticky
            # --- PERCEPCION GPU (HILO APARTE): depth -> scan virtual (la MESA que el LiDAR no ve) + suelo despejado ---
            if perc_worker is not None and now - perc_t > PERC_PERIOD:
                _fr = grab_cam(cdp)
                # MEDIR SIEMPRE, ACTUAR SOLO SI EL GATE ESTA ACTIVO (25-ago, cazado por el
                # ensayo general): illum_b se computaba solo con DOOR_VIS_GATE=1, asi que las
                # piernas de CONTROL del A/B no llevaban NINGUN registro de iluminacion --
                # justo donde hace falta para demostrar que ambos brazos vieron la misma luz.
                # illum_b es un campo de I1 (calidad de sensado): se observa siempre. Quien
                # decide actuar es dvis_gate, que sigue siendo None con el gate apagado.
                if _fr and _fr.startswith("data:image"):
                    try:                                  # luma media del frame (EMA): ~1ms a 320px
                        import base64 as _b64, io as _io
                        from PIL import Image as _Im, ImageStat as _Ist
                        _lum = _Ist.Stat(_Im.open(_io.BytesIO(
                            _b64.b64decode(_fr.split(",", 1)[1]))).convert("L")).mean[0]
                        illum_ema = _lum if illum_ema is None else 0.8 * illum_ema + 0.2 * _lum
                    except Exception:
                        pass
                cambuf.append((now, _fr))                      # buffer para la autopsia pre-colision
                perc_worker.submit(_fr, x, y, yaw)             # no bloquea: el hilo hace la consulta GPU
                perc_t = now
                if FILM_PERIOD > 0 and _fr and now - film_t > FILM_PERIOD:   # PELICULA de la run
                    rd.save_cam(f"t{int(now - t0):03d}s", _fr)
                    film_t = now
            # WATCHDOG de vision EN CALIENTE (auditoria 22-jul: el 21-jul el canal murio 62s
            # en plena run sin aviso — el gate solo comprobaba al ARRANCAR)
            if perc_worker is not None and perc_rx_t is not None:
                _dvis = now - perc_rx_t
                if not vis_lost and _dvis > 15.0:
                    vis_lost = True; vis_lost_t = now
                    lg.write(f"[VIS] canal de percepcion SIN RESPUESTA hace {_dvis:.0f}s t={now - t0:.0f}s\n")
                    print(f"  [VIS] *** percepcion caida ({_dvis:.0f}s sin respuesta) ***")
                    rd.event("vision_lost", now - t0, x, y, extra={"since_s": round(_dvis, 1)})
                elif vis_lost and _dvis < 3.0:
                    vis_lost = False
                    lg.write(f"[VIS] percepcion RECUPERADA tras {now - vis_lost_t:.0f}s\n")
                    rd.event("vision_back", now - t0, x, y, extra={"out_s": round(now - vis_lost_t, 1)})
            if perc_worker is not None and perc_worker.latest is not None:
                res = perc_worker.latest                       # ultimo resultado disponible (puede ir 1-2 ticks por detras)
                nperc = perc_worker.n_ok
                if nperc != perc_seen:                         # respuesta NUEVA (no la cacheada del tick anterior)
                    perc_rx_t = now; perc_seen = nperc
                perc_cells = res.cells
                perc_dets = res.detections or []
                perc_raw = res.raw if isinstance(res.raw, dict) else {}   # telemetria del canal de color/puerta
                if res.free_center is not None:                # VISION basada en depth/seg (mejor que la heuristica)
                    vis_center, vis_nearrun = res.free_center, (res.near_run or 0); vis_t = now
            vis_conf = {c for c in perc_cells         # obstaculos de vision (mesa) -> tambien pasan por el score
                        if math.hypot(c[0] * g.OCELL - x, c[1] * g.OCELL - y) >= g.NEAR_BLIND}
            seen_now = confirmed | vis_conf           # candidatos vistos AHORA (laser confirmado + vision)
            # --- GATE DE ROTACION: estima yaw_rate; si el giro es rapido, la nube va poco fiable ---
            if yaw_prev is not None and yaw_prev_t is not None and now > yaw_prev_t:
                yaw_rate = abs((yaw - yaw_prev + 180.0) % 360.0 - 180.0) / (now - yaw_prev_t)
            yaw_prev, yaw_prev_t = yaw, now
            turning_fast = (YAW_GATE > 0) and (abs(prev_cmd[2]) > RX_GATE)   # GATE por giro COMANDADO (no por bamboleo)
            if turning_fast:
                gated_ticks += 1                      # (diag) % del run con el mapa congelado por giro

            if OLDMAP:
                # --- MODO ANTIGUO (G1_OLDMAP=1): entra al instante y dura NAV_OMAP_TTL -> UNION de 60s (acumula ruido) ---
                for c in seen_now:
                    omap[c] = now
                omap = {c: t for c, t in omap.items() if now - t < NAV_OMAP_TTL}
            else:
                # SEGURIDAD (prioridad sobre el anti-ruido): un obstaculo de laser CONFIRMADO y CERCA entra YA
                # al mapa a score maximo, aunque el gate este congelando por giro y sin esperar al umbral. Esto
                # habria metido la mesa a tiempo (choque run 164456: c0=2.50 hasta el impacto, perc_n=0).
                # SOLO el laser CONFIRMADO salta el gate y el umbral (seguridad de campo cercano).
                # La VISION pasa por el score normal (+1/frame, obstaculo en ~2 frames = ~1s): sigue
                # siendo rapida para la mesa/escritorio, pero mientras el robot PIVOTA (turning_fast)
                # no inserta nada -> imposible pintarse la jaula de clamps de la run 130524.
                near_now = {c for c in confirmed
                            if math.hypot(c[0] * g.OCELL - x, c[1] * g.OCELL - y) < SAFE_R}
                safer_ins += sum(1 for c in near_now if oscore.get(c, 0.0) < SC_OBST)   # (diag) forzadas por SAFE_R
                for c in near_now:
                    oscore[c] = SC_CAP; oseen[c] = now
                # --- SCORE/DECAY: obstaculo solo si se ve en la MAYORIA de barridos recientes ---
                # Solo se actualiza con barrido fresco (si cae la nube, no penalizamos -> no se borra el mapa).
                # El decay solo actua dentro de SC_RANGE (ventana de plan); fuera de rango se conserva por TTL.
                # GATE: no actualizar mientras se gira rapido NI con barrido repetido (scan_fresh: el mismo
                # buffer no puede subir el score dos veces ni penalizar dos veces -> nube congelada = sin datos).
                if scan_fresh and (live or seen_now) and not turning_fast:
                    for c in confirmed:               # laser confirmado: sube score; mapa estatico -> tope (instantaneo)
                        oscore[c] = SC_CAP if (refmap and c in refmap) else min(SC_CAP, oscore.get(c, 0.0) + SC_HIT)
                        oseen[c] = now
                    for c in vis_conf:                # vision: sube score (necesita ~2 frames -> filtra depth ruidoso)
                        if c not in confirmed:
                            oscore[c] = min(SC_CAP, oscore.get(c, 0.0) + SC_HIT)
                            oseen[c] = now
                    for c in list(oscore.keys()):     # decay + purga
                        if c in seen_now:
                            continue
                        if math.hypot(c[0] * g.OCELL - x, c[1] * g.OCELL - y) < SC_RANGE:
                            oscore[c] -= SC_MISS      # en rango y no visto -> ruido intermitente: penaliza
                        if oscore[c] <= 0.0 or now - oseen.get(c, now) >= NAV_OMAP_TTL:
                            oscore.pop(c, None); oseen.pop(c, None)
                # mapa ACTIVO = celdas con score suficiente (omap sigue siendo celda->instante para el resto del codigo)
                omap = {c: oseen[c] for c, s in oscore.items() if s >= SC_OBST}
            # --- OBSTACULOS de ALTA CONFIANZA (idea de Renxi 2026-07-02): pared/mueble PERSISTENTE.
            # El robot los trataba igual que el ruido (mismo margen agresivo de 0.13) y los "ignoraba".
            # DURO = celda del mapa estatico confirmada, o score SATURADO (visto casi siempre), o colision.
            if OLDMAP:
                hard_set = set(colmap)
            else:
                hard_set = {c for c, sc in oscore.items()
                            if sc >= SC_CAP or (refmap and c in refmap)} | colmap
            # (diag) churn del mapa ACTIVO: cuantas celdas entran/salen (ruido entrando = adds altos sin moverse)
            cur_oset_diag = set(omap.keys())
            tick_add = len(cur_oset_diag - prev_oset_diag); tick_del = len(prev_oset_diag - cur_oset_diag)
            map_add += tick_add; map_del += tick_del; prev_oset_diag = cur_oset_diag
            obs_max = max(obs_max, len(cur_oset_diag))
            oset = set(omap.keys()) | colmap
            # --- LOG [VIS]: que ve YOLO/depth y si la navegacion lo USA (entra al mapa) o lo IGNORA ---
            if perc_dets or now - vis_log_t > 1.0:
                vis_log_t = now
                obst_dets = [d for d in perc_dets if str(d.get("label", "")).lower() in VIS_OBST_LABELS]
                for d in obst_dets:
                    lab = d.get("label"); rng = d.get("range_m"); bd = d.get("bearing_deg") or 0.0
                    if rng is None:
                        lg.write(f"[VIS] YOLO ve {lab}@{bd:+.0f} SIN RANGO (depth no dio dist) -> IGNORADA\n"); continue
                    aa = math.radians(yaw + bd)
                    dc = (round((x + rng * math.cos(aa)) / g.OCELL), round((y + rng * math.sin(aa)) / g.OCELL))
                    inmap = any((dc[0] + i, dc[1] + j) in omap for i in (-2, -1, 0, 1, 2) for j in (-2, -1, 0, 1, 2))
                    tag = "USADA (en mapa)" if inmap else f"IGNORADA (gate={'G' if turning_fast else '-'} perc_n={len(perc_cells)})"
                    lg.write(f"[VIS] YOLO ve {lab} conf={(d.get('conf') or 0):.2f} @{bd:+.0f}/{rng}m -> {tag}\n")
                if obst_dets or now - vis_log_t <= 0.01:   # resumen periodico aunque no haya detecciones
                    nvis_map = sum(1 for c in vis_conf if c in omap)
                    dstr = ",".join(f"{d.get('label')}@{(d.get('bearing_deg') or 0):+.0f}/{d.get('range_m')}m" for d in obst_dets) or "-"
                    cdoor = perc_raw.get("door") or {}
                    lg.write(f"[VIS] perc_n={len(perc_cells)} age={round(now - perc_rx_t, 1) if perc_rx_t is not None else '-'} "
                             f"free_c={vis_center if vis_center is not None else '-'} "
                             f"vismap={nvis_map}/{len(vis_conf)} color={perc_raw.get('color_pts', '-')} "
                             f"carpet={perc_raw.get('carpet_pct', '-')} cnear={perc_raw.get('color_near', '-')} "
                             f"crmin={perc_raw.get('color_rmin', '-')} door={cdoor.get('bearing_deg', '-')} "
                             f"dets=[{dstr}]\n")
                lg.flush()
            op = [(cx * g.OCELL, cy * g.OCELL) for (cx, cy) in oset
                  if abs(cx * g.OCELL - x) < 2.6 and abs(cy * g.OCELL - y) < 2.6]
            op_hard = [(cx * g.OCELL, cy * g.OCELL) for (cx, cy) in hard_set
                       if abs(cx * g.OCELL - x) < 2.6 and abs(cy * g.OCELL - y) < 2.6]
            c0 = clear_dir(x, y, yaw, 0, op); minc0 = min(minc0, c0)
            c0_hard = clear_dir(x, y, yaw, 0, op_hard)         # holgura frontal contra PAREDES/persistentes
            mm = sei.update(now - t0, d_goal, c0)            # 2 metricas SEI: clearance (espacio libre) + progression (avance a B)
            m_clear, m_prog = mm["clearance"], mm["progression"]
            cl_left = clear_dir(x, y, yaw, +60, op); cl_right = clear_dir(x, y, yaw, -60, op)   # clearance IZQ/DCHA (Renxi: balancear)
            m_cl = min(1.0, cl_left / 1.5); m_cr = min(1.0, cl_right / 1.5)
            # VISION en zona estrecha (puerta/mesa): el laser ahi es ruidoso -> mira si la camara ve suelo despejado.
            # Si hay servidor de percepcion GPU, ya rellena vis_* arriba; esto es el FALLBACK heuristico (sin GPU).
            if perc is None and c0 < 1.1 and now - vis_t > 0.5:
                vis_center, vis_nearrun = cam_floor_clear(grab_cam(cdp)); vis_t = now
            cmd = None; ph = ""

            # --- CONTACTO (odom-stall fiable / IMU rapido por par-accel) ---
            if now - low_t > 0.2:
                lw = g.read_low(cdp)
                if lw:
                    last_low = (math.hypot(lw.get("ax", 0.0), lw.get("ay", 0.0)), lw.get("legtau", 0.0))
                low_t = now
            cur_ah, cur_lt = last_low if last_low else (None, None)
            if prev_fwd:
                fhist.append((now, x, y))
            fhist = [h for h in fhist if now - h[0] <= 2.0]
            mvd = math.hypot(x - fhist[0][1], y - fhist[0][2]) if len(fhist) >= 2 else 0.0
            if prev_fwd and cur_lt is not None and mvd > 0.10:
                lt_base.append(cur_lt); lt_base = lt_base[-40:]
                ah_base.append(cur_ah); ah_base = ah_base[-40:]
            contact = False; ctype = ""
            if recov is None and brk is None and now - last_col_t > 4.0:
                if len(fhist) >= 8 and now - fhist[0][0] >= 0.9 and mvd < 0.05:
                    contact = True; ctype = "odom"
                elif (g.IMU_CONTACT and prev_fwd and cur_lt is not None and len(fhist) >= 5
                      and now - fhist[0][0] >= 0.5 and mvd < 0.04):
                    bl = sorted(lt_base)[len(lt_base) // 2] if len(lt_base) >= 5 else 15.0
                    ba = sorted(ah_base)[len(ah_base) // 2] if len(ah_base) >= 5 else 1.5
                    if cur_lt > bl * 1.5 + 3.0 or cur_ah > ba + 1.8:
                        contact = True; ctype = "imu"
            if contact:
                ncol += 1; last_col_t = now
                yr = math.radians(yaw); fxx, fyy = math.cos(yr), math.sin(yr); pxx, pyy = -fyy, fxx
                for d in (0.30, 0.45):                     # marca PEQUEÑA (antes encerraba el paso de vuelta)
                    for L in (-0.1, 0.0, 0.1):
                        colmap.add((round((x + d * fxx + L * pxx) / g.OCELL),
                                    round((y + d * fyy + L * pyy) / g.OCELL)))
                cl = clear_dir(x, y, yaw, +55, op); cr = clear_dir(x, y, yaw, -55, op)
                rside = 1 if cl >= cr else -1
                recov = {"ph": "BACK", "t0": now}; fhist = []; plan_pts = []
                print(f"\n  COLISION #{ncol} [{ctype}] en ({x:+.2f},{y:+.2f}) -> marco y recupero.")
                _cd = ",".join(f"{d.get('label')}@{(d.get('bearing_deg') or 0):+.0f}/{d.get('range_m')}m"
                               for d in perc_dets if str(d.get('label', '')).lower() in VIS_OBST_LABELS) or "-"
                lg.write(f"COLISION #{ncol} [{ctype}] pos=({x:+.2f},{y:+.2f}) yaw={yaw:+.0f} c0={c0:.2f} obs={len(oset)} "
                         f"perc_n={len(perc_cells)} free_c={vis_center if vis_center is not None else '-'} dets=[{_cd}] "
                         f"-> {'VISION CIEGA (perc_n=0): mesa LiDAR-ciega no vista' if len(perc_cells) == 0 else 'vision aportaba celdas'}\n")
                rd.event("collision", now - t0, x, y,
                         {"src": ctype,
                          "color_pts": perc_raw.get("color_pts"), "carpet_pct": perc_raw.get("carpet_pct"),
                          "color_near": perc_raw.get("color_near"),
                          # mapa ACTIVO alrededor en el instante del golpe: ¿el mapa LO SABIA y el DWA fallo,
                          # o el obstaculo nunca llego al mapa? (la pregunta clave de cada autopsia)
                          "omap_near": [[c[0], c[1]] for c in oset
                                        if math.hypot(c[0] * g.OCELL - x, c[1] * g.OCELL - y) < 2.5][:400]})
                rd.save_cloud(f"col{ncol}", [round(x, 3), round(y, 3), round(yaw, 1)], grab_full_cloud(cdp))
                rd.save_cam(f"col{ncol}", grab_cam(cdp))
                # AUTOPSIA: frames de la APROXIMACION (t-1s/-2s/-3s) — la foto del impacto suele ser
                # una pared borrosa a 0 cm; lo diagnostico esta en lo que se veia ANTES.
                for _ago in (1.0, 2.0, 3.0):
                    _best = None
                    for _tf, _fj in cambuf:
                        if _fj and (_best is None or abs((now - _tf) - _ago) < abs((now - _best[0]) - _ago)):
                            _best = (_tf, _fj)
                    if _best and abs((now - _best[0]) - _ago) < 0.7:
                        rd.save_cam(f"col{ncol}_pre{int(_ago)}s", _best[1])

            # --- ESCAPE inicial (HANDOFF 8.8): si NACE encajonado (B con la nariz en el sofa), retrocede
            #     ~ESC_DIST en recto ANTES de planificar: asi el primer giro se hace CON sitio, no rozando.
            #     Va ANTES de RECUPERACION/DESATASCO a proposito: una colision real (recov) sigue mandando.
            esc_cp = perc_raw.get("carpet_pct") if isinstance(perc_raw.get("carpet_pct"), (int, float)) else None
            # 'nace encajonado' = pegado a algo SIN haberse movido aun (<0.3m del punto de arranque). El gate
            # de desplazamiento evita el falso disparo al acercarse a la puerta (142725: carpet 0.46 en t=4.7s
            # tras andar 1.5m — run perfecta, ahi NO toca retroceder).
            if (ESCAPE_ON and esc is None and not esc_done and now - t0 < 5.0 and d_goal > 1.5
                    and trail and math.hypot(x - trail[0][0], y - trail[0][1]) < 0.30
                    and (c0 < ESC_TRIG or (esc_cp is not None and esc_cp < ESC_CARPET))):
                esc_src = "laser" if c0 < ESC_TRIG else "vision"
                esc = {"t0": now, "x0": x, "y0": y, "src": esc_src}
                print(f"\n  ESCAPE [{esc_src}]: nazco encajonado (c0={c0:.2f}m carpet={esc_cp if esc_cp is not None else '-'})"
                      f" -> retrocedo {ESC_DIST:.1f}m antes de planificar.")
                lg.write(f"ESCAPE-START src={esc_src} c0={c0:.2f} carpet={esc_cp} pos=({x:+.2f},{y:+.2f})\n")
                rd.event("escape_start", now - t0, x, y,
                         {"src": esc_src, "c0": round(c0, 2), "carpet_pct": esc_cp})
            if esc is not None:
                esc_el = now - esc["t0"]; esc_mv = math.hypot(x - esc["x0"], y - esc["y0"])
                esc_rear = clear_dir(x, y, yaw, 180, op)
                # 'despejado' exige AMBOS sensores contentos (si hay vision): el sofa es LiDAR-ciego a 40cm,
                # con c0 solo el escape acabaria en el primer tick con la nariz aun en el cojin.
                esc_clear = (c0 >= 0.85) and (esc_cp is None or esc_cp >= ESC_CARPET + 0.05)
                if esc_el < 6.0 and esc_mv < ESC_DIST and esc_rear > 0.50 and not esc_clear:
                    cmd = (0, -0.35, 0, 0); ph = "ESC-BK"   # -0.35: la deadzone del stick es ~0.3
                else:
                    why = ("despejado" if esc_clear else "distancia hecha" if esc_mv >= ESC_DIST
                           else "timeout" if esc_el >= 6.0 else "sin hueco detras")
                    print(f"  ESCAPE fin [{why}]: retrocedi {esc_mv:.2f}m, c0={c0:.2f}m "
                          f"carpet={esc_cp if esc_cp is not None else '-'} -> planifico.")
                    lg.write(f"ESCAPE-END why={why} moved={esc_mv:.2f} c0={c0:.2f} carpet={esc_cp} rear={esc_rear:.2f}\n")
                    rd.event("escape_end", now - t0, x, y,
                             {"why": why, "moved": round(esc_mv, 2), "c0": round(c0, 2), "carpet_pct": esc_cp})
                    esc = None; esc_done = True; plan_pts = []; dhist = []

            # --- RECUPERACION: mini paso atras (si hay hueco detras) + pivota ---
            if recov is not None:
                el = now - recov["t0"]
                if recov["ph"] == "BACK":
                    rear = clear_dir(x, y, yaw, 180, op)
                    if el < 0.45 and rear > 0.6:
                        cmd = (0, -0.35, 0, 0); ph = "R-BACK"
                    else:
                        recov = {"ph": "TURN", "t0": now}; el = 0
                if recov is not None and recov["ph"] == "TURN":
                    if el < 1.3:
                        cmd = (0, 0, -g.AV_TURN if rside > 0 else g.AV_TURN, 0); ph = "R-TURN"
                    else:
                        recov = {"ph": "GO", "t0": now}; el = 0
                if recov is not None and recov["ph"] == "GO":
                    if el < 1.0 and c0 > g.EXP_FWD_MIN:
                        cmd = (0, g.FWD_SPEED, 0, 0); ph = "R-GO "
                    else:
                        recov = None

            # --- DESATASCO: sin avanzar STUCK_SEC -> mini atras + giro grande hacia el lado mas abierto ---
            dhist.append((now, x, y))
            dhist = [h for h in dhist if now - h[0] <= g.STUCK_SEC]
            if (recov is None and brk is None and now > brk_cool and len(dhist) >= 2
                    and now - dhist[0][0] >= g.STUCK_SEC * 0.9
                    and math.hypot(x - dhist[0][1], y - dhist[0][2]) < g.STUCK_DISP):
                cl = clear_dir(x, y, yaw, +55, op); cr = clear_dir(x, y, yaw, -55, op)
                nbrk += 1; brk = {"ph": "BACK", "t0": now, "dir": -g.AV_TURN if cl >= cr else g.AV_TURN}
                plan_pts = []
                print(f"\n  DESATASCO #{nbrk} en ({x:+.2f},{y:+.2f}).")
                lg.write(f"DESATASCO #{nbrk} pos=({x:+.2f},{y:+.2f})\n")
            if brk is not None:
                el = now - brk["t0"]
                if brk["ph"] == "BACK":
                    rear = clear_dir(x, y, yaw, 180, op)
                    if el < 0.45 and rear > 0.6:
                        cmd = (0, -0.35, 0, 0); ph = "BRK-BK"
                    else:
                        brk = {"ph": "TURN", "t0": now, "dir": brk["dir"]}; el = 0
                if brk is not None and brk["ph"] == "TURN":
                    if el < g.BRK_TURN_SEC:
                        cmd = (0, 0, brk["dir"], 0); ph = "BRK-TR"
                    else:
                        brk = None; brk_cool = now + 6.0; dhist = []; plan_pts = []

            # --- MODO AGRESIVO: si lleva atascado sin ACERCARSE a B, baja inflado y holgura (con minimo seguro) ---
            if d_goal < best_d - 0.15:
                best_d = d_goal; best_d_t = now
            if not aggressive and now - best_d_t > AGGR_AFTER:
                aggressive = True
                print(f"\n  >>> MODO AGRESIVO ON ({AGGR_AFTER:.0f}s sin acercarse a B): reduzco inflado y holgura "
                      f"(min seguridad {AGGR_ROBOT_R}m) para cruzar la puerta.")
                lg.write(f"AGGRESSIVE-ON t={now-t0:.0f}s d={d_goal:.2f}\n")
                rd.event("aggressive_on", now - t0, x, y, {"d": round(d_goal, 2)})
            # EXTENSION B: holgura del DWA gobernada por el perfil (la 'inflacion' local de la
            # tesis, L3/Pl_env modulando). El agresivo conserva su minimo validado.
            _prr = (m2o or {}).get("robot_r") if META2_MODE == "2" else None
            g.ROBOT_R = AGGR_ROBOT_R if aggressive else (_prr if _prr else ROBOT_R0)
            # DOORGUARD: cuerpo con margen real en zona-vano — SOLO fuera de agresivo (el
            # minimo 0.24 del agresivo existe justo para enhebrar el vano atascado; recorte
            # del verificador). En crucero cerca del vano, semiancho fisico con brazos.
            if DOORGUARD and not aggressive and g.ROBOT_R < 0.28 \
                    and math.hypot(x - DOOR_CX, y - DOOR_CY) < DOORGUARD_R:
                g.ROBOT_R = 0.28

            # --- PLAN A* + CONTROL LOCAL DWA (hacia el WAYPOINT, no una frontera) ---
            if cmd is None:
                if (not plan_pts) or (now - plan_t > g.PLAN_SEC):
                    # PLAN: 1) sobre el LASER VIVO (rapido y preciso para ACERCARSE, como antes);
                    #       2) si falla (puerta sellada/estrecha en el mapa vivo) -> sobre el MAPA DE
                    #          REFERENCIA limpio SIN inflado (que tiene la puerta abierta). DWA = seguridad.
                    scell = (round(x / g.OCELL), round(y / g.OCELL))
                    gcell = (round(wx / g.OCELL), round(wy / g.OCELL))
                    if aggressive:
                        # AGRESIVO: obstaculos RECIENTES (4s, no sella la puerta) + mapa limpio (puerta abierta),
                        # SIN colmap (las marcas de colision encierran) ni inflado. Rodea la mesa, cruza la puerta.
                        recent = {c for c, tt in omap.items() if now - tt < 4.0}
                        cm = {c: math.inf for c in (recent | (refmap or set()))}
                    else:
                        # PLAN GLOBAL sobre mapa ESTABLE (Adrian 2026-07-02, viendo el viewer): antes se
                        # planificaba sobre el LASER VIVO (oset) -> el ruido intermitente doblaba el plan
                        # en cada replanificacion (linea verde en zigzag esquivando manchas, 'ddir' de la
                        # puerta temblando -> thrash del DOOR-AL). Arquitectura correcta (la de Nav2):
                        # GLOBAL = mapa cargado + persistentes DUROS (saturados/colisiones; no parpadean);
                        # LOCAL (DWA) = laser vivo. Lo nuevo/no mapeado lo esquiva el DWA y, si de verdad
                        # bloquea la ruta, el modo agresivo ya replanifica con lo reciente.
                        if GLOBAL_SRC == "live":               # G1_GLOBALMAP=live: comportamiento antiguo
                            cm = g.build_costmap(oset)
                        elif GLOBAL_SRC == "ref":              # G1_GLOBALMAP=ref: SOLO el mapa cargado
                            cm = g.build_costmap(refmap or set())
                        elif GLOBAL_SRC == "hard":             # P8 v1 (manana 2026-07-02). OJO medido offline:
                            # con INFL_HARD=1 la puerta (~4 celdas) queda sellada -> este A* fallaba y TODAS
                            # las replanificaciones caian al fallback de abajo (solo paredes, sin hard_set).
                            cm = g.build_costmap(hard_set | (refmap or set()))
                        else:                                  # "static" (DEFECTO, P8b): paredes duras sin inflar
                            # + muebles conocidos/persistentes/colisiones en coste BLANDO (ver docstring de
                            # global_static_costmap). El plan rodea lo conocido pero nunca sella un paso.
                            cm = global_static_costmap(refmap or set(), staticmap | hard_set)
                    cells_path = g.astar(scell, gcell, cm)
                    if not cells_path and refmap:          # ultimo recurso: solo el mapa limpio (siempre tiene la puerta)
                        cells_path = g.astar(scell, gcell, {c: math.inf for c in refmap})
                    plan_pts = [(c[0] * g.OCELL, c[1] * g.OCELL) for c in cells_path] if cells_path else []
                    plan_t = now
                    if not plan_pts:
                        lg.write(f"A*-FAIL goal=({wx:+.1f},{wy:+.1f}) d={d_goal:.1f} obs={len(oset)}\n")
                        rd.event("astar_fail", now - t0, x, y, {"obs": len(oset)})   # jaulas/sellados: visibles en el dataset
                if plan_pts:
                    carrot = g.path_carrot(plan_pts, x, y)
                    # --- ENGAGEMENT DE PUERTA (mapa estatico): pre-entrada -> parar -> alinear -> cruzar RECTO ---
                    engcmd = None
                    # fix del gate (rama): el CROSS sobrevive al gate de d_goal para poder emitir
                    # door_crossed (en el gemelo A->B el goal esta a 1.97m de la puerta y el gate lo
                    # cortaba antes). GUARDADO tras DOORLIB: el A/B formal mostro que con robot_r fijo
                    # 0.22 la liberacion tardia del CROSS cambia la entrada al bolsillo de B (6/16
                    # abortos vs 0/12 del main historico, p=0.024); con perfiles (robot_r 0.28) es
                    # seguro (1/16). Sin flags = comportamiento main exacto.
                    _gatefix = _DOOR_GATEFIX and eng["state"] == "CROSS"
                    if DOOR_ENGAGE and (d_goal > DOOR_MIN_GOAL or _gatefix) and now >= eng["cool"]:
                        # (rama analogy-profiles) un cruce EN CURSO se completa aunque d_goal baje de
                        # DOOR_MIN_GOAL: en el gemelo A->B el goal esta a 1.97m del centro y el gate
                        # cortaba el bloque ANTES del umbral de salida (0.75m) -> door_crossed nunca
                        # se emitia (hueco latente pre-existente; solo B->A lo emitia). Las fases de
                        # aproximacion (GOTO/ALIGN) siguen suprimidas cerca del goal (fix 1858520).
                        ux = math.cos(math.radians(DOOR_AXIS)); uy = math.sin(math.radians(DOOR_AXIS))
                        sgoal = (wx - DOOR_CX) * ux + (wy - DOOR_CY) * uy     # lado del GOAL respecto al vano
                        srob = (x - DOOR_CX) * ux + (y - DOOR_CY) * uy        # lado del ROBOT
                        ddc = math.hypot(x - DOOR_CX, y - DOOR_CY)
                        if eng["state"] is None and ddc < 1.8 and sgoal * srob < 0:
                            eng.update(state="GOTO", ok=0, wt=0, ts=now, c0n=ncol)   # goal al OTRO lado y puerta cerca
                            lg.write(f"DOOR-ENG start t={now-t0:.0f}s pos=({x:+.2f},{y:+.2f}) sgn={'AB' if sgoal>0 else 'BA'}\n")
                            rd.event("door_engage", now - t0, x, y, {"dir": "AB" if sgoal > 0 else "BA"})
                        if eng["state"]:
                            sgn = 1.0 if sgoal > 0 else -1.0
                            # --- CENTRO MEDIDO del vano (ver cabecera G1_DOOR_CENTER) ---
                            door_c_meas = None
                            if DOOR_CENTER:
                                # v2 (auditoria 22-jul): SOLO evidencia estatica (refmap o celdas
                                # fijadas por DOORSTICKY) — el ruido vivo K-de-3 podia desplazar el
                                # centro ~12cm pasando el sanity check (misma leccion que sticky v3).
                                _L = []; _R = []
                                _cand = [c for c in omap.keys()
                                         if (refmap and c in refmap) or c in door_sticky]
                                for (_ccx, _ccy) in _cand:
                                    _mx = _ccx * g.OCELL - DOOR_CX; _my = _ccy * g.OCELL - DOOR_CY
                                    # v3 (shakedown real 24-jul, colision B->A en (-3.74,1.25)): el vano
                                    # es un PASAJE profundo (~0.5m: cara A en x~-3.4, cara B en ~-3.9).
                                    # Con +-0.35 desde B se media la boca ancha de SU cara (gap 1.13) y
                                    # jamas el pellizco de la cara A -> centro sesgado -> golpe justo en
                                    # la cara no vista. +-0.60 cubre TODO el tunel: min(_L)/max(_R) dan
                                    # el estrechamiento REAL, simetrico en ambas direcciones.
                                    if abs(_mx * ux + _my * uy) > 0.60:      # fuera del TUNEL del vano
                                        continue
                                    _lt = -_mx * uy + _my * ux               # lateral (+= izq del eje)
                                    if 0.15 <= _lt <= 1.2:
                                        _L.append(_lt)
                                    elif -1.2 <= _lt <= -0.15:
                                        _R.append(_lt)
                                if (_L and not _R) or (_R and not _L):
                                    if not eng.get("cmiss"):                 # jamba UNICA: avisar, no callar
                                        eng["cmiss"] = 1
                                        lg.write("DOOR-CENTER-MISS: solo una jamba visible; centro del mapa\n")
                                        rd.event("door_center_miss", now - t0, x, y, None)
                                if _L and _R:
                                    _gap = min(_L) - max(_R)
                                    if 0.55 <= _gap <= 1.30:                 # parece el vano de verdad
                                        _c = 0.5 * (min(_L) + max(_R))
                                        _c = max(-DOOR_CENTER_MAX, min(DOOR_CENTER_MAX, _c))
                                        # mediana de 5 + rechazo de saltos: una medicion corrupta
                                        # aislada no mueve el servo
                                        _h = eng.setdefault("cmeds", [])
                                        _h.append(_c); del _h[:-5]
                                        _med = sorted(_h)[len(_h) // 2]
                                        if eng.get("cused") is not None and abs(_med - eng["cused"]) > 0.08:
                                            _med = eng["cused"]              # hold: salto sospechoso
                                        # META-PARAMETRO de recuperacion (Renxi): "number of
                                        # contradictory center suggestions" — sugerencia nueva
                                        # que contradice a la anterior (salto >0.10m) = contradiccion
                                        if dc_last is not None and abs(_c - dc_last) > 0.10:
                                            dc_contra.append(now)
                                        dc_last = _c
                                        del dc_contra[:len(dc_contra) - 20]
                                        eng["cused"] = _med
                                        door_c_meas = _med
                                        if eng.get("cseen") is None:
                                            eng["cseen"] = 1
                                            lg.write(f"DOOR-CENTER medido: off={door_c_meas:+.2f}m gap={_gap:.2f}m "
                                                     f"(jambas L={min(_L):+.2f} R={max(_R):+.2f})\n")
                                            rd.event("door_center", now - t0, x, y,
                                                     {"off": round(door_c_meas, 3), "gap": round(_gap, 2)})
                            # EXTENSION A (rama analogy-profiles): la variante de puerta del bridge
                            # parametriza el engagement (pre-entrada, tolerancia, bias en eje FIJO).
                            _dv = (m2o or {}).get("door") if META2_MODE == "2" else None
                            _engd = (_dv or {}).get("eng_d", DOOR_ENG_D)
                            _atol = (_dv or {}).get("align_tol", DOOR_ALIGN_TOL)
                            _bias = (_dv or {}).get("lat_bias", 0.0)
                            if door_c_meas is not None:
                                _bias = _bias + door_c_meas                   # apuntar al centro MEDIDO
                            exp = (DOOR_CX - sgn * _engd * ux - _bias * uy,
                                   DOOR_CY - sgn * _engd * uy + _bias * ux)   # pre-entrada
                            head = DOOR_AXIS if sgn > 0 else ((DOOR_AXIS + 360) % 360 - 180)           # rumbo de cruce
                            he = (head - yaw + 180) % 360 - 180
                            # DOOR-VIS (ver cabecera): rumbo por vision si hay bearing fresco y sano.
                            # Signo verificado: bearing + = vano a la IZQUIERDA; he + = girar izquierda.
                            if DOOR_VIS:
                                _db = (perc_raw.get("door") or {}).get("bearing_deg") if isinstance(perc_raw, dict) else None
                                _vfresh = perc_rx_t is not None and (now - perc_rx_t) < 1.2
                                dvis_gate = 1 if (DOOR_VIS_GATE and illum_ema is not None
                                                  and illum_ema > DOOR_VIS_LUX) else 0
                                if dvis_gate and _db is not None and not _dvg_log:
                                    _dvg_log = 1
                                    lg.write(f"DOOR-VIS SUPRIMIDO por iluminacion (luma={illum_ema:.0f} > {DOOR_VIS_LUX:.0f}): gobierna el eje del mapa\n")
                                    rd.event("door_vis_gated", now - t0, x, y,
                                             {"luma": round(illum_ema, 1), "bearing_desc": round(_db, 1)})
                                if (not dvis_gate) and _db is not None and _vfresh and abs(_db) < 45 and abs(_db - he) < 35:
                                    if not eng.get("vlog"):
                                        eng["vlog"] = 1
                                        lg.write(f"DOOR-VIS activo: bearing={_db:+.1f} (mapa he={he:+.1f})\n")
                                        rd.event("door_vis", now - t0, x, y, {"bearing": round(_db, 1),
                                                                              "he_mapa": round(he, 1)})
                                    he = _db
                            if eng["state"] == "GOTO":
                                de = math.hypot(exp[0] - x, exp[1] - y)
                                if de <= DOOR_ENG_TOL:
                                    eng.update(state="ALIGN", ok=0, ts=now)
                                    lg.write(f"DOOR-ENG at-position t={now-t0:.0f}s err={de:.2f}m\n")
                                else:
                                    be = (math.degrees(math.atan2(exp[1] - y, exp[0] - x)) - yaw + 180) % 360 - 180
                                    if abs(be) > 25:
                                        engcmd = ((0, 0, -g.AV_TURN if be > 0 else g.AV_TURN, 0), "ENG-T"); eng["wt"] = 0
                                    elif c0 > 0.35:
                                        engcmd = ((0, 0.28, 0, 0), "ENG-F"); eng["wt"] = 0
                                    else:
                                        engcmd = ((0, 0, 0, 0), "ENG-WT"); eng["wt"] += 1
                                        if eng["wt"] > 12:                    # ~4s bloqueado hacia la pre-entrada
                                            eng.update(state=None, cool=now + 8.0)
                                            lg.write("DOOR-ENG abort (GOTO bloqueado) -> logica normal 8s\n")
                                            if meta2 is not None and hasattr(meta2, "door_result"):
                                                try:
                                                    meta2.door_result(False, ncol - eng.get("c0n", ncol))
                                                except Exception:
                                                    pass
                            if eng["state"] == "ALIGN":
                                if abs(he) <= _atol:
                                    eng["ok"] += 1; engcmd = ((0, 0, 0, 0), "ENG-AL.")
                                    if eng["ok"] >= 2:
                                        eng.update(state="CROSS", ts=now)
                                        lg.write(f"DOOR-ENG aligned t={now-t0:.0f}s yaw={yaw:+.0f} he={he:+.1f}\n")
                                elif now - eng["ts"] > 12.0:                  # no consigue alinear: no bloquear la run
                                    eng.update(state=None, cool=now + 8.0)
                                    lg.write(f"DOOR-ENG abort (ALIGN timeout, he={he:+.0f})\n")
                                    if meta2 is not None and hasattr(meta2, "door_result"):
                                        try:
                                            meta2.door_result(False, ncol - eng.get("c0n", ncol))
                                        except Exception:
                                            pass
                                else:
                                    eng["ok"] = 0
                                    # pulso anti-sobregiro (giro real ~15-20 deg/tick por la deadzone)
                                    if abs(he) < 40 and (int(now * 3.33) % 2 == 0):
                                        engcmd = ((0, 0, 0, 0), "ENG-AL.")
                                    else:
                                        engcmd = ((0, 0, -g.AV_TURN if he > 0 else g.AV_TURN, 0), "ENG-AL")
                            if eng["state"] == "CROSS":
                                if srob * sgn > DOOR_EXIT_D:                  # cruzado: control normal otra vez
                                    eng.update(state=None)
                                    lg.write(f"DOOR-ENG CROSSED t={now-t0:.0f}s pos=({x:+.2f},{y:+.2f})\n")
                                    rd.event("door_crossed", now - t0, x, y, None)
                                    if meta2 is not None and hasattr(meta2, "door_result"):
                                        try:
                                            meta2.door_result(True, ncol - eng.get("c0n", ncol))
                                        except Exception:
                                            pass
                                elif abs(he) > DOOR_REALIGN:                  # deriva del bipedo: NO avanzar torcido
                                    _lr = -(x - DOOR_CX) * uy + (y - DOOR_CY) * ux
                                    if door_c_meas is not None and not (DOOR_EXIT_CTR and srob * sgn < 0.0):
                                        _lr = _lr - door_c_meas
                                    if (DOOR_YAW2 and abs(_lr) < DOOR_YAW_LAT
                                            and abs(he) <= DOOR_YAW_HARD):     # centrado: girar SIN parar
                                        engcmd = ((0, DOOR_CTR_VY,
                                                   -g.AV_TURN if he > 0 else g.AV_TURN, 0), "ENG-RG")
                                    else:
                                        eng.update(state="ALIGN", ok=0, ts=now); engcmd = ((0, 0, 0, 0), "ENG-RE")
                                else:
                                    _latrob = -(x - DOOR_CX) * uy + (y - DOOR_CY) * ux
                                    _sal = (srob * sgn < 0.0)                  # ya pasado el centro del vano
                                    if door_c_meas is not None and not (DOOR_EXIT_CTR and _sal):
                                        _latrob = _latrob - door_c_meas       # servo al centro MEDIDO
                                    elif DOOR_EXIT_CTR and _sal and door_c_meas is not None:
                                        if not eng.get("salaviso"):
                                            eng["salaviso"] = 1
                                            lg.write("DOOR-EXIT-CTR saliendo: ignoro centro medido "
                                                     "(%+.2f m) y servoo al eje del mapa\n" % door_c_meas)
                                    latL = _latrob * sgn                       # + = IZQ del eje
                                    if abs(latL) > DOOR_CTR_TOL and abs(srob) > DOOR_CTR_S:  # descentrado -> strafe al eje
                                        lx = DOOR_STRAFE_SIGN * (DOOR_STRAFE if latL < 0 else -DOOR_STRAFE)
                                        if not DOOR_CTR2:
                                            engcmd = ((lx, 0, 0, 0), "ENG-C")
                                        else:                                  # corregir SIN dejar de avanzar
                                            _fwd = 0.0 if abs(latL) > DOOR_CTR_HOLD else DOOR_CTR_VY
                                            if _fwd == 0.0:                    # red de seguridad del hold
                                                if "hold_t0" not in eng:
                                                    eng["hold_t0"] = now; eng["hold_lat"] = abs(latL)
                                                elif (now - eng["hold_t0"] > DOOR_CTR_HOLD_S
                                                      and abs(latL) > eng["hold_lat"] - 0.05):
                                                    _fwd = DOOR_CTR_VY         # no reduce: no bloquear el run
                                                    lg.write("DOOR-CTR2 hold sin efecto %.1fs (lat %.2f->%.2f) -> avanzo\n"
                                                             % (now - eng["hold_t0"], eng["hold_lat"], abs(latL)))
                                                    eng.pop("hold_t0", None)
                                            else:
                                                eng.pop("hold_t0", None)
                                            engcmd = ((lx, _fwd, 0, 0), "ENG-C" if _fwd == 0.0 else "ENG-CG")
                                    else:
                                        engcmd = ((0, 0.28, 0, 0), "ENG-GO")
                                        if "pt" not in eng or now - eng["pt"] > 1.2:   # avance real cada ~1.2s
                                            if "pt" in eng and math.hypot(x - eng["px"], y - eng["py"]) < 0.06:
                                                eng["wt"] += 1               # empujando sin moverse (marco)
                                                if eng["wt"] >= 4:           # ~5s presionando -> no insistir
                                                    eng.update(state=None, cool=now + 8.0)
                                                    lg.write("DOOR-ENG abort (CROSS sin avance) -> logica normal 8s\n")
                                                    if meta2 is not None and hasattr(meta2, "door_result"):
                                                        try:
                                                            meta2.door_result(False, ncol - eng.get("c0n", ncol))
                                                        except Exception:
                                                            pass
                                            else:
                                                eng["wt"] = 0
                                            eng["px"] = x; eng["py"] = y; eng["pt"] = now
                    # --- MANIOBRA DE PUERTA: SOLO cuando el robot YA esta en zona estrecha (c0 bajo) y hay un
                    #     cuello MUY cerca por delante. Asi NO se activa al inicio (en abierto va con DWA rapido). ---
                    door = None
                    if engcmd is None and op and c0 < 0.9 and d_goal > DOOR_MIN_GOAL:   # cerca de B NO hay puerta: deja el DWA rodear
                        bc = 9.9; bi = -1
                        for i, p in enumerate(plan_pts):
                            dd = math.hypot(p[0] - x, p[1] - y)
                            if dd < 0.1 or dd > 0.9:        # cuello MUY cerca (no desde lejos)
                                continue
                            clr = min((p[0] - o[0]) ** 2 + (p[1] - o[1]) ** 2 for o in op) ** 0.5
                            if clr < bc:
                                bc = clr; bi = i
                        if bi >= 0 and bc < 0.42:           # vano estrecho justo delante = puerta
                            door = (bi, plan_pts[bi], bc)
                    if engcmd is not None:                    # el ENGAGEMENT manda sobre DOOR-*/DWA
                        cmd, ph = engcmd
                        nstop = 0
                    elif door is not None:
                        bi, dp, bc = door
                        a = plan_pts[max(0, bi - 2)]; b = plan_pts[min(len(plan_pts) - 1, bi + 2)]
                        ddir = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))   # eje de la puerta
                        he = (ddir - yaw + 180) % 360 - 180
                        # rumbo directo al PUNTO del vano: si ya APUNTA al hueco, no hace falta alinear con el
                        # eje (el ddir del A* tiembla; run 145010: alineado a ge=-1.2 y DOOR-AL siguio girando
                        # hasta +124 -> huida. "Deberia entrar recto, estaba bien orientado" — Adrian).
                        bdp = math.degrees(math.atan2(dp[1] - y, dp[0] - x))
                        hep = (bdp - yaw + 180) % 360 - 180
                        # VISION manda en la puerta (el laser es ruidoso ahi): suelo despejado por delante?
                        vis_ok = (vis_center is not None and vis_center > 0.45 and (vis_nearrun or 0) < 8)
                        bal = m_cl - m_cr                   # >0 = mas libre a la IZQ ; <0 = mas libre a la DCHA
                        if abs(he) > 25 and abs(hep) > 15:  # 1) alinea SOLO si tampoco apunta ya al hueco.
                            # Banda ANCHA (25, no 12): el eje de puerta 'ddir' tiembla porque el A* replanifica
                            # cada tick con el laser ruidoso; con banda estrecha + giro fijo 0.45 (no se puede
                            # bajar: hay deadzone ~0.3) el robot oscilaba sin parar (thrash). 25 lo tolera y deja
                            # de cazar el ruido; el centrado fino lo hace DOOR-CTR (strafe).
                            # PULSO anti-sobregiro: el giro real es ~15-20 grados/tick (deadzone: rx fijo 0.45);
                            # cerca de alineado (|he|<45) se gira en ticks ALTERNOS (mitad de ritmo efectivo)
                            # para no pasarse de largo y entrar en oscilacion (145010/145306).
                            if abs(he) < 45 and (int(now * 3.33) % 2 == 0):
                                cmd = (0, 0, 0, 0); ph = "DOOR-AL."
                            else:
                                cmd = (0, 0, -g.AV_TURN if he > 0 else g.AV_TURN, 0); ph = "DOOR-AL"
                        elif DOOR_CENTER and abs(bal) > DOOR_BAL_TH and max(cl_left, cl_right) > 0.30:
                            # 2) DESCENTRADO -> strafe hacia el lado MAS LIBRE para entrar centrado (Renxi)
                            lx = DOOR_STRAFE_SIGN * (DOOR_STRAFE if bal > 0 else -DOOR_STRAFE)
                            cmd = (lx, 0, 0, 0); ph = "DOOR-CTR"
                        elif c0 > AGGR_ROBOT_R or vis_ok:   # 3) entra RECTO si el LASER o la VISION lo ven despejado
                            cmd = (0, 0.28, 0, 0); ph = "DOOR-GO" + ("v" if vis_ok and c0 <= AGGR_ROBOT_R else "")
                        else:
                            cmd = (0, 0, 0, 0); ph = "DOOR-WT"   # ni laser ni vision: espera (no fuerza)
                        nstop = 0
                    else:
                        _, lyc, rxc, _, lbl = g.dwa_step(x, y, yaw, carrot, op)
                        cmd = (0, lyc, rxc, 0); ph = lbl
                        if lyc == 0 and rxc == 0:
                            nstop += 1
                            if nstop > 12:                  # ~1.2s encajonado -> desatasco
                                cl = clear_dir(x, y, yaw, +55, op); cr = clear_dir(x, y, yaw, -55, op)
                                brk = {"ph": "BACK", "t0": now, "dir": -g.AV_TURN if cl >= cr else g.AV_TURN}
                                nstop = 0; plan_pts = []
                        else:
                            nstop = 0
                else:
                    # sin ruta A* -> orienta al goal y avanza si el frente esta despejado (busqueda simple)
                    bg = math.degrees(math.atan2(wy - y, wx - x))
                    be = (bg - yaw + 180) % 360 - 180
                    if abs(be) > 20:
                        cmd = (0, 0, -g.AV_TURN if be > 0 else g.AV_TURN, 0); ph = "SEEK-T"
                    elif c0 > g.EXP_FWD_MIN:
                        cmd = (0, g.FWD_SPEED, 0, 0); ph = "SEEK-F"
                    else:
                        nstop += 1; cmd = (0, 0, 0, 0); ph = "SEEK-S"
                        if nstop > 12:
                            cl = clear_dir(x, y, yaw, +55, op); cr = clear_dir(x, y, yaw, -55, op)
                            brk = {"ph": "BACK", "t0": now, "dir": -g.AV_TURN if cl >= cr else g.AV_TURN}
                            nstop = 0

            # --- DIAGNOSTICO: rumbos a objetivo y carrot ---
            bg = math.degrees(math.atan2(wy - y, wx - x))
            beg = (bg - yaw + 180) % 360 - 180                          # error de rumbo al OBJETIVO
            bce = None
            if carrot is not None:
                bc = math.degrees(math.atan2(carrot[1] - y, carrot[0] - x))
                bce = (bc - yaw + 180) % 360 - 180                      # error de rumbo al CARROT
            # --- DIAGNOSTICO: CALIBRACION DE GIRO (signo) — clave del "da mil vueltas" ---
            # compara el giro REAL medido (dyaw/dt) con el que el comando ANTERIOR deberia producir.
            # modelo del DWA: wz=-1.8*rx (rad/s) -> deg/s = -103*rx. Si el signo medido != esperado -> el
            # robot gira al REVES que el modelo -> nunca converge -> vueltas infinitas.
            if prev_lt is not None and prev_yaw is not None:
                dt = now - prev_lt
                if dt > 0.01:
                    dyaw = (yaw - prev_yaw + 180) % 360 - 180
                    yawrate = dyaw / dt
                    rxp = prev_cmd[2]; lyp = prev_cmd[1]
                    if abs(rxp) > 0.1 and abs(lyp) < 0.05:             # giro puro previo
                        exp = -103.0 * rxp                            # deg/s esperado por el modelo
                        ok = (yawrate * exp > 0) or abs(yawrate) < 5
                        turncal.append((rxp, yawrate))
                        lg.write(f"  TURN-CAL rx={rxp:+.2f} esperado={exp:+.0f}deg/s medido={yawrate:+.0f}deg/s "
                                 f"{'OK' if ok else '>>> SIGNO INVERTIDO <<<'}\n")
                    # STRAFE-CAL: paso lateral puro previo -> desplazamiento REAL sobre el eje izquierdo.
                    # (asi cazamos el signo invertido del strafe EN VIVO, como TURN-CAL con el giro)
                    lxp = prev_cmd[0]
                    if abs(lxp) > 0.2 and abs(prev_cmd[1]) < 0.05 and abs(rxp) < 0.05 and prev_xy is not None:
                        yr0 = math.radians(prev_yaw)
                        dl = (x - prev_xy[0]) * (-math.sin(yr0)) + (y - prev_xy[1]) * math.cos(yr0)
                        strafecal.append((lxp, dl))
                        if abs(dl) > 0.008:
                            lg.write(f"  STRAFE-CAL lx={lxp:+.2f} medido={dl*100:+.1f}cm/tick "
                                     f"{'IZQ' if dl > 0 else 'DCHA'}\n")
                    spin_acc += abs(dyaw)
            prev_yaw = yaw; prev_lt = now; prev_cmd = cmd; prev_xy = (x, y)
            # progreso real (desplazamiento): si avanza, resetea el acumulador de giro
            if prog_pos is None or math.hypot(x - prog_pos[0], y - prog_pos[1]) > 0.15:
                prog_pos = (x, y); prog_t = now; spin_acc = 0.0
            phcount[ph.strip()] = phcount.get(ph.strip(), 0) + 1
            if spin_acc > 540 and now - prog_t > 4.0:                  # >1.5 vueltas sin avanzar 15cm
                lg.write(f"  SPIN!! girado {spin_acc:.0f}deg sin avanzar en {now-prog_t:.0f}s | yaw={yaw:+.0f} "
                         f"goal_err={beg:+.0f} carrot_err={(bce if bce is not None else 0):+.0f} c0={c0:.2f} "
                         f"plan={len(plan_pts)} fases={phcount}\n"); lg.flush()
                spin_acc = 0.0

            if aggressive:
                ph = "AGR-" + ph.strip()
            line = (f"t={now-t0:5.1f} {ph} pos=({x:+.2f},{y:+.2f}) yaw={yaw:+6.1f} d={d_goal:.2f} "
                    f"goal_err={beg:+.0f} carrot_err={(bce if bce is not None else 0):+4.0f} "
                    f"c0={c0:.2f} clear={m_clear:.2f} prog={m_prog:.2f} rel={m_rel:.2f} bal={m_cl-m_cr:+.2f} "
                    f"obs={len(oset)} hard={len(hard_set)} obsc={len(oscore)} yr={yaw_rate:3.0f}{'G' if turning_fast else ' '} "
                    f"nz={ss2['laser_noise']:.2f} flt={filt_rej:.2f} dmap=+{tick_add}/-{tick_del} "
                    f"shz={((len(fresh_times)-1)/max(1e-6, fresh_times[-1]-fresh_times[0])) if len(fresh_times) >= 2 else 0.0:.1f} "
                    f"plan={len(plan_pts)} cmd=(lx={cmd[0]:+.2f},ly={cmd[1]:+.2f},rx={cmd[2]:+.2f})")
            lg.write(line + "\n"); lg.flush()
            if now - health_t > 1.0:
                hh = read_telemetry(cdp); health_t = now
                rd.telem(now - t0, _telem_row(hh))
            # auto-evaluacion de sensado SOLO con barrido FRESCO: un buffer repetido tiene churn=0 y conteo
            # estable -> el laser_noise saldria artificialmente BAJO justo cuando los datos son peores
            # (nube congelada). En ticks stale se reusa el ultimo valor. reloc_jump no se pierde (pend_jump).
            if scan_fresh:
                loc = match_score(live, refmap)
                ss2 = sens.update(now - t0, live, c0, loc, pend_jump); pend_jump = False
                nz = ss2["laser_noise"]; nz_sum += nz; nz_max = max(nz_max, nz); nz_n += 1
            m_rel = ss2["reliability"]
            h = hh.get("h") or {}
            # --- META2: decision de gobernanza DCE a ~2Hz con las metricas del tick ---
            if meta2 is not None:
                try:
                    # parada COMANDADA (alinear/girar/strafe del engagement o de DOOR): progression=0
                    # es voluntaria -> el bridge la congela para no reportar estancamiento falso
                    _php = ph.strip().replace("AGR-", "").rstrip("!HM")
                    _hold = _php.startswith(("ENG-T", "ENG-AL", "ENG-RE", "ENG-WT", "ENG-C", "ENG-CG",
                                             "DOOR-AL", "DOOR-WT", "DOOR-CTR", "DWA-T", "SEEK-T"))
                    # canal de RESISTENCIA: velocidad real vs comandada en ~1.2s (None sin comando de avance)
                    m2trk.append((now, x, y, (cmd[1] if cmd else 0.0)))
                    while m2trk and now - m2trk[0][0] > 1.2:
                        m2trk.popleft()
                    _mob = None
                    if len(m2trk) >= 3:
                        _dtm = now - m2trk[0][0]
                        _cmy = sum(c4 for _, _, _, c4 in m2trk) / len(m2trk)
                        if _cmy >= 0.2 and _dtm > 0.5:
                            _v = math.hypot(x - m2trk[0][1], y - m2trk[0][2]) / _dtm
                            _mob = max(0.0, min(1.0, _v / max(0.05, 0.75 * _cmy)))
                    _sp_dt = ((time.time() - rd.last_spill_t)
                              if getattr(rd, "last_spill_t", None) else None)
                    _o = meta2.tick(now, m_clear, m_prog, m_rel, ss2.get("laser_noise"), h.get("bat"),
                                    hold_progression=_hold, mobility=_mob,
                                    spill_dt=_sp_dt, spill_count=getattr(rd, "spill_count", 0),
                                    spd=_v, wz=prev_cmd[2])
                except Exception as e:
                    _o = None
                    if now - vis_log_t > 10.0:
                        lg.write(f"[META2] error: {repr(e)}\n")
                if _o is not None:
                    if _o.get("changed") or _o["action"] in ("SWITCH", "HELP"):
                        lg.write(f"[META2] {_o['action']} activo={_o['active']} tens={_o['tension']} "
                                 f"ful={_o['fulfillment']} cap={_o['cap']} rej={_o['rejections']}\n")
                    if _o.get("changed") and _o["action"] not in ("KEEP", "WARMUP", "PEND"):
                        rd.event("meta2_" + _o["action"].lower().rstrip("?"), now - t0, x, y,
                                 {"active": _o["active"], "tension": _o["tension"], "ful": _o["fulfillment"]})
                    m2o = _o
                    if not m2_mode_logged:                       # el dataset se autodescribe (¿shadow o activo?)
                        rd.event("meta2_mode", now - t0, x, y, {"mode": META2_MODE}); m2_mode_logged = True
                    # --- ESCALADA: la experiencia sostenida de "nada sirve" debe terminar en ABORT ---
                    if META2_ABORT:
                        bad = str(_o["action"]) in ("FALLBACK", "HELP")      # solo decisiones FIRMES (sin '?')
                        m2win.append((now, 1 if bad else 0, d_goal))
                        while m2win and now - m2win[0][0] > M2_ABORT_WIN:
                            m2win.popleft()
                        if str(_o["action"]) == "HELP":
                            m2_help_t0 = m2_help_t0 or now
                        else:
                            m2_help_t0 = None
                        span = now - m2win[0][0] if m2win else 0.0
                        badf = (sum(b for _, b, _ in m2win) / len(m2win)) if m2win else 0.0
                        prog = (m2win[0][2] - d_goal) if m2win else 9.9
                        help_sus = (m2_help_t0 is not None and now - m2_help_t0 >= M2_HELP_S)
                        window_bad = (span >= M2_ABORT_WIN * 0.95 and badf >= M2_ABORT_BAD
                                      and prog < M2_ABORT_PROG)
                        if help_sus or window_bad:
                            why = (f"HELP continuo {now - (m2_help_t0 or now):.0f}s" if help_sus else
                                   f"experiencia mala {badf*100:.0f}% en {span:.0f}s con progreso {prog:+.2f}m")
                            if META2_MODE == "2":
                                cdp.eval(g.STOP_JS); time.sleep(0.2); cdp.eval(g.STOP_JS)
                                print(f"\n  >>> META2 EXPERIENCE-ABORT: {why}. Ninguna analogia es valida aqui:"
                                      f" STOP y aborto (pide ayuda/reposiciona). G1_M2_ABORT=0 desactiva.")
                                lg.write(f"META2-EXPERIENCE-ABORT {why} pos=({x:+.2f},{y:+.2f}) "
                                         f"{time.strftime('%Y-%m-%d %H:%M:%S')}\n"); lg.flush()
                                rd.event("meta2_experience_abort", now - t0, x, y,
                                         {"why": why, "badf": round(badf, 2), "prog": round(prog, 2)})
                                rd.finish("aborted_meta2_help",
                                          {"time_s": round(now - t0, 2), "path_m": round(_path_len(trail), 2),
                                           "collisions": ncol, "c0min": round(minc0, 2), **diag_summary()})
                                return False
                            elif now - m2_warn_t > 60.0:         # SHADOW: avisar (una vez/min) sin actuar
                                m2_warn_t = now
                                lg.write(f"META2-ABORT-SHADOW (aqui habria abortado: {why})\n")
                                rd.event("meta2_abort_shadow", now - t0, x, y,
                                         {"why": why, "badf": round(badf, 2), "prog": round(prog, 2)})
                                m2win.clear(); m2_help_t0 = None   # re-arma la ventana para el siguiente aviso
            # --- FEEDBACK RENXI: canal humano + validez retrospectiva del laser ---
            _hct = getattr(rd, "_h_col_t", 0.0)
            if _hct and _hct > h_col_seen:
                h_col_seen = _hct
                ncol += 1                              # el reporte humano ES una colision: alimenta
                lg.write(f"COLISION reportada por HUMANO t={now - t0:.0f}s pos=({x:+.2f},{y:+.2f})\n")
            if ncol > lt_prev_ncol:
                # colision (IMU o humana) con el laser diciendo "libre" el tick anterior ->
                # el laser MINTIO: penalizar su validez (Renxi: "use the past data to
                # calculate the validity of the laser reading"). Recupera despacio.
                if c0_prev > 0.6:
                    laser_trust = max(0.2, laser_trust - 0.25)
                    rd.event("laser_lied", now - t0, x, y,
                             extra={"c0_prev": round(c0_prev, 2), "trust": round(laser_trust, 2)})
                    lg.write(f"LASER-VALIDEZ: colision con c0_prev={c0_prev:.2f} -> trust={laser_trust:.2f}\n")
            lt_prev_ncol = ncol
            laser_trust = min(1.0, laser_trust + 0.001)
            c0_prev = c0
            # recall de asistencias humanas pasadas cerca de aqui (memoria episodica)
            if assist_mem and now - assist_recall_t > 30.0:
                for _a in assist_mem:
                    if math.hypot(x - _a.get("x", 99), y - _a.get("y", 99)) < 0.5:
                        assist_recall_t = now
                        rd.event("assist_recall", now - t0, x, y,
                                 extra={"cm": _a.get("cm"), "cuando": _a.get("when")})
                        lg.write(f"[HUMANO] RECUERDO: aqui me movieron {_a.get('cm')}cm ({_a.get('when')})\n")
                        print(f"  [HUMANO] recuerdo episodico: en esta zona me asistieron ({_a.get('cm')} cm)")
                        break
            rd.sample(now - t0, x, y, yaw, d_goal, math.hypot(x - prog_pos[0], y - prog_pos[1]) if prog_pos else 0.0,
                      c0, len(oset), cmd=cmd, phase=ph.strip(),
                      extra={"err": hh.get("err"), "bat": h.get("bat"), "cpuT": h.get("cpuT"),
                             "merr": h.get("merr"), "loc_match": loc,
                             "clearance": mm["clearance"], "clearance_m": mm["clearance_m"],
                             "progression": mm["progression"], "progress_rate": mm["progress_rate"],
                             "reliability": ss2["reliability"], "laser_noise": ss2["laser_noise"],
                             "loc_conf": ss2["loc_conf"], "c0_std": ss2["c0_std"],
                             "scan_churn": ss2["scan_churn"], "reloc_rate10s": ss2["reloc_rate10s"],
                             "filt_rej": round(filt_rej, 3),                  # fraccion del barrido rechazada por persistencia
                             "scan_fresh": bool(scan_fresh),                  # este tick trajo nube nueva?
                             "map_add": tick_add, "map_del": tick_del,        # churn del mapa activo este tick
                             "color_pts": perc_raw.get("color_pts"),          # puntos que aporto el canal de MOQUETA
                             "carpet_pct": perc_raw.get("carpet_pct"),        # fraccion del frame clasificada moqueta
                             "color_near": perc_raw.get("color_near"),        # puntos clampeados (obstaculo encima)
                             "color_rmin": perc_raw.get("color_rmin"),        # obstaculo de color mas cercano (m)
                             "door_b": (perc_raw.get("door") or {}).get("bearing_deg"),   # rumbo de puerta por vision
                             "illum_b": (round(illum_ema, 1) if illum_ema is not None else None),  # luma EMA del frame (gate iluminacion)
                             "dvis_gate": (dvis_gate if DOOR_VIS_GATE else None),  # 1 = representante de vision suprimido
                             "carrot": ([round(carrot[0], 2), round(carrot[1], 2)] if carrot else None),
                             "goal_err": round(beg, 1),                       # error de rumbo al objetivo
                             "carrot_err": (round(bce, 1) if bce is not None else None),   # y al carrot del plan
                             "plan_n": len(plan_pts),                         # 0 = A* sin ruta ese tick
                             "c0_hard": round(c0_hard, 2),                    # holgura frontal contra PAREDES/persistentes
                             "n_hard": len(hard_set),                         # celdas de alta confianza en el mapa
                             "perc_n": len(perc_cells),                       # nº de celdas-obstaculo que aporto la VISION este tick
                             "perc_age": (round(now - perc_rx_t, 2) if perc_rx_t is not None else None),   # s desde la ultima respuesta REAL del server (P3: ¿flicker=latencia o escena vacia?)
                             "meta2_act": (m2o or {}).get("action"),           # gobernanza DCE (G1_META2)
                             "meta2_cap": (m2o or {}).get("cap"),              # techo vigente (None=sin techo); en modo 2 se aplica
                             "meta2_rho": (m2o or {}).get("rho"),              # rho_DCA runtime: margen de arbitraje / presupuesto de perturbacion
                             "meta2_env": (m2o or {}).get("env"),              # Layer 3: escala de relevancia de entorno (G1_M2_L3)
                             "meta2_unc": (m2o or {}).get("unc"),              # incertidumbre DST por parametro (dispersion empirica -> intervalos)
                             "meta2_pf": (m2o or {}).get("pf"),                # posterior del PF de regimen (SOMBRA): tapa/llenado/sensado/bateria/atasco
                             "sent": [round(last_sent[1], 3), round(last_sent[2], 3)],   # lo ENVIADO el tick anterior (post-guardas/rampa; 'cmd' es pre-guardas)
                             "phase_sent": (last_ph_sent or None),             # fase ACTUADA el tick anterior, con marcadores de guardia (autoridad; W4)
                             "laser_trust": round(laser_trust, 2),             # validez retrospectiva del laser (Renxi)
                             "cov_def": cov_def,                               # cobertura: predichos por el mapa y AUSENTES
                             "cov_blind": cov_blind,                           # ...y los inobservables por el recorte del fabricante
                             "cov_n": cov_n,                                   # rumbos que informan (0 = sin mapa)
                             "vox_inj": (vox_inj if VOXMEM else None),   # celdas sostenidas por memoria
                             "vox_ray": (vox_ray if VOXMEM else None),   # celdas soltadas por rayo este tick
                             "cov_missing": cov_missing,                       # celdas de la ref de SESION ausentes 2 barridos (None = sin G1_COVREF)
                             "meta_state": (meta_state if METASM else None),   # estado de la MAQUINA META
                             "iface_q": (iface_q if METASM else None),         # calidad de la interfaz (Renxi)
                             "door_contra": sum(1 for t_ in dc_contra if now - t_ <= 10.0),   # contradicciones del centro de puerta en 10s
                             "meta2_active": (m2o or {}).get("active"),
                             "meta2_tens": (m2o or {}).get("tension"),
                             "meta2_ful": (m2o or {}).get("fulfillment"),
                             "clear_left": round(m_cl, 3), "clear_right": round(m_cr, 3),   # clearance lateral (Renxi: balance)
                             "clearL_m": round(cl_left, 2), "clearR_m": round(cl_right, 2),
                             "balance": round(m_cl - m_cr, 3),                # +izq libre / -dcha libre (0 = centrado)
                             "dets": ([[d.get("label"), round(d.get("conf", 0), 2),
                                        d.get("bearing_deg"), d.get("range_m")] for d in perc_dets] or None)})
            rd.maybe_laser(now - t0, op, ctx={"x": round(x, 3), "y": round(y, 3),
                                              "yaw": round(yaw, 1),
                                              "fwd": round(prev_cmd[1], 2), "wz": round(prev_cmd[2], 2),
                                              "fresh": bool(scan_fresh)})
            # CRUCE GEOMETRICO (auditoria 22-jul): 2 de 6 cruces reales ocurrieron FUERA del
            # FSM (sin door_crossed). Evento pasivo por cambio de lado del plano del vano —
            # solo registro (la biblioteca sigue aprendiendo solo de cruces del FSM, v1).
            _gdc = math.hypot(x - DOOR_CX, y - DOOR_CY)
            if _gdc < 1.4:
                _gux = math.cos(math.radians(DOOR_AXIS)); _guy = math.sin(math.radians(DOOR_AXIS))
                _gs = 1 if ((x - DOOR_CX) * _gux + (y - DOOR_CY) * _guy) > 0 else -1
                if door_side is not None and _gs != door_side and now - door_geom_t > 5.0:
                    door_geom_t = now
                    rd.event("door_crossed_geom", now - t0, x, y, None)
                door_side = _gs
            else:
                door_side = None
            if now - tprint > 0.4:
                print("  " + line); tprint = now

            # --- PLAN GLOBAL origen->destino (para verlo completo en la ventana) ---
            # OJO (2026-07-02, cazado en la primera run de P8b): esta linea verde usaba SIEMPRE el laser
            # vivo (oset) via global_plan() "solo para visualizar" -> zigzagueaba entre manchas aunque el
            # plan de CONTROL ya fuera estatico, y parecia que el fix no funcionaba. Ahora en modo 'static'
            # pinta el plan sobre el MISMO costmap que el control (paredes inf + muebles blandos).
            if vshare is not None and now - gplan_t > 3.0 and trail:
                if GLOBAL_SRC == "static":
                    gcm = global_static_costmap(refmap or set(), staticmap | hard_set)
                    gcells = g.astar((round(trail[0][0] / g.OCELL), round(trail[0][1] / g.OCELL)),
                                     (round(wx / g.OCELL), round(wy / g.OCELL)), gcm, margin=25)
                    gplan = ([(c[0] * g.OCELL, c[1] * g.OCELL) for c in gcells]
                             if gcells and len(gcells) > 1 else [(trail[0][0], trail[0][1]), (wx, wy)])
                else:
                    gplan = global_plan(trail[0][0], trail[0][1], wx, wy, oset or refmap)
                gplan_t = now
            # --- publica estado para la ventana en vivo ---
            if vshare is not None:
                if now - cam_t > 0.5:
                    cam_jpg = grab_cam(cdp); cam_t = now
                with lock:
                    vshare["x"] = x; vshare["y"] = y; vshare["yaw"] = yaw; vshare["ph"] = ph
                    vshare["d"] = d_goal; vshare["col"] = ncol; vshare["t"] = now - t0
                    vshare["goal"] = (wx, wy); vshare["carrot"] = carrot; vshare["cam"] = cam_jpg
                    vshare["obs"] = [(cx * g.OCELL, cy * g.OCELL) for (cx, cy) in oset]      # mapa acumulado (m)
                    vshare["laser"] = [(cx * g.OCELL, cy * g.OCELL) for (cx, cy) in live]    # barrido en vivo (m)
                    vshare["plan"] = list(plan_pts)                                          # ruta A* local (m)
                    vshare["gplan"] = list(gplan)                                            # PLAN GLOBAL origen->destino
                    vshare["trail"] = list(trail)                                            # odometria recorrida
                    vshare["clear"] = m_clear; vshare["prog"] = m_prog; vshare["rel"] = m_rel  # 3 metricas (valor actual)
                    vshare["mhist"] = sei.history()                                          # historia (t,clearance,progression)
                    vshare["shist"] = sens.history()                                         # historia (t,reliability,noise,loc_conf)

            # --- MODERADOR por vision (principio RENXI: LiDAR decide, vision APOYA): si el canal de
            # color clampea muchas columnas ('lo tengo encima'), NO vetamos el paso (eso hacia huir al
            # robot de puertas pasables) pero SI limitamos la velocidad de avance: acercarse con cuidado.
            if cmd[1] > 0.24 and isinstance(perc_raw.get("color_near"), (int, float)) and perc_raw["color_near"] >= 8:
                cmd = (cmd[0], 0.24, cmd[2], 0)
            # --- RETREAT v2 POR MIGAS (portado de main; ver consts) ---
            if RETREAT:
                stk_hist.append((now, x, y))
                while stk_hist and now - stk_hist[0][0] > 4.5:
                    stk_hist.pop(0)
                if retreat is None and (not crumbs or math.hypot(x - crumbs[-1][0], y - crumbs[-1][1]) >= 0.15):
                    crumbs.append((x, y)); del crumbs[:-30]
                if ncol > rt_prev_ncol:
                    rt_col_ts.append(now); del rt_col_ts[:-6]
                if retreat is None and now > retreat_cool and len(crumbs) >= 3:
                    _ncol_20s = sum(1 for t_ in rt_col_ts if now - t_ <= 20.0)
                    _old = next((p for p in stk_hist if now - p[0] >= 3.8), None)
                    _stuck = (_old is not None and math.hypot(x - _old[1], y - _old[2]) < 0.08
                              and c0 < 0.42)
                    _colhit = (_ncol_20s >= 2) or (ncol > rt_prev_ncol and _stuck)
                    _span = math.hypot(x - crumbs[0][0], y - crumbs[0][1])
                    _near_start = (EXIT_RT and exit_p0 is not None
                                   and math.hypot(x - exit_p0[0], y - exit_p0[1]) < 1.5)
                    _span_min = 0.15 if _near_start else 0.5
                    if (_colhit or _stuck) and (now - t0) > 12.0 and _span >= _span_min and rt_count < 3:
                        retreat = {"trail": list(reversed(crumbs[:-1])), "d": 0.0,
                                   "lx": x, "ly": y, "t0": now}
                        rt_count += 1
                        eng.update(state=None, cool=now + 10.0)
                        lg.write(f"RETREAT start ({'colision' if _colhit else 'atasco'}, c0={c0:.2f}) "
                                 f"pos=({x:+.2f},{y:+.2f}) t={now - t0:.0f}s n={rt_count}/3\n")
                        rd.event("retreat_start", now - t0, x, y,
                                 {"por": "col" if _colhit else "atasco", "c0": round(c0, 2), "n": rt_count})
                rt_prev_ncol = ncol
                if retreat is not None:
                    m2_help_t0 = None                    # la retirada PAUSA el reloj HELP del P9
                    tr = retreat["trail"]
                    while tr and math.hypot(tr[0][0] - x, tr[0][1] - y) < 0.22:
                        tr.pop(0)
                    retreat["d"] += math.hypot(x - retreat["lx"], y - retreat["ly"])
                    retreat["lx"], retreat["ly"] = x, y
                    if retreat["d"] >= RETREAT_D or not tr or now - retreat["t0"] > 14.0:
                        lg.write(f"RETREAT fin: deshecho {retreat['d']:.2f}m en {now - retreat['t0']:.1f}s\n")
                        rd.event("retreat_end", now - t0, x, y, {"d": round(retreat["d"], 2)})
                        retreat = None; retreat_cool = now + 6.0
                        crumbs = crumbs[:max(1, len(crumbs) - 6)]
                        m2win.clear(); m2_help_t0 = None
                    else:
                        _tx, _ty = tr[0]
                        _b = math.degrees(math.atan2(_ty - y, _tx - x))
                        _back = (_b - (yaw + 180.0) + 180.0) % 360.0 - 180.0
                        _rx = max(-0.18, min(0.18, -_back * 0.02))
                        cmd = (0.0, -RETREAT_V, _rx, 0)
                        ph = "RETREAT"
            # --- MAQUINA DE ESTADOS META (feedback Renxi; ver consts) ---
            if METASM:
                _dcn = sum(1 for t_ in dc_contra if now - t_ <= 10.0)
                _pfresh = None
                if perc_rx_t is not None:
                    _pfresh = 1.0 if (now - perc_rx_t) < 1.5 else 0.0
                _hb = None
                if getattr(rd, "_gt_hb_seen", False):
                    _hb = 1.0 if (time.time() - (rd._gt_hb_t or 0)) < 6.0 else 0.0
                _cdp_ok = 1.0 if cdp_lat < 0.20 else 0.0
                _comps = [c_ for c_ in (_pfresh, _hb, _cdp_ok) if c_ is not None]
                iface_q = round(sum(_comps) / len(_comps), 2) if _comps else 1.0
                _near_known = c0                     # distancia al obstaculo mas cercano (ya en metros)
                # asistencia humana recibida -> re-armar presupuestos y reanudar
                _hat = getattr(rd, "_h_assist_t", 0.0)
                if _hat and _hat > h_assist_seen:
                    h_assist_seen = _hat
                    rt_count = 0; rt_col_ts = []; retreat = None
                    stk_hist = []; crumbs = []           # borron y cuenta nueva: parado en ASSIST no es atasco
                    m2win.clear(); m2_help_t0 = None
                    lg.write(f"[HUMANO] asistencia RECIBIDA t={now - t0:.0f}s -> presupuestos re-armados, reanudando\n")
                _stuck_now = False
                _oldp = next((p for p in stk_hist if now - p[0] >= 3.8), None)
                if _oldp is not None:
                    _stuck_now = math.hypot(x - _oldp[1], y - _oldp[2]) < 0.10
                if retreat is not None:
                    _ms = "RECOVERY"
                elif rt_count >= 3 and (_stuck_now or c0 < 0.45):
                    _ms = "ASSIST"
                elif laser_trust < 0.6 or _dcn >= 3 or iface_q < 0.5:
                    _ms = "DEGRADED"
                elif _near_known < 1.0 or now < retreat_cool + 4.0:
                    _ms = "BLIND"           # zona ciega O cautela post-recuperacion (~10s tras retirada)
                else:
                    _ms = "NORMAL"
                if _ms != meta_state and now - ms_ev_t > 1.0:
                    ms_ev_t = now
                    rd.event("meta_state", now - t0, x, y, {"de": meta_state, "a": _ms})
                    lg.write(f"META-SM: {meta_state} -> {_ms} (trust={laser_trust:.2f} contra={_dcn} "
                             f"iface={iface_q:.2f} near={_near_known:.2f}) t={now - t0:.0f}s\n")
                meta_state = _ms
                # actuacion conservadora por estado (solo QUITA velocidad, jamas anade)
                if _ms == "DEGRADED" and cmd[1] > 0.24:
                    cmd = (cmd[0], 0.24, cmd[2], 0); ph = ph.strip() + "!S"
                elif _ms == "BLIND" and cmd[1] > 0.28:
                    cmd = (cmd[0], 0.28, cmd[2], 0); ph = ph.strip() + "!S"
                elif _ms == "ASSIST":
                    cmd = (0.0, 0.0, 0.0, 0); ph = "ASSIST"
                    if now - ms_ev_t > 8.0 and int(now) % 10 == 0:
                        print("  [META-SM] ASISTENCIA: parado esperando al humano ('m <cm>' en el marcador)")
            if exit_p0 is None:
                exit_p0 = (x, y)                     # punto de arranque (lo usa RETREAT-en-salida)
            # --- HARD-GUARD (G1_HARDGUARD=1): las paredes/persistentes NO se rozan ni en agresivo.
            # Frena el avance segun la holgura contra lo DURO; lo blando/ruidoso sigue negociable.
            if HARD_GUARD and cmd[1] > 0.05:
                if c0_hard < HARD_STOP:
                    cmd = (cmd[0], 0.0, cmd[2], 0); ph = ph.strip() + "!H"
                    if now - hg_log_t > 2.0:
                        lg.write(f"HARD-GUARD STOP c0_hard={c0_hard:.2f} (<{HARD_STOP}) pos=({x:+.2f},{y:+.2f})\n")
                        hg_log_t = now
                elif c0_hard < HARD_SLOW and cmd[1] > 0.22:
                    cmd = (cmd[0], 0.22, cmd[2], 0)            # acercarse a una pared: despacio
            # --- FIX B: FRENO POR CAMARA (ver cabecera). Solo-camara: el laser dice libre pero
            # el canal clamp lleva CB_TICKS ticks gritando "lo tengo encima" (mueble bajo el
            # plano del laser: brazo de sofa, asiento). Para el avance; DWA sigue girando y las
            # recuperaciones (retroceso) no se tocan.
            if COLOR_BRAKE and cmd[1] > 0.05 and math.hypot(x - DOOR_CX, y - DOOR_CY) > CB_DOOR_R:
                _ccn = (perc_raw.get("color_near") or 0) if isinstance(perc_raw, dict) else 0
                _cfresh = (perc_rx_t is not None) and (now - perc_rx_t) < 1.5
                if _cfresh and c0 > CB_C0 and _ccn >= CB_NPTS:
                    cb_hits += 1
                else:
                    cb_hits = 0; cb_on = False
                if cb_hits >= CB_TICKS and _ccn >= CB_STOP:
                    cmd = (cmd[0], 0.0, cmd[2], 0); ph = ph.strip() + "!C"
                    if not cb_on:
                        cb_on = True
                        lg.write(f"COLOR-BRAKE STOP color_near={_ccn} c0={c0:.2f} "
                                 f"pos=({x:+.2f},{y:+.2f}) t={now - t0:.0f}s\n")
                        rd.event("color_brake", now - t0, x, y, {"near": _ccn, "c0": round(c0, 2)})
                elif cb_hits >= CB_TICKS and cmd[1] > CB_CREEP:
                    cmd = (cmd[0], CB_CREEP, cmd[2], 0); ph = ph.strip() + "!c"
                    if not cb_on:
                        cb_on = True
                        lg.write(f"COLOR-BRAKE CREEP color_near={_ccn} c0={c0:.2f} "
                                 f"pos=({x:+.2f},{y:+.2f}) t={now - t0:.0f}s\n")
                        rd.event("color_creep", now - t0, x, y, {"near": _ccn, "c0": round(c0, 2)})
            # --- SUELO DE ZONA-VANO (G1_DOORGUARD; ver cabecera): determinista, bajo el
            # razonador. Cubre el agujero verificado: cruce via DWA puro tras un abort del
            # FSM / A*-fail / fragilidad dormida. Las fases de negociacion quedan exentas.
            if DOORGUARD and math.hypot(x - DOOR_CX, y - DOOR_CY) < DOORGUARD_R \
                    and not any(k in ph for k in ("ENG", "AGR", "ESC", "R-", "BRK")):
                _dg_hit = False
                if cmd[1] > 0.28:
                    cmd = (cmd[0], 0.28, cmd[2], 0); _dg_hit = True
                if abs(cmd[2]) > 0.38:
                    cmd = (cmd[0], cmd[1], math.copysign(0.38, cmd[2]), 0); _dg_hit = True
                if _dg_hit:
                    ph = ph.strip() + "!D"
                    if not dg_on:
                        dg_on = True
                        lg.write(f"DOOR-GUARD techo de zona-vano t={now - t0:.0f}s pos=({x:+.2f},{y:+.2f})\n")
                        rd.event("door_guard", now - t0, x, y, None)
            elif dg_on and math.hypot(x - DOOR_CX, y - DOOR_CY) > DOORGUARD_R + 0.3:
                dg_on = False
            # --- META2 ACTIVO (G1_META2=2): el perfil de la analogia DCE es un TECHO de avance.
            # Solo modera cmd[1]>0 (como HARD-GUARD y el moderador Renxi): retroceso/giros de las
            # recuperaciones y del ESCAPE no se tocan. HELP firme -> avance 0 (el DCE pide parar).
            if META2_MODE == "2" and m2o is not None and m2o.get("cap") is not None and cmd[1] > m2o["cap"]:
                cmd = (cmd[0], m2o["cap"], cmd[2], 0); ph = ph.strip() + "!M"
                m2_ncap += 1
                if not m2_cap_on:                              # 3) flanco: evento + log = prueba de ACTUACION real
                    m2_cap_on = True
                    lg.write(f"META2-CAP ON cap={m2o['cap']} analogia={m2o['active']} "
                             f"act={m2o['action']} t={now - t0:.0f}s\n")
                    rd.event("meta2_cap_on", now - t0, x, y, {"cap": m2o["cap"], "active": m2o["active"]})
            else:
                m2_cap_on = False
            # --- G1_VCAP: techo de avance FIJO sin gobernanza (el M1 'simbolico' de la tesis
            # Cap.5: politica conservadora rigida, sin reasoner, sin vetos, sin abortos).
            # Para el brazo M1 de la campana payload: G1_META2=0 G1_VCAP=0.28.
            if VCAP is not None and cmd[1] > VCAP:
                cmd = (cmd[0], VCAP, cmd[2], 0)
            # --- EXTENSION B (rama analogy-profiles): techo de GIRO del perfil de analogia.
            # Los giros de arranque dominan el derrame con taza llena (sesion real 2026-07-08)
            # y no estaban gobernados. Solo bucle normal (recuperaciones intactas, como el cap).
            # (auditoria 22-jul) el techo de giro clampaba TAMBIEN recovery/desatasco/FSM,
            # al reves de su proposito: gobierna el giro SOSTENIDO de crucero. Solo DWA/SEEK.
            if META2_MODE == "2" and m2o is not None and m2o.get("turn") \
                    and ph.strip().startswith(("DWA", "SEEK")):
                if abs(cmd[2]) > m2o["turn"]:
                    cmd = (cmd[0], cmd[1], math.copysign(m2o["turn"], cmd[2]), 0)
            # --- EXT E: jerk limitado (ver cabecera). ULTIMO de la cadena: rampa solo al
            # ACELERAR o INVERTIR; cualquier reduccion hacia cero (guardas/HELP/paradas)
            # pasa sin tocar. prev_cmd = lo realmente ENVIADO el tick anterior.
            # Exenciones (smoke 171335: 5 colisiones en la APROXIMACION de la puerta con la
            # rampa global — el alineado necesita correcciones rapidas): sin rampa a <2m del
            # vano ni en fases de recuperacion/escape/agresivo. Suavidad en crucero (donde
            # vive la senal 5.7x de los escalones de avance); agilidad donde se negocia
            # geometria. El RTF del gemelo ademas dobla la rampa en tiempo-sim: alli es peor caso.
            _slew_ok = (SLEW_ON
                        and math.hypot(x - DOOR_CX, y - DOOR_CY) > 2.0
                        and not any(k in ph for k in ("ENG", "AGR", "R-", "ESC", "BRK")))
            if _slew_ok:
                _f, _w = cmd[1], cmd[2]
                if abs(_f) > abs(last_sent[1]) or _f * last_sent[1] < 0:
                    _f = max(last_sent[1] - SLEW_LIN, min(last_sent[1] + SLEW_LIN, _f))
                if abs(_w) > abs(last_sent[2]) or _w * last_sent[2] < 0:
                    _w = max(last_sent[2] - SLEW_ANG, min(last_sent[2] + SLEW_ANG, _w))
                if (_f, _w) != (cmd[1], cmd[2]):
                    cmd = (cmd[0], _f, _w, 0); ph = ph.strip() + "~"
            # --- OMNI-GUARD (ver cabecera): reflejo final, TODAS las fuentes, SIN exenciones ---
            if OMNIGUARD and op:
                _cy = math.cos(math.radians(yaw)); _sy = math.sin(math.radians(yaw))
                _near = []                              # obstaculos a <1.0m en frame ROBOT (x=alante, y=izq)
                for (_ox, _oy) in op:
                    _dx = _ox - x; _dy = _oy - y
                    if _dx * _dx + _dy * _dy > 1.0:
                        continue
                    _near.append((_dx * _cy + _dy * _sy, -_dx * _sy + _dy * _cy))
                if _near:
                    _hit = None
                    _f, _l = cmd[1], cmd[0]             # avance, lateral (stick: lx + = derecha)
                    if abs(_f) > 0.05 or abs(_l) > 0.05:
                        # v2 (cazado por el GEMELO, runs 180802/181151: el sector angular +-45
                        # bloqueaba la ENTRADA del vano — las jambas quedan delante-diagonal
                        # dentro del sector aunque el hueco central este libre; con pellizco
                        # 0.71 son 7cm/lado y ningun test angular pasa). Test de CORREDOR:
                        # bloquear solo si el obstaculo esta en la FRANJA que el cuerpo va a
                        # barrer (|lateral|<0.30 = semiancho 0.28 + margen) y a <OMNI_STOP por
                        # delante. Jamba a 0.355 de lado: pasa. Pared de frente a 0.30: bloquea.
                        _ma = math.atan2(-_l, _f) if (_f or _l) else 0.0
                        _cma = math.cos(_ma); _sma = math.sin(_ma)
                        _dmin = 9.9
                        for (_rx, _ry) in _near:
                            _along = _rx * _cma + _ry * _sma
                            _lat = -_rx * _sma + _ry * _cma
                            if 0.02 < _along < OMNI_STOP and abs(_lat) < 0.30:
                                _dmin = min(_dmin, _along)
                        if _dmin < OMNI_STOP:
                            cmd = (0.0, 0.0, cmd[2], 0); _hit = ("trans", _dmin)
                    if abs(cmd[2]) > 0.05:
                        _dc = min(math.hypot(_rx, _ry) for (_rx, _ry) in _near)
                        if _dc < OMNI_TOUCH:
                            cmd = (cmd[0], cmd[1], 0.0, 0); _hit = _hit or ("rot0", _dc)
                        elif _dc < OMNI_ROT and abs(cmd[2]) > 0.20:
                            cmd = (cmd[0], cmd[1], math.copysign(0.20, cmd[2]), 0)
                            _hit = _hit or ("rotslow", _dc)
                    if _hit:
                        ph = ph.strip() + "!O"
                        if now - omni_log_t > 3.0:
                            omni_log_t = now
                            lg.write(f"OMNI-GUARD {_hit[0]} d={_hit[1]:.2f}m fase={ph} pos=({x:+.2f},{y:+.2f})\n")
                            rd.event("omni_guard", now - t0, x, y, {"que": _hit[0], "d": round(_hit[1], 2)})
            prev_fwd = (cmd[1] > 0.1)
            _t_send = time.time()
            cdp.eval(g.set_cmd_js(*cmd))
            cdp_lat = time.time() - _t_send            # componente CDP de iface_q
            last_sent = cmd
            last_ph_sent = ph
            time.sleep(0.1)
        rd.finish("aborted", {"time_s": round(time.time() - t0, 2), "path_m": round(_path_len(trail), 2),
                              "collisions": ncol, "c0min": round(minc0, 2), **diag_summary()})
        return False                                  # salida por cierre de ventana (stop_event)
    except KeyboardInterrupt:
        print(f"\n  [ABORTADO '{label}']"); lg.write(f"ABORT {label} {time.strftime('%Y-%m-%d %H:%M:%S')}\n"); lg.flush()
        rd.finish("aborted", {"time_s": round(time.time() - t0, 2), "path_m": round(_path_len(trail), 2),
                              "collisions": ncol, "c0min": round(minc0, 2), **diag_summary()})
        return False
    finally:
        cdp.eval(g.STOP_JS); time.sleep(0.2); cdp.eval(g.STOP_JS)
        g.ROBOT_R = locals().get("ROBOT_R0", g.ROBOT_R)    # restaura la holgura normal del DWA
        try:
            sc = locals().get("strafecal") or []
            if sc:
                izq = [d for l, d in sc if l > 0]; dch = [d for l, d in sc if l < 0]
                mi = (sum(izq) / len(izq) * 100) if izq else None
                md = (sum(dch) / len(dch) * 100) if dch else None
                lg.write(f"STRAFE-CAL-RESUMEN lx>0 -> {mi if mi is not None else '-'}cm/tick ; "
                         f"lx<0 -> {md if md is not None else '-'}cm/tick "
                         f"(con el mapeo BIEN, lx>0 debe dar cm POSITIVOS=IZQ)\n"); lg.flush()
        except Exception:
            pass
        # resumen de la calibracion de giro: ¿el robot gira en el sentido que el modelo cree?
        tc = locals().get("turncal", [])
        if tc:
            pos = [yr for rx, yr in tc if rx > 0]; neg = [yr for rx, yr in tc if rx < 0]
            mp = sorted(pos)[len(pos) // 2] if pos else None
            mn = sorted(neg)[len(neg) // 2] if neg else None
            verdict = "OK (modelo correcto)"
            if mp is not None and mp > 5:   verdict = ">>> SIGNO INVERTIDO: rx>0 deberia BAJAR yaw y lo SUBE <<<"
            if mn is not None and mn < -5:  verdict = ">>> SIGNO INVERTIDO: rx<0 deberia SUBIR yaw y lo BAJA <<<"
            ph = locals().get("phcount", {})
            lg.write(f"TURN-CAL-RESUMEN rx>0 -> medido~{mp}deg/s (modelo<0) ; rx<0 -> medido~{mn}deg/s (modelo>0) "
                     f":: {verdict}\n")
            lg.write(f"FASES {ph}\n")
            lg.flush()
        if stop_event is not None:
            stop_event.set()


def cmd_goto(label=None):
    """PASO 4: navega a un waypoint guardado. Con argumento (A/B/...) va una vez; sin argumento, menu en
    vivo: escribes la etiqueta y te lleva, al llegar pide otra. 'q' para salir."""
    wps = _load_wps()
    if not wps:
        print("Sin waypoints. Captura primero: python g1_goto.py waypoint A"); return
    cdp = g.get_cdp()
    _install(cdp)
    lg = _open_goto_log()
    lg.write(f"\n=== GOTO {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n")

    def go(lbl):
        lbl = (lbl or "").strip()
        if lbl not in wps:
            print(f"  '{lbl}' no existe. Waypoints: {list(wps.keys())}"); return
        w = wps[lbl]
        navigate_to(cdp, lg, w["x"], w["y"], lbl)

    try:
        if label:
            go(label); return
        print(f"Waypoints disponibles: {', '.join(wps.keys())}")
        print("Escribe una etiqueta (A/B/...) y Enter para ir; 'q' para salir.")
        while True:
            try:
                sel = input("goto> ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if sel.lower() in ("q", "quit", "exit", ""):
                break
            go(sel)
    finally:
        cdp.eval(g.STOP_JS); time.sleep(0.2); cdp.eval(g.STOP_JS)
        lg.write("FIN\n"); lg.close()
        print("\nFin goto.")


# Hook de DESCUBRIMIENTO: guarda las estructuras CRUDAS completas de slam_info (por 'type') y
# slam_relocation/odom, para ver si traen covarianza / score / confianza de localizacion.
POSEDUMP_JS = r"""(function(){
  if(!window.__poseDump){ window.__poseDump=1; window.__sirawTypes={}; window.__reloraw=null;
    var jp=JSON.parse;
    JSON.parse=function(s){ var v=jp.apply(this,arguments);
      try{ if(v && v.topic){ var tp=''+v.topic;
        if(tp.indexOf('slam_info')>=0){ var d=(typeof v.data==='string')?jp(v.data):v.data;
          var ty=(d&&d.type)?(''+d.type):'?'; window.__sirawTypes[ty]=d; }
        if(tp.indexOf('slam_relocation/odom')>=0){ window.__reloraw=v.data; }
      }}catch(e){}
      return v;
    };
  } return 1;
})()"""


def cmd_posedump():
    """DESCUBRE si la pose trae COVARIANZA / score / confianza. Vuelca las estructuras crudas de
    slam_info y slam_relocation/odom a dataset/ y resalta cualquier campo de incertidumbre."""
    cdp = g.get_cdp()
    cdp.eval(POSEDUMP_JS)
    print(">>> POSEDUMP. Robot RELOCALIZADO. Recojo ~6s las estructuras crudas de pose...")
    time.sleep(6)
    try:
        si = json.loads(cdp.eval("JSON.stringify(window.__sirawTypes||{})") or "{}")
        rel = json.loads(cdp.eval("JSON.stringify(window.__reloraw||null)") or "null")
    except Exception:
        si = {}; rel = None
    try:
        os.makedirs(DATASET_DIR, exist_ok=True)
    except Exception:
        pass
    fn = os.path.join(DATASET_DIR, time.strftime("%Y%m%d_%H%M%S") + "_posedump.json")
    try:
        json.dump({"slam_info_by_type": si, "slam_relocation_odom": rel}, open(fn, "w"), indent=1)
    except Exception:
        pass
    UNC = ("cov", "score", "conf", "reliab", "status", "quality", "valid", "uncert", "error", "std")
    print("\n--- slam_info ---")
    for ty, d in si.items():
        keys = list(d.keys()) if isinstance(d, dict) else type(d).__name__
        print(f"  type={ty}: {keys}")
        if isinstance(d, dict):
            for k in d:
                if any(w in k.lower() for w in UNC):
                    print(f"     >> {k} = {d[k]}")
    print("--- slam_relocation/odom ---")
    if isinstance(rel, dict):
        print("  claves:", list(rel.keys()))
        po = rel.get("pose")
        if isinstance(po, dict):
            print("  pose.claves:", list(po.keys()))
            if "covariance" in po:
                print("  >> pose.covariance =", po["covariance"])
    else:
        print("  (no llego slam_relocation/odom; quizas solo slam_info en este modo)")
    print(f"\nGuardado {os.path.basename(fn)} en dataset/. Di 'mira el posedump' y te digo qué confianza hay.")


# Captura el MAPA CARGADO: cuando la app carga el .pcd para relocalizar, lo decodifica y renderiza en el
# WebView. Este hook guarda la nube MAS GRANDE que pase por los workers (= el mapa entero, decenas de
# miles de pts, frente a los ~1-3k del laser en vivo 'location'). Asi bajamos el mapa SIN getBigFile.
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
            if(n>window.__mapinfo.n){                          // guarda la nube MAS GRANDE = mapa cargado
              var a=(ArrayBuffer.isView(arr))?Array.prototype.slice.call(arr):Object.values(arr);
              window.__mapbuf=a.slice(0,600000);
              window.__mapinfo={n:n,type:ty,t:Date.now()};
            }
          }
        }catch(e){} });
      } }catch(e){}
      return o.apply(this,arguments);
    };
  } return JSON.stringify(window.__mapinfo);
})()"""


def cmd_mapgrab(secs=30):
    """Descarga el MAPA CARGADO desde el WebView (sin getBigFile): captura la nube mas grande que renderiza
    la app al cargar el .pcd. Con el mapa cargado, MUEVE/ROTA la vista del mapa en la app para que se
    redibuje. Guarda dataset/map_loaded.json."""
    cdp = g.get_cdp()
    cdp.eval(MAPGRAB_JS)
    print(">>> MAPGRAB. En la app: mapa CARGADO. Mueve/rota la VISTA del mapa (o re-localiza) para que se")
    print(f"    redibuje. Capturo la nube MAS GRANDE durante {secs}s. Ctrl+C para fijar antes.\n")
    t0 = time.time()
    try:
        while time.time() - t0 < secs:
            info = json.loads(cdp.eval(MAPGRAB_JS) or "{}")
            n = info.get("n", 0)
            print(f"  mapa max: {n // 3 if n else 0} puntos (type='{info.get('type', '')}')   "
                  f"t={time.time()-t0:.0f}/{secs}s", end="\r")
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    buf = json.loads(cdp.eval("JSON.stringify(window.__mapbuf||[])") or "[]")
    info = json.loads(cdp.eval(MAPGRAB_JS) or "{}")
    if len(buf) < 300:
        print(f"\n  Solo {len(buf)//3} puntos. ¿El mapa esta cargado/visible? Prueba a mover la vista del mapa.")
        return
    try:
        os.makedirs(DATASET_DIR, exist_ok=True)
    except Exception:
        pass
    fn = os.path.join(DATASET_DIR, "map_loaded.json")
    try:
        json.dump({"source": "app_loaded_map", "msg_type": info.get("type", ""),
                   "npts": len(buf) // 3, "points": buf}, open(fn, "w"))
        print(f"\n  MAPA CARGADO capturado: {len(buf)//3} puntos (msg type='{info.get('type','')}') -> {fn}")
        print("  Di 'mira el map_loaded' y detecto el frame (Y-up vs Z-up) + lo dibujo con waypoints y paths.")
    except Exception as e:
        print("\n  no se pudo guardar:", repr(e))


def cmd_buildmap(secs=40, force_loc=False):
    """Reconstruye el MAPA 3D del entorno. Dos fuentes segun el modo:
      - OPERACION/relocalizacion: acumula la nube 'location' (frame mapa Z-up, ANCLADO al .pcd, = frame de
        los waypoints, SIN drift). Es la fiable para validar paths. Mueve/gira el robot DESPACIO.
      - MAPEO (#/newSlam): nube densa en window.__buf (Three.js Y-up); rapida pero su frame DERIVA al
        conducir -> no encaja con A/B. Solo como vista rapida.
    'force_loc'=True obliga a usar 'location' (operacion) aunque haya __buf. Guarda dataset/map_full.json."""
    cdp = g.get_cdp()
    _install(cdp)
    time.sleep(1.0)
    nloc = int(cdp.eval("(window.__relocbuf||[]).length") or 0)
    nbuf = int(cdp.eval("(window.__buf||[]).length") or 0)
    acc = {}; src = ""

    if not force_loc and nloc < 100 and nbuf > 3000:
        # --- MODO MAPEO: nube densa en window.__buf (Three.js Y-up), va ACUMULANDO al conducir ---
        src = "mapping __buf (Y-up -> map)"
        print(f">>> BUILDMAP modo mapeo: CONDUCE el robot DESPACIO por las DOS habitaciones.")
        print(f"    window.__buf acumula (empezo en {nbuf//3} pts). Guardo al llegar a {secs}s o con Ctrl+C.")
        t0 = time.time()
        try:
            while time.time() - t0 < secs:
                n = int(cdp.eval("(window.__buf||[]).length") or 0)
                print(f"  __buf acumulado: {n//3} puntos   t={time.time()-t0:.0f}/{secs}s   "
                      f"(Ctrl+C para guardar)", end="\r")
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\n  Ctrl+C -> fijo el mapa con lo acumulado.")
        buf = json.loads(cdp.eval("JSON.stringify((window.__buf||[]).slice(0,800000))") or "[]")
        for i in range(0, len(buf) - 2, 3):
            X = buf[i]; H = buf[i + 1]; Z = buf[i + 2]          # idx0, idx1=altura, idx2
            k = (round(X / 0.05), round((-Z) / 0.05), round(H / 0.05))   # map: x=idx0, y=-idx2, z=idx1
            acc[k] = acc.get(k, 0) + 1
        minhits = 1
    else:
        # --- MODO OPERACION: acumula 'location' (frame mapa Z-up) ---
        src = "location (Z-up, map frame)"
        print(f">>> BUILDMAP {secs}s (modo operacion): conduce DESPACIO, ATRAVIESA la puerta a la habitacion B.")
        print(f"    Filtros: campo cercano (<{g.NEAR_BLIND}m), SALTOS de relocalizacion (frame descartado) y")
        print("    PERSISTENCIA (un voxel debe verse en varios frames -> mata el rastro de persona/dinamico).")
        if nloc < 100:
            print("  AVISO: no llega nube 'location'. ¿Mapa cargado y RELOCALIZADO (modo operation, como en benchmark)?")
        t0 = time.time(); nf = 0; jumps = 0; fi = 0; prevp = None
        try:
            while time.time() - t0 < secs:
                src2, p, _ = read_pose(cdp)
                px, py = (p[0], p[1]) if p else (None, None)
                if p and prevp is not None and math.hypot(px - prevp[0], py - prevp[1]) > 0.5:
                    jumps += 1; prevp = (px, py)             # salto de pose (imposible andando) = glitch reloc
                    print(f"  [reloc-JUMP #{jumps}] salto de pose -> descarto frame        ", end="\r")
                    time.sleep(0.3); continue                # los puntos irian a un sitio equivocado
                if p:
                    prevp = (px, py)
                buf = grab_full_cloud(cdp, cap=20000)
                fr = set()
                for i in range(0, len(buf) - 2, 3):
                    xx, yy, zz = buf[i], buf[i + 1], buf[i + 2]
                    if abs(zz) > 2.0:
                        continue                            # altura imposible = outlier
                    if px is not None and math.hypot(xx - px, yy - py) < g.NEAR_BLIND:
                        nf += 1; continue                   # anillo fantasma (cuerpo/suelo) junto al robot
                    fr.add((round(xx / 0.05), round(yy / 0.05), round(zz / 0.05)))
                for k in fr:
                    acc[k] = acc.get(k, 0) + 1              # cuenta FRAMES distintos en que aparece el voxel
                fi += 1
                print(f"  voxels={len(acc)} frames={fi}  (cercano {nf}, saltos {jumps})  t={time.time()-t0:.0f}/{secs}s   ", end="\r")
                time.sleep(0.3)
        except KeyboardInterrupt:
            pass
        minhits = 3        # PERSISTENCIA: solo voxels vistos en >=3 frames distintos (estatico). Persona/ruido cae.
        print(f"\n  {jumps} frames descartados por salto de relocalizacion; {fi} frames usados.")

    pts = [[k[0] * 0.05, k[1] * 0.05, k[2] * 0.05] for k, c in acc.items() if c >= minhits]
    try:
        os.makedirs(DATASET_DIR, exist_ok=True)
    except Exception:
        pass
    fn = os.path.join(DATASET_DIR, "map_full.json")
    try:
        json.dump({"frame": "map idx0=x,idx1=y,idx2=altura", "src": src, "voxel": 0.05,
                   "hband_obstac": [HBAND_LO, HBAND_HI], "npts": len(pts), "points": pts}, open(fn, "w"))
        print(f"\n  Mapa 3D: {len(pts)} voxels (fuente: {src}) -> {fn}")
        print("  Di 'mira el map_full' y lo dibujo con A/B + valido los paths. (Si es modo mapeo, verifico que A/B encajan.)")
    except Exception as e:
        print("\n  no se pudo guardar:", repr(e))


def cmd_tablecheck():
    """Captura AHORA la nube 3D completa + foto de la camara + pose, y muestra un histograma de ALTURA
    delante del robot, para ver si hay una MESA u otro obstaculo invisible al LiDAR de banda de torso.
    Coloca el robot MIRANDO al sitio del choque (a ~0.5-1 m)."""
    import collections
    cdp = g.get_cdp()
    _install(cdp)
    print(">>> TABLECHECK. Robot MIRANDO al obstaculo (mesa) a ~0.5-1 m. Capturo nube 3D + foto...")
    for _ in range(40):
        src, p, _ = read_pose(cdp)
        n = int(cdp.eval("(window.__relocbuf||[]).length") or 0)
        if p and n > 100:
            break
        time.sleep(0.3)
    else:
        print(" sin nube/pose. ¿Mapa cargado y relocalizado?"); return
    try:
        os.makedirs(DATASET_DIR, exist_ok=True)
    except Exception:
        pass
    base = os.path.join(DATASET_DIR, time.strftime("%Y%m%d_%H%M%S") + "_tablecheck")
    buf = grab_full_cloud(cdp); jpg = grab_cam(cdp)
    saved = []
    try:
        json.dump({"pose": p, "yaw": round(yaw_of(p), 1), "npts": len(buf) // 3, "points": buf,
                   "frame": "map idx0=x,idx1=y,idx2=altura"}, open(base + ".json", "w"))
        saved.append(os.path.basename(base + ".json"))
    except Exception as e:
        print("  no se pudo guardar la nube:", repr(e))
    if jpg:
        import base64
        try:
            with open(base + ".jpg", "wb") as f:
                f.write(base64.b64decode(jpg.split(",", 1)[1]))
            saved.append(os.path.basename(base + ".jpg"))
        except Exception:
            pass
    x, y = p[0], p[1]; yaw = yaw_of(p)
    h = collections.Counter(); nf = 0
    for i in range(0, len(buf) - 2, 3):
        px, py, pz = buf[i], buf[i + 1], buf[i + 2]
        dd = math.hypot(px - x, py - y)
        if dd < 0.1 or dd > 2.0:
            continue
        ang = abs((math.degrees(math.atan2(py - y, px - x)) - yaw + 180) % 360 - 180)
        if ang > 30:
            continue
        nf += 1; h[round(pz * 2) / 2] += 1
    print(f"\nFRENTE del robot (<2 m, cono ±30°): {nf} puntos. Histograma de ALTURA (idx2):")
    print("  (suelo ~ -1.3/-1.0 | torso/paredes ~ -0.5..+0.5 | techo ~ +1.3)")
    for k in sorted(h):
        print(f"  z~{k:+.1f}: {'#' * max(1, h[k] // 2)} {h[k]}")
    print(f"\nGuardado en dataset/: {', '.join(saved)}")
    print("Di 'mira el tablecheck' y analizo la nube + te enseño la foto.")


def cmd_turntest():
    """DIAGNOSTICO del SIGNO DE GIRO (causa tipica del 'da mil vueltas'). Gira el robot en el sitio a un
    lado y a otro midiendo el yaw REAL de slam_info, y comprueba si coincide con el modelo del DWA
    (wz=-1.8*rx -> rx>0 BAJA el yaw, rx<0 lo SUBE). ESPACIO LIBRE + mando en mano (L2+B)."""
    cdp = get_live_cdp()
    _install(cdp)
    print(">>> TURN-TEST. Espacio libre alrededor; mando en mano. Voy a girar el robot en el sitio.\n")
    for _ in range(20):
        src, p, _ = read_pose(cdp)
        if p:
            break
        time.sleep(0.3)
    else:
        print("Sin pose. ¿Mapa cargado y robot relocalizado?"); return

    def yaw_now():
        s, q, _ = read_pose(cdp)
        return yaw_of(q) if q else None

    def spin(rx, secs, name):
        y0 = yaw_now(); t0 = time.time(); acc = 0.0; prev = y0
        print(f"  girando {name} (rx={rx:+.2f}) {secs}s...")
        while time.time() - t0 < secs:
            cdp.eval(g.set_cmd_js(0, 0, rx, 0))
            time.sleep(0.1)
            yn = yaw_now()
            if yn is not None and prev is not None:
                acc += (yn - prev + 180) % 360 - 180; prev = yn
        cdp.eval(g.STOP_JS); time.sleep(0.5)
        rate = acc / secs
        exp = -103.0 * rx
        ok = (rate * exp > 0)
        print(f"    -> yaw cambio {acc:+.0f}deg ({rate:+.0f}deg/s). modelo esperaba {exp:+.0f}deg/s. "
              f"{'OK' if ok else '>>> INVERTIDO <<<'}")
        return rate, exp, ok

    try:
        r1 = spin(+0.35, 2.5, "DERECHA-modelo(yaw baja)")
        time.sleep(0.5)
        r2 = spin(-0.35, 2.5, "IZQUIERDA-modelo(yaw sube)")
        cdp.eval(g.STOP_JS)
        inverted = (not r1[2]) and (not r2[2])
        print("\n  VEREDICTO:", ">>> SIGNO DE GIRO INVERTIDO: hay que invertir rx en el control <<<"
              if inverted else ("OK: el modelo del DWA coincide con el giro real (el spin viene de otra cosa)"
                                 if r1[2] and r2[2] else "MIXTO/RUIDOSO: repite con mas espacio y robot quieto al inicio"))
        with _open_goto_log() as lg:
            lg.write(f"\n=== TURNTEST {time.strftime('%H:%M:%S')} ===\n")
            lg.write(f"  rx=+0.35 -> {r1[0]:+.0f}deg/s (exp {r1[1]:+.0f}) ok={r1[2]}\n")
            lg.write(f"  rx=-0.35 -> {r2[0]:+.0f}deg/s (exp {r2[1]:+.0f}) ok={r2[2]}\n")
            lg.write(f"  INVERTIDO={inverted}\n")
        print(f"\n  (guardado en {GOTO_LOG})")
    except KeyboardInterrupt:
        cdp.eval(g.STOP_JS); print("\n  cancelado.")
    finally:
        cdp.eval(g.STOP_JS); time.sleep(0.2); cdp.eval(g.STOP_JS)


def _load_refmap_points():
    """Puntos de pared del mapa de referencia (frame G1) para pintar de FONDO. Mapa elegido por G1_REFMAP
    ('g1' por defecto = mapa propio del G1; 'summit' = mapa del Summit alineado)."""
    return ref_points()


def _goto_window(vshare, lock, stop_event, label, wps):
    """Ventana en vivo (hilo principal), DOS paneles:
       izq = mapa cargado (fondo) + plan global + ruta + recorrido + robot;  der = LASER en vivo (robot-centrico)."""
    try:
        import matplotlib
        import matplotlib.pyplot as plt
    except Exception as e:
        print("!! matplotlib no disponible para la ventana:", repr(e))
        while not stop_event.is_set():
            time.sleep(0.3)
        return
    refmap = _load_refmap_points()
    try:
        import base64 as _b64, io as _io
        from PIL import Image as _Image
        _have_cam = True
    except Exception:
        _have_cam = False
    plt.ion()
    fig = plt.figure(figsize=(17, 9.2))
    gs = fig.add_gridspec(3, 2, width_ratios=[1.45, 1], height_ratios=[1, 1, 0.7])
    ax = fig.add_subplot(gs[:, 0])      # mapa+plan (izq, alto completo)
    axl = fig.add_subplot(gs[0, 1])     # laser (arriba dcha)
    axc = fig.add_subplot(gs[1, 1])     # camara (medio dcha)
    axm = fig.add_subplot(gs[2, 1])     # METRICAS clearance+progression (abajo dcha)
    try:
        fig.canvas.manager.set_window_title(f"G1 {label} — mapa+plan | laser | camara | metricas")
    except Exception:
        pass
    closed = {"v": False}
    def _on_close(e):
        closed["v"] = True; stop_event.set()      # cerrar la ventana PARA el robot si aun navega
    fig.canvas.mpl_connect("close_event", _on_close)
    print("Ventana abierta. Al TERMINAR la navegacion la dejo ABIERTA para que la revises; cierrala tu cuando quieras.")
    print("(cerrar la ventana o Ctrl+C durante la navegacion PARA el robot.)")
    try:
        while not closed["v"]:                     # sigue viva aunque la navegacion termine (stop_event)
            with lock:
                x = vshare["x"]; y = vshare["y"]; yaw = vshare["yaw"]; ph = vshare["ph"]
                d = vshare.get("d", 0); col = vshare.get("col", 0); t = vshare.get("t", 0)
                goal = vshare.get("goal"); carrot = vshare.get("carrot")
                obs = list(vshare.get("obs", [])); laser = list(vshare.get("laser", []))
                plan = list(vshare.get("plan", [])); trail = list(vshare.get("trail", []))
                gplan = list(vshare.get("gplan", [])); cam = vshare.get("cam")
                mhist = list(vshare.get("mhist", [])); m_clear = vshare.get("clear", 0); m_prog = vshare.get("prog", 0)
                shist = list(vshare.get("shist", [])); m_rel = vshare.get("rel", 1.0)
            # ===================== PANEL IZQ: mapa cargado + plan =====================
            ax.clear()
            if refmap:                               # FONDO = mapa real (Summit en frame G1) = paredes/puerta
                ax.scatter([p[0] for p in refmap], [p[1] for p in refmap],
                           s=4, c="#9aa6b2", marker="s", linewidths=0, alpha=0.55, label="mapa cargado (paredes)")
            for k, w in wps.items():
                ax.plot([w["x"]], [w["y"]], "s", c="#cfcfcf", ms=6)
                ax.annotate(k, (w["x"], w["y"]), fontsize=9, color="#777")
            if obs:                                  # obstaculos del laser acumulados (TTL)
                ax.scatter([p[0] for p in obs], [p[1] for p in obs],
                           s=14, c="#c0392b", marker="s", linewidths=0, alpha=0.5, label="obstaculos (laser)")
            if trail and len(trail) > 1:
                ax.plot([p[0] for p in trail], [p[1] for p in trail], "-", c="#34495e", lw=1.2, alpha=0.8, label="recorrido")
            if gplan and len(gplan) > 1:             # PLAN GLOBAL (verde)
                ax.plot([p[0] for p in gplan], [p[1] for p in gplan], "-", c="#00d000", lw=3.2, alpha=0.95,
                        label="PLAN GLOBAL (A* origen->destino)")
            if plan and len(plan) > 1:
                ax.plot([p[0] for p in plan], [p[1] for p in plan], "-", c="#1565c0", lw=1.8, label="ruta A* local")
            if carrot:
                ax.plot([carrot[0]], [carrot[1]], "o", c="#00bcd4", ms=8)
            if goal:
                ax.plot([goal[0]], [goal[1]], "*", c="#f39c12", ms=22, label=f"objetivo {label}")
            ax.plot([x], [y], "o", c="#2980b9", ms=11)
            ax.arrow(x, y, 0.4 * math.cos(math.radians(yaw)), 0.4 * math.sin(math.radians(yaw)),
                     head_width=0.16, head_length=0.16, fc="#2980b9", ec="#2980b9", length_includes_head=True)
            ax.set_aspect("equal"); ax.grid(True, alpha=0.2)
            ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
            done_sfx = "   [TERMINADO — cierra la ventana cuando quieras]" if stop_event.is_set() else ""
            ax.set_title(f"{label}  t={t:.0f}s  {ph.strip()}  dist={d:.2f}m  colis={col}{done_sfx}",
                         color=("#1a7d3c" if stop_event.is_set() else "black"))
            # CENTRA la vista en la accion (robot+waypoints+plan+recorrido), ignora el ruido lejano del mapa
            vw = [(x, y)] + ([goal] if goal else []) + [(w["x"], w["y"]) for w in wps.values()] + trail + gplan
            if vw:
                vx = [p[0] for p in vw]; vy = [p[1] for p in vw]
                cx = (min(vx) + max(vx)) / 2; cy = (min(vy) + max(vy)) / 2
                half = max(max(vx) - min(vx), max(vy) - min(vy)) / 2 + 1.5
                ax.set_xlim(cx - half, cx + half); ax.set_ylim(cy - half, cy + half)
            try:
                ax.legend(loc="upper right", fontsize=7)
            except Exception:
                pass
            # ===================== PANEL DER: laser en vivo en FRAME DEL ROBOT (delante = ARRIBA) =====
            axl.clear()
            if laser:                                  # rota los puntos para que el rumbo del robot apunte hacia +y
                a = math.radians(90 - yaw); ca, sa = math.cos(a), math.sin(a)
                lx = [ca * (p[0] - x) - sa * (p[1] - y) for p in laser]
                ly = [sa * (p[0] - x) + ca * (p[1] - y) for p in laser]
                axl.scatter(lx, ly, s=10, c="#16a085", marker="o", linewidths=0)
            for rr in (1, 2):                          # anillos de distancia
                axl.add_artist(plt.Circle((0, 0), rr, fill=False, color="#445", lw=0.6, alpha=0.6))
            axl.plot(0, 0, "o", c="#2980b9", ms=9)
            axl.arrow(0, 0, 0, 0.5, head_width=0.18, fc="#2980b9", ec="#2980b9")   # robot mira ARRIBA
            axl.annotate("delante", (0, 1.0), ha="center", fontsize=8, color="#2980b9")
            axl.set_xlim(-3, 3); axl.set_ylim(-3, 3); axl.set_aspect("equal")
            axl.grid(True, alpha=0.2); axl.set_title(f"LASER en vivo — frame robot  ({len(laser)} pts)")
            axl.set_xlabel("izq/dcha (m)"); axl.set_ylabel("delante (m)")
            # ===================== PANEL CAMARA (abajo dcha) =====================
            axc.clear(); axc.axis("off")
            if _have_cam and cam and isinstance(cam, str) and cam.startswith("data:image"):
                try:
                    imgc = _Image.open(_io.BytesIO(_b64.b64decode(cam.split(",", 1)[1])))
                    axc.imshow(imgc); axc.set_title("camara en vivo", fontsize=10)
                except Exception:
                    axc.set_title("camara (frame invalido)", fontsize=10)
            else:
                axc.set_title("camara (esperando frame...)", fontsize=10)
            # ===================== PANEL METRICAS: clearance + progression (las 2 del tutor) =====================
            axm.clear()
            if mhist:
                tt = [h[0] for h in mhist]; cc = [h[1] for h in mhist]; pp = [h[2] for h in mhist]
                axm.plot(tt, cc, "-", c="#1565c0", lw=1.6, label="clearance")
                axm.plot(tt, pp, "-", c="#e67e22", lw=1.6, label="progression")
                axm.fill_between(tt, cc, alpha=0.08, color="#1565c0")
                if tt[-1] - tt[0] > 40:                # ventana movil de ~40s
                    axm.set_xlim(tt[-1] - 40, tt[-1])
            if shist:                                  # fiabilidad de sensado (auto-evaluacion)
                st_t = [h[0] for h in shist]; st_r = [h[1] for h in shist]
                axm.plot(st_t, st_r, "-", c="#2ca02c", lw=1.6, label="sensing reliab.")
                if st_t[-1] - st_t[0] > 40:
                    axm.set_xlim(st_t[-1] - 40, st_t[-1])
            if mhist or shist:
                axm.legend(loc="upper left", fontsize=7, ncol=3)
            axm.set_ylim(-0.02, 1.05); axm.grid(True, alpha=0.3)
            axm.set_xlabel("t (s)")
            axm.set_title(f"clearance={m_clear:.2f}  progression={m_prog:.2f}  sensing={m_rel:.2f}", fontsize=10)
            plt.pause(0.25)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        plt.ioff()
        try:
            plt.close(fig)
        except Exception:
            pass


def cmd_goto_viz(label):
    """goto a un waypoint CON ventana en vivo (mapa + laser + odometria + ruta A*). El control corre en
    un hilo de fondo y la ventana en el principal. Una sola travesia (no menu)."""
    if not label:
        print("uso: python g1_goto.py gotoviz <N>   (ej: B)"); return
    wps = _load_wps()
    if label not in wps:
        print(f"'{label}' no existe. Waypoints: {list(wps.keys())}"); return
    w = wps[label]
    vshare = {"x": 0.0, "y": 0.0, "yaw": 0.0, "ph": "", "d": 0.0, "col": 0, "t": 0.0,
              "goal": (w["x"], w["y"]), "carrot": None, "obs": [], "laser": [], "plan": [], "gplan": [], "trail": [], "cam": None}
    lk = threading.Lock(); stop_event = threading.Event()

    def control():
        try:
            cdp = get_live_cdp()
            _install(cdp)
            lg = _open_goto_log()
            lg.write(f"\n=== GOTOVIZ {label} {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n")
            navigate_to(cdp, lg, w["x"], w["y"], label, vshare=vshare, lock=lk, stop_event=stop_event)
            lg.write("FIN\n"); lg.close()
        except Exception as e:
            print("Error en control:", repr(e)); stop_event.set()

    th = threading.Thread(target=control, daemon=True)
    th.start()
    try:
        _goto_window(vshare, lk, stop_event, label, wps)
    except KeyboardInterrupt:
        stop_event.set()
    finally:
        stop_event.set()
        th.join(timeout=6)
    print("Ventana cerrada. Fin goto.")


# =========================== BENCHMARK: navegacion NATIVA del firmware ===========================
# El firmware conduce (anyPointNavigation 1102, sniffeado); nosotros SOLO registramos los mismos
# metricas que nuestra nav (tiempo, recorrido, colisiones, laser, odometria) para comparar.
BENCH_LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "goto_native.log")

# apaga NUESTRO driver (el setInterval que envia rt/wirelesscontroller) para no pelear con el firmware
DISABLE_DRV_JS = ("(function(){if(window.__drv){clearInterval(window.__drv);window.__drv=null;}"
                  "window.__cmd={lx:0,ly:0,rx:0,ry:0};return 'drv-off';})()")

# captura SOLO el datachannel (sin arrancar driver), para poder enviar el goal nativo
NATIVE_CAP_JS = r"""(function(){
  if(!window.__dcHook){ window.__dcHook=1;
    var S=RTCDataChannel.prototype.send;
    RTCDataChannel.prototype.send=function(d){ try{ if((this.label||'')==='data') window.__dc=this; }catch(e){} return S.apply(this,arguments); };
  }
  return !!window.__dc;
})()"""


def _native_req_js(api_id, parameter_js, topic="rt/api/slam_operate/request"):
    return ("(function(){if(!window.__dc)return 'nodc';var id=Math.floor(Math.random()*1e9);"
            "var par=%s;"
            "var msg={type:'req',topic:'%s',data:{header:{identity:{id:id,api_id:%d}},parameter:par}};"
            "try{window.__dc.send(JSON.stringify(msg));return 'sent';}catch(e){return 'err:'+e;}})()"
            % (parameter_js, topic, api_id))


def native_goal_js(x, y):
    """anyPointNavigation 1102 -> destino (x,y) en frame del mapa (q todo 0 = sin restriccion de rumbo)."""
    par = ("JSON.stringify({data:{targetPose:{x:%g,y:%g,z:0,q_x:0,q_y:0,q_z:0,q_w:0},mode:1}})" % (x, y))
    return _native_req_js(1102, par)


def native_avoid_js(enable):
    """obstacles_avoid 1001 ON/OFF (que el firmware esquive = benchmark justo)."""
    par = "JSON.stringify({data:{enable:%s}})" % ("true" if enable else "false")
    return _native_req_js(1001, par, topic="rt/api/obstacles_avoid/request")


def native_cancel_js():
    """closeNavControlTask 1203 -> para la navegacion del firmware (seguridad al acabar/abortar)."""
    return _native_req_js(1203, "''")


def _path_len(trail, k=1):
    """Camino recorrido. k>1 DECIMA la traza antes de sumar.

    POR QUE EXISTE k (25-ago): la variacion total de una senal ruidosa crece con la
    frecuencia de muestreo -- el problema de la longitud de la costa. A cadencia nativa el
    temblor de pose del SLAM real infla este numero un 44% y el del gemelo solo un 11%, asi
    que comparar path_m ENTRE SISTEMAS a escala nativa midio ruido y fabrico tres hallazgos
    falsos (ver analysis/escala_pose.py y el commit 078be70).
    REGLA: path_m (k=1) se conserva para continuidad historica y es valido DENTRO de un
    mismo sistema, donde el ruido es compartido y se cancela en el contraste. Para comparar
    sistemas distintos se usa path_m_k8, a la escala declarada K_COMPARA=8 (~2.4 s).
    """
    t = trail[::max(1, int(k))]
    return sum(math.hypot(t[i + 1][0] - t[i][0], t[i + 1][1] - t[i][1])
               for i in range(len(t) - 1)) if len(t) > 1 else 0.0


def benchmark_run(cdp, lg, wx, wy, label, vshare=None, lock=None, stop_event=None):
    """BENCHMARK: lanza la navegacion NATIVA del firmware al waypoint y registra (pasivo) las mismas
    metricas que nuestra nav, para comparar. El firmware conduce; nosotros NO enviamos velocidad."""
    print(f"\n>>> BENCHMARK NATIVO -> '{label}' ({wx:+.2f},{wy:+.2f}). El FIRMWARE conduce; yo registro.")
    cdp.eval(DISABLE_DRV_JS)                       # no pelear con el firmware
    cdp.eval(g.LOWSTATE_JS); cdp.eval(RELOC_JS); cdp.eval(RELOC_CLOUD_JS); cdp.eval(NATIVE_CAP_JS)
    cdp.eval(HEALTH_JS); cdp.eval(IMUFULL_JS)
    refmap = load_ref_map()                        # mapa conocido para estimar confianza de localizacion
    print(f"  mapa de referencia: {len(refmap)} celdas" + (" (sin mapa -> confianza N/A)" if not refmap else ""))
    print("  Esperando pose + datachannel...", end="", flush=True)
    for _ in range(40):
        src, p, _ = read_pose(cdp)
        dc = cdp.eval("!!window.__dc")
        if p and dc:
            break
        time.sleep(0.3)
    else:
        print(" sin pose/datachannel. ¿Mapa cargado y robot relocalizado?"); return False
    print(" ok.")
    # --- GATE de relocalizacion (como la app): si la pose inicial esta lejos del waypoint mas cercano,
    #     la relocalizacion es dudosa -> NO navegamos (evita ir a un sitio equivocado). Override G1_NOGATE=1.
    fc = frame_check(lg, p[0], p[1])
    if fc and fc["offset_m"] > GATE_M and os.environ.get("G1_NOGATE") != "1":
        print(f"\n  >>> RELOCALIZACION DUDOSA: arrancas a {fc['offset_m']}m del waypoint {fc['nearest_wp']}.")
        print("      NO envio el goal. Re-localiza en la app (que los puntitos encajen con el mapa) y reintenta.")
        print("      (si de verdad arrancas lejos de un waypoint a proposito: G1_NOGATE=1 ...)")
        lg.write("GATE-BLOCKED reloc dudosa\n")
        return False
    cdp.eval(native_avoid_js(True))               # esquiva del firmware ON
    r = cdp.eval(native_goal_js(wx, wy))          # GOAL nativo (1102)
    print(f"  Goal nativo (1102) enviado: {r}.  Mando en mano (L2+B) por seguridad.")
    lg.write(f"NATIVE-GOAL {label} ({wx:+.3f},{wy:+.3f}) send={r}\n")
    rd = RunRecorder("native", label, (wx, wy))
    rd.rec["frame_check"] = fc

    t0 = time.time(); tprint = 0; trail = []; poshist = []
    low_t = 0; last_low = None; lt_base = []; ah_base = []; ncol = 0; last_col_t = -99
    minc0 = 9.9; stall_t = t0; last_movepos = None; pose_t = time.time()
    health_t = 0; hh = {}; jprev = None; cloud_ok = False; cloud_warned = False
    omap = {}; gplan = []; gplan_t = 0; start_xy = None      # mapa acumulado + plan global (solo viz/comparacion)
    cam_t = 0; cam_jpg = None
    try:
        while not (stop_event is not None and stop_event.is_set()):
            now = time.time()
            src, p, pcd = read_pose(cdp)
            if not p:
                if now - pose_t > 4.0:
                    print("\n  POSE PERDIDA (4s)."); break
                time.sleep(0.15); continue
            pose_t = now
            x, y, yaw = p[0], p[1], yaw_of(p)
            if jprev is not None and math.hypot(x - jprev[0], y - jprev[1]) > 0.5:
                jd = math.hypot(x - jprev[0], y - jprev[1])
                rd.event("reloc_jump", now - t0, x, y, {"dist": round(jd, 2),
                                                        "from": [round(jprev[0], 2), round(jprev[1], 2)]})
                lg.write(f"RELOC-JUMP {jd:.2f}m\n")
            jprev = (x, y)
            if not trail or math.hypot(x - trail[-1][0], y - trail[-1][1]) > 0.05:
                trail.append((x, y))
            poshist.append((now, x, y)); poshist = [h for h in poshist if now - h[0] <= 0.8]
            spd = (math.hypot(x - poshist[0][1], y - poshist[0][2]) / max(1e-3, now - poshist[0][0])
                   if len(poshist) >= 2 else 0.0)
            d_goal = math.hypot(wx - x, wy - y)
            if d_goal < NAV_REACH:                # --- LLEGADA ---
                cdp.eval(native_cancel_js())
                T = now - t0; plen = _path_len(trail)
                straight = math.hypot(wx - trail[0][0], wy - trail[0][1]) if trail else 0.0
                eff = (straight / plen) if plen > 0 else 0.0
                print(f"\n  LLEGADO (NATIVO) a '{label}' en {T:.1f}s | recorrido={plen:.2f}m recto={straight:.2f}m "
                      f"efic={eff:.2f} | colis={ncol} c0min={minc0:.2f}")
                lg.write(f"NATIVE-REACHED {label} t={T:.1f}s path={plen:.2f}m straight={straight:.2f}m "
                         f"eff={eff:.2f} ncol={ncol} c0min={minc0:.2f}\n"); lg.flush()
                rd.save_cloud("end", [round(x, 3), round(y, 3), round(yaw, 1)], grab_full_cloud(cdp))
                rd.save_cam("end", grab_cam(cdp))
                rd.finish("reached", {"time_s": round(T, 2), "path_m": round(plen, 2),
                                      "straight_m": round(straight, 2), "efficiency": round(eff, 2),
                                      "collisions": ncol, "c0min": round(minc0, 2),
                                      "start": {"x": round(trail[0][0], 3), "y": round(trail[0][1], 3)} if trail else None})
                if vshare is not None:
                    with lock:
                        vshare["ph"] = "LLEGADO"
                return True

            live = reloc_cells(cdp, (x, y))       # laser en vivo (mismo metodo que el nuestro -> comparable)
            if live:
                cloud_ok = True
            elif not cloud_ok and not cloud_warned and now - t0 > 4.0:
                print("\n  [AVISO] no llega la nube 'location' (nobs=0) -> dataset sin laser/loc_match.")
                print("          ¿se ven los PUNTITOS del laser en la app? Si no, el robot no la publica.")
                lg.write("NO-CLOUD warning (no 'location' stream)\n"); cloud_warned = True
            op = [(cx * g.OCELL, cy * g.OCELL) for (cx, cy) in live
                  if abs(cx * g.OCELL - x) < 2.6 and abs(cy * g.OCELL - y) < 2.6]
            c0 = clear_dir(x, y, yaw, 0, op); minc0 = min(minc0, c0)
            # mapa acumulado + PLAN GLOBAL (nuestro A* origen->destino) solo para ver/comparar en la ventana
            if start_xy is None:
                start_xy = (x, y)
            for c in live:
                omap[c] = now
            omap = {c: tt for c, tt in omap.items() if now - tt < NAV_OMAP_TTL}
            if vshare is not None and now - gplan_t > 3.0:
                obs_plan = set(omap.keys()) or refmap          # mapa vivo, o el de referencia, o (vacio)=recta
                gplan = global_plan(start_xy[0], start_xy[1], wx, wy, obs_plan)
                gplan_t = now

            if now - low_t > 0.2:                  # contacto por IMU/par (mismo detector)
                lw = g.read_low(cdp)
                if lw:
                    last_low = (math.hypot(lw.get("ax", 0.0), lw.get("ay", 0.0)), lw.get("legtau", 0.0))
                low_t = now
            cur_ah, cur_lt = last_low if last_low else (None, None)
            if spd > 0.06 and cur_lt is not None:
                lt_base.append(cur_lt); lt_base = lt_base[-40:]
                ah_base.append(cur_ah); ah_base = ah_base[-40:]
            if now - last_col_t > 4.0 and cur_lt is not None and len(lt_base) >= 5:
                bl = sorted(lt_base)[len(lt_base) // 2]; ba = sorted(ah_base)[len(ah_base) // 2]
                if cur_lt > bl * 1.5 + 3.0 or cur_ah > ba + 1.8:
                    ncol += 1; last_col_t = now
                    print(f"\n  CONTACTO #{ncol} [imu] (nativo) en ({x:+.2f},{y:+.2f}).")
                    lg.write(f"NATIVE-CONTACT #{ncol} pos=({x:+.2f},{y:+.2f}) legtau={cur_lt:.1f}\n")
                    rd.event("collision", now - t0, x, y, {"src": "imu", "legtau": round(cur_lt, 1)})
                    rd.save_cloud(f"col{ncol}", [round(x, 3), round(y, 3), round(yaw, 1)], grab_full_cloud(cdp))
                    rd.save_cam(f"col{ncol}", grab_cam(cdp))
            if last_movepos is None or math.hypot(x - last_movepos[0], y - last_movepos[1]) > 0.1:
                last_movepos = (x, y); stall_t = now
            stalled = now - stall_t > 3.0

            if now - health_t > 1.0:
                hh = read_telemetry(cdp); health_t = now
                rd.telem(now - t0, _telem_row(hh))
            loc = match_score(live, refmap)
            h = hh.get("h") or {}
            line = (f"t={now-t0:5.1f} NATIVO pos=({x:+.2f},{y:+.2f}) yaw={yaw:+6.1f} d={d_goal:.2f} "
                    f"spd={spd:.2f} c0={c0:.2f} loc={loc if loc is not None else '-'} bat={h.get('bat','-')} "
                    f"nobs={len(op)} col={ncol}{' STALL' if stalled else ''}")
            lg.write(line + "\n"); lg.flush()
            rd.sample(now - t0, x, y, yaw, d_goal, spd, c0, len(op), phase="NATIVO",
                      extra={"err": hh.get("err"), "bat": h.get("bat"), "cpuT": h.get("cpuT"),
                             "merr": h.get("merr"), "loc_match": loc})
            rd.maybe_laser(now - t0, op, ctx={"x": round(x, 3), "y": round(y, 3),
                                              "yaw": round(yaw, 1), "spd": round(spd, 2)})
            if now - tprint > 0.4:
                print("  " + line); tprint = now
            if vshare is not None:
                if now - cam_t > 0.5:
                    cam_jpg = grab_cam(cdp); cam_t = now
                with lock:
                    vshare["x"] = x; vshare["y"] = y; vshare["yaw"] = yaw; vshare["ph"] = "NATIVO"
                    vshare["d"] = d_goal; vshare["col"] = ncol; vshare["t"] = now - t0
                    vshare["goal"] = (wx, wy); vshare["carrot"] = None; vshare["plan"] = []
                    vshare["cam"] = cam_jpg
                    vshare["gplan"] = list(gplan)                                     # plan global (nuestro A*, referencia)
                    vshare["obs"] = [(cx * g.OCELL, cy * g.OCELL) for (cx, cy) in omap]   # mapa acumulado (m)
                    vshare["laser"] = [(cx * g.OCELL, cy * g.OCELL) for (cx, cy) in live]
                    vshare["trail"] = list(trail)
            time.sleep(0.1)
        if "x" in dir():
            rd.save_cloud("end", [round(x, 3), round(y, 3), round(yaw, 1)], grab_full_cloud(cdp))
        rd.finish("aborted", {"time_s": round(time.time() - t0, 2), "path_m": round(_path_len(trail), 2),
                              "collisions": ncol, "c0min": round(minc0, 2)})
        return False
    except KeyboardInterrupt:
        print(f"\n  [ABORTADO benchmark '{label}']"); lg.write(f"NATIVE-ABORT {label}\n")
        if "x" in dir():
            rd.save_cloud("end", [round(x, 3), round(y, 3), round(yaw, 1)], grab_full_cloud(cdp))
        rd.finish("aborted", {"time_s": round(time.time() - t0, 2), "path_m": round(_path_len(trail), 2),
                              "collisions": ncol, "c0min": round(minc0, 2)})
        return False
    finally:
        cdp.eval(native_cancel_js()); time.sleep(0.2); cdp.eval(native_cancel_js())
        if stop_event is not None:
            stop_event.set()


def cmd_benchmark(label, viz=False):
    """Lanza la navegacion NATIVA del firmware a un waypoint y registra metricas en goto_native.log
    (benchmark para comparar con nuestra nav). 'viz' abre la ventana en vivo."""
    if not label:
        print("uso: python g1_goto.py benchmark <N> [viz]"); return
    wps = _load_wps()
    if label not in wps:
        print(f"'{label}' no existe. Waypoints: {list(wps.keys())}"); return
    w = wps[label]
    if not viz:
        cdp = get_live_cdp()
        lg = open(BENCH_LOG, "a")
        lg.write(f"\n=== BENCHMARK NATIVO {label} {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n")
        try:
            benchmark_run(cdp, lg, w["x"], w["y"], label)
        finally:
            lg.write("FIN\n"); lg.close()
            print(f"\nFin benchmark. Log -> {BENCH_LOG}")
        return
    vshare = {"x": 0.0, "y": 0.0, "yaw": 0.0, "ph": "", "d": 0.0, "col": 0, "t": 0.0,
              "goal": (w["x"], w["y"]), "carrot": None, "obs": [], "laser": [], "plan": [], "gplan": [], "trail": [], "cam": None}
    lk = threading.Lock(); stop_event = threading.Event()

    def control():
        try:
            cdp = get_live_cdp()
            lg = open(BENCH_LOG, "a")
            lg.write(f"\n=== BENCHMARK NATIVO {label} {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n")
            benchmark_run(cdp, lg, w["x"], w["y"], label, vshare=vshare, lock=lk, stop_event=stop_event)
            lg.write("FIN\n"); lg.close()
        except Exception as e:
            print("Error en benchmark:", repr(e)); stop_event.set()

    th = threading.Thread(target=control, daemon=True); th.start()
    try:
        _goto_window(vshare, lk, stop_event, label + " (NATIVO)", wps)
    except KeyboardInterrupt:
        stop_event.set()
    finally:
        stop_event.set(); th.join(timeout=6)
    print(f"Ventana cerrada. Fin benchmark. Log -> {BENCH_LOG}")


def _load_wps():
    try:
        return json.load(open(WP_FILE))
    except Exception:
        return {}


def _save_map(cdp, pose=None):
    """Acumula el mapa de obstaculos a nav_map.json desde la nube 'location' (frame del MAPA, celdas OCELL),
    filtrando el campo cercano (anillo fantasma) con la pose. El mapa es siempre el mismo: se va completando."""
    try:
        prev = set(tuple(c) for c in json.load(open(MAP_FILE)).get("cells", []))
    except Exception:
        prev = set()
    prev |= reloc_cells(cdp, pose)
    try:
        json.dump({"cells": [list(c) for c in prev], "OCELL": g.OCELL,
                   "frame": "map", "hband": [HBAND_LO, HBAND_HI]}, open(MAP_FILE, "w"))
    except Exception:
        pass
    return len(prev)


def cmd_furniture(secs=15):
    """CENSO DE MUEBLES por VISION: acumula en nav_map.json las celdas que ve el canal DEPTH
    (banda 0.10-1.30 m absoluta: sofa, sillas, muebles BAJO el plano del laser). El acumulador
    de 'waypoint' usa el laser y NO puede verlos (autopsia 2026-07-21: freno-arrastre -> HELP
    en el pasillo del sofa porque el A* planeaba a traves de el; el mueble avisado por camara
    no estaba mapeado). USO: robot QUIETO de pie ENFRENTE del mueble a 1-1.5 m; repetir desde
    2 angulos. Requiere G1_PERC y el perception_server corriendo. Anti-ruido: solo celdas
    vistas en >=5 frames y a 0.4-3.0 m del robot."""
    if g1_perception is None:
        print("falta g1_perception.py"); return
    perc = g1_perception.make_client_from_env(g.OCELL)
    if perc is None:
        print("define G1_PERC=host:puerto (y perception_server corriendo)"); return
    cdp = g.get_cdp()
    _install(cdp)
    print(f">>> CENSO {secs}s: robot QUIETO frente al mueble. Acumulando celdas de VISION...")
    counts = {}
    frames = 0
    t0 = time.time()
    px = py = None
    while time.time() - t0 < secs:
        srcp, p, _ = read_pose(cdp)
        if not p:
            time.sleep(0.2); continue
        x, y, yaw = p[0], p[1], yaw_of(p)
        px, py = x, y
        fr = grab_cam(cdp)
        if not fr:
            time.sleep(0.2); continue
        res = perc.query(fr, x, y, yaw)
        if res is None:
            time.sleep(0.2); continue
        frames += 1
        for c in (res.cells or ()):  # celdas OCELL en frame del MAPA
            d = math.hypot(c[0] * g.OCELL - x, c[1] * g.OCELL - y)
            if 0.4 <= d <= 3.0:
                counts[tuple(c)] = counts.get(tuple(c), 0) + 1
        print(f"  frames={frames} celdas_candidatas={len(counts)}   t={time.time()-t0:.0f}/{secs}s", end="\r")
        time.sleep(0.25)
    good = {c for c, n in counts.items() if n >= 5}
    print()
    if not good:
        print("Sin celdas estables (¿mueble a 1-1.5 m y de frente? ¿perception_server con depth vivo?)")
        return
    try:
        prev = json.load(open(MAP_FILE)); cells = set(tuple(c) for c in prev.get("cells", []))
    except Exception:
        cells = set()
    added = good - cells
    if not added:
        print("Todas las celdas ya estaban en el mapa. Nada que guardar."); return
    print(f"+{len(added)} celdas CANDIDATAS (revisa que caen SOBRE el mueble, no sobre personas):")
    for c in sorted(added):
        print("   (%.1f, %.1f)" % (c[0] * g.OCELL, c[1] * g.OCELL))
    # salvaguardas (auditoria 22-jul): una persona en el encuadre se volvia mueble permanente
    try:
        resp = input("¿Guardar estas celdas en nav_map.json? [s/N] ").strip().lower()
    except EOFError:
        resp = "n"
    if resp != "s":
        print("Descartado (nada escrito)."); return
    try:
        import shutil
        shutil.copy(MAP_FILE, MAP_FILE + ".bak")      # backup rodante: 'furniture undo' restaura
    except Exception:
        pass
    cells |= added
    json.dump({"cells": [list(c) for c in sorted(cells)], "OCELL": g.OCELL,
               "frame": "map", "hband": [HBAND_LO, HBAND_HI]}, open(MAP_FILE, "w"))
    print(f"nav_map.json: +{len(added)} celdas (total {len(cells)}). 'furniture undo' deshace.")
    print("Repite desde otro angulo si el mueble es grande.")


def cmd_furniture_undo():
    """Restaura nav_map.json desde el backup previo al ultimo censo (furniture)."""
    import shutil
    if not os.path.exists(MAP_FILE + ".bak"):
        print("No hay backup (.bak) que restaurar."); return
    shutil.copy(MAP_FILE + ".bak", MAP_FILE)
    print("nav_map.json restaurado desde el backup previo al ultimo censo.")


def cmd_waypoint(label):
    """PASO 2: conduce el robot al destino (con la app o teleop) y Ctrl+C -> guarda la ULTIMA pose como 'label'.
    Mientras, acumula el mapa de obstaculos en nav_map.json."""
    if not label:
        print("uso: python g1_goto.py waypoint <NOMBRE>   (ej: A, B, cocina...)"); return
    cdp = g.get_cdp()
    _install(cdp)
    print(f">>> WAYPOINT '{label}'. Lleva el robot al destino (app/teleop). Cuando este EN el punto, Ctrl+C.")
    print("    (voy mostrando la pose y acumulando el mapa). \n")
    last = None
    try:
        while True:
            src, p, pcd = read_pose(cdp)
            if p:
                last = (src, p, pcd)
                ncells = _save_map(cdp, p)
                print(f"  [{src}] x={p[0]:+.2f} y={p[1]:+.2f} yaw={yaw_of(p):+6.1f}  mapa={ncells} celdas", end="\r")
            time.sleep(0.3)
    except KeyboardInterrupt:
        if not last:
            print("\n!! No capture pose. ¿Mapa cargado y relocalizado?"); return
        src, p, pcd = last
        wps = _load_wps()
        wps[label] = {"x": round(p[0], 3), "y": round(p[1], 3), "yaw": round(yaw_of(p), 1),
                      "src": src, "pcd": pcd, "t": time.strftime("%Y-%m-%d %H:%M:%S")}
        json.dump(wps, open(WP_FILE, "w"), indent=2)
        print(f"\n\n  WAYPOINT '{label}' guardado: x={p[0]:+.3f} y={p[1]:+.3f} (fuente {src}, mapa '{pcd}')")
        print(f"  Total waypoints: {list(wps.keys())}  -> {WP_FILE}")


def cmd_listwp():
    wps = _load_wps()
    if not wps:
        print("Sin waypoints. Captura con: python g1_goto.py waypoint A"); return
    print("Waypoints guardados:")
    for k, v in wps.items():
        print(f"  {k}: x={v['x']:+.2f} y={v['y']:+.2f} yaw={v.get('yaw', 0):+.0f}  ({v.get('src')}, {v.get('pcd', '')})")


def cmd_noisecheck(secs=20):
    """Mide el RUIDO REAL de los sensores con el robot QUIETO (sin conducir): jitter del laser, deriva de
    pose, confianza de localizacion + bateria/temperatura/salud de motor por-junta. Es el 'feedback del
    robot sobre su propia capacidad' (Renxi). Guarda dataset/<ts>_noise.json + .png."""
    cdp = get_live_cdp()
    if not cdp:
        print("No hay pagina viva del WebView. ¿proxy + app + relocalizado?"); return
    cdp.eval(g.LOWSTATE_JS); cdp.eval(HEALTH_JS)
    print("Esperando pose + nube...", end="", flush=True)
    for _ in range(30):
        src, p, _ = read_pose(cdp); n = int(cdp.eval("(window.__relocbuf||[]).length") or 0)
        if p and n > 30:
            break
        time.sleep(0.3)
    else:
        print(" sin datos. ¿Mapa cargado y robot RELOCALIZADO?"); return
    print(" ok.")
    print(f"\n>>> MANTÉN EL ROBOT QUIETO {secs}s (de pie, sin conducir). Midiendo ruido de sensores...")
    refmap = load_ref_map(); t0 = time.time(); rows = []
    _covref_nc = load_cov_ref()               # instrumento v2: referencia de SESION (G1_COVREF)
    _ultimo_celdas = [-9.9]
    _luma_t = 0.0                             # luma CRUDA por fila (25-ago): el modelo de ruido
    sens = g1_metrics.SensingMonitor()        # del canal de luz del gemelo se ajusta de aqui
    while time.time() - t0 < secs:
        src, p, _ = read_pose(cdp)
        if not p:
            time.sleep(0.1); continue
        x, y, yaw = p[0], p[1], yaw_of(p)
        live = reloc_cells(cdp)
        op = [(cx * g.OCELL, cy * g.OCELL) for (cx, cy) in set(live)]
        c0 = clear_dir(x, y, yaw, 0, op)
        loc = match_score(live, refmap) if refmap else None
        sv = sens.update(time.time() - t0, live, c0, loc, False)
        # Cobertura (21-ago, para el testigo W1 con cristal REAL): el deficit contra el mapa
        # historico por fila, y las CELDAS CRUDAS a ~1 Hz para recomputar offline contra
        # cualquier referencia (p.ej. la de sesion). Solo grabacion: noisecheck no conduce.
        cd, cn, cb = _cov_deficit(x, y, yaw, live, refmap, g.OCELL, g.NEAR_BLIND) \
            if refmap else (None, 0, None)
        cd2, cn2, cb2 = _cov_deficit_v2(x, y, yaw, live, _covref_nc, g.OCELL, g.NEAR_BLIND) \
            if _covref_nc else (None, 0, None)
        fila = {"t": round(time.time() - t0, 2), "x": round(x, 4), "y": round(y, 4), "yaw": round(yaw, 2),
                "n": len(live), "c0": round(c0, 3), "loc": loc,
                "reliability": sv["reliability"], "laser_noise": sv["laser_noise"],
                "c0_std": sv["c0_std"], "scan_churn": sv["scan_churn"],
                "cov_def": cd, "cov_n": cn, "cov_blind": cb,
                "cov2_def": cd2, "cov2_n": cn2, "cov2_blind": cb2}
        # luma cruda del frame a ~3 Hz (sin EMA: la serie cruda es la que fija sd y
        # autocorrelacion del ruido; la EMA del contrato se deriva offline)
        if time.time() - t0 - _luma_t >= 0.3:
            _fr_nc = grab_cam(cdp)
            if _fr_nc and _fr_nc.startswith("data:image"):
                try:
                    import base64 as _b64, io as _io
                    from PIL import Image as _Im, ImageStat as _Ist
                    fila["luma"] = round(_Ist.Stat(_Im.open(_io.BytesIO(
                        _b64.b64decode(_fr_nc.split(",", 1)[1]))).convert("L")).mean[0], 2)
                    _luma_t = time.time() - t0
                except Exception:
                    pass
        if not rows or (fila["t"] - _ultimo_celdas[0]) >= 1.0:
            fila["celdas"] = sorted(list(live))[:600]
            _ultimo_celdas[0] = fila["t"]
        rows.append(fila)
        time.sleep(0.1)
    hh = read_telemetry(cdp); H = hh.get("h") or {}
    import statistics as _st
    xs = [r["x"] for r in rows]; ys = [r["y"] for r in rows]; ns = [r["n"] for r in rows]
    c0s = [r["c0"] for r in rows]; locs = [r["loc"] for r in rows if r["loc"] is not None]
    drift = (_st.pstdev(xs) ** 2 + _st.pstdev(ys) ** 2) ** 0.5 if len(xs) > 1 else 0.0
    summary = {"secs": secs, "ticks": len(rows), "pose_drift_cm": round(drift * 100, 2),
               "count_mean": round(_st.mean(ns), 1) if ns else 0,
               "count_std": round(_st.pstdev(ns), 1) if len(ns) > 1 else 0,
               "c0_mean": round(_st.mean(c0s), 3) if c0s else 0,
               "c0_std_m": round(_st.pstdev(c0s), 3) if len(c0s) > 1 else 0,
               "loc_mean": round(_st.mean(locs), 3) if locs else None,
               "loc_std": round(_st.pstdev(locs), 3) if len(locs) > 1 else None,
               "battery_pct": H.get("bat"), "batT": H.get("batT"), "cpuT": H.get("cpuT"),
               "motTmax": H.get("motTmax"), "motThot_idx": H.get("motThot"), "merr": H.get("merr")}
    os.makedirs(DATASET_DIR, exist_ok=True)
    base = os.path.join(DATASET_DIR, time.strftime("%Y%m%d_%H%M%S") + "_noise")
    json.dump({"schema": "g1_noise/v1", "summary": summary, "rows": rows,
               "motorTemp": H.get("motorTemp"), "motorError": H.get("motorError")}, open(base + ".json", "w"))
    print("\n=== RUIDO DE SENSORES (robot quieto) = capacidad de sensado del robot ===")
    for k, v in summary.items():
        print(f"  {k}: {v}")
    print(f"  -> {base}.json")
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        t = [r["t"] for r in rows]
        loc_al = [(r["loc"] if r["loc"] is not None else float("nan")) for r in rows]
        rel_al = [r["reliability"] for r in rows]
        fig, ax = plt.subplots(2, 2, figsize=(12, 8))
        ax[0, 0].plot(t, ns, c="#16a085"); ax[0, 0].set_title(f"laser point count (std={summary['count_std']})")
        ax[0, 0].set_xlabel("s")
        ax[0, 1].plot(t, c0s, c="#1565c0"); ax[0, 1].set_title(f"forward clearance c0 (noise std={summary['c0_std_m']} m)")
        ax[0, 1].set_xlabel("s")
        sc = ax[1, 0].scatter(xs, ys, s=10, c=t, cmap="viridis")
        ax[1, 0].set_title(f"pose drift while still (std={summary['pose_drift_cm']} cm)")
        ax[1, 0].set_aspect("equal"); ax[1, 0].set_xlabel("x (m)"); ax[1, 0].set_ylabel("y (m)")
        ax[1, 1].plot(t, loc_al, c="#8e44ad", label="loc_match")
        ax[1, 1].plot(t, rel_al, c="#2ca02c", label="reliability")
        ax[1, 1].set_ylim(0, 1.05); ax[1, 1].set_title("localisation conf. / sensing reliability")
        ax[1, 1].set_xlabel("s"); ax[1, 1].legend(fontsize=8)
        fig.suptitle(f"G1 sensor self-capacity (still {secs}s)   battery={summary['battery_pct']}%   "
                     f"motTmax={summary['motTmax']}C   drift={summary['pose_drift_cm']}cm")
        fig.tight_layout(); fig.savefig(base + ".png", dpi=100)
        print(f"  -> {base}.png")
    except Exception as e:
        print("  (grafica omitida:", e, ")")


def main():
    if len(sys.argv) < 2:
        print(__doc__); return
    c = sys.argv[1]
    if c == "reloccheck":
        cmd_reloccheck()
    elif c == "clouddebug":
        cmd_clouddebug()
    elif c == "cloudgrab":
        cmd_cloudgrab()
    elif c == "pathsniff":
        cmd_pathsniff(sys.argv[2] if len(sys.argv) > 2 else None)
    elif c == "appsniff":
        cmd_appsniff(int(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2].isdigit() else 40)
    elif c == "waypoint":
        cmd_waypoint(sys.argv[2] if len(sys.argv) > 2 else None)
    elif c == "furniture":
        if len(sys.argv) > 2 and sys.argv[2] == "undo":
            cmd_furniture_undo()
        else:
            cmd_furniture(int(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2].isdigit() else 15)
    elif c == "listwp":
        cmd_listwp()
    elif c == "noisecheck":
        cmd_noisecheck(int(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2].isdigit() else 20)
    elif c == "turntest":
        cmd_turntest()
    elif c == "tablecheck":
        cmd_tablecheck()
    elif c == "posedump":
        cmd_posedump()
    elif c == "buildmap":
        secs = next((int(a) for a in sys.argv[2:] if a.isdigit()), 40)
        cmd_buildmap(secs, force_loc=("loc" in sys.argv[2:] or "op" in sys.argv[2:]))
    elif c == "mapgrab":
        cmd_mapgrab(int(sys.argv[2]) if len(sys.argv) > 2 else 30)
    elif c in ("benchmark", "native"):
        label = sys.argv[2] if len(sys.argv) > 2 else None
        cmd_benchmark(label, viz=("viz" in sys.argv[2:]))
    elif c == "gotoviz":
        cmd_goto_viz(sys.argv[2] if len(sys.argv) > 2 else None)
    elif c == "goto":
        label = sys.argv[2] if len(sys.argv) > 2 else None
        if label and "viz" in sys.argv[2:]:          # 'goto B viz' -> con ventana
            cmd_goto_viz(label)
        else:
            cmd_goto(label)
    else:
        print("comandos: reloccheck | clouddebug | cloudgrab | waypoint <N> | listwp | goto [N] | gotoviz <N>")
        print(__doc__)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("Error:", repr(e)); sys.exit(1)
