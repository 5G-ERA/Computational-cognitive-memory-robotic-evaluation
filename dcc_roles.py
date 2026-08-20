#!/usr/bin/env python3
"""DCC: ventanas de interfaz, resolucion de roles y verificacion del titular.

Las CUATRO condiciones del protocolo salen de combinar dos ejes, y este modulo es el unico
sitio donde viven, para que el nivel de replay y el fisico no puedan divergir:

                        interfaz I0 (original)     interfaz I1 (revisada)
  verificacion titular        C1                          C3
  resolucion distribuida      C2                          C4

DECISION DE DISENO QUE IMPORTA: C2 no esta "empeorado" a mano. Es el MISMO resolutor que C4
mirando por una ventana que no expone las distinciones. Falla, cuando falla, porque esta ciego
-- que es justo lo que predice el Teorema 1 (Contextual Aliasing Bound) y lo que el protocolo
pide proteger: "prevent hidden history leakage in original-interface conditions". La ventana se
aplica BORRANDO campos, no confiando en que el resolutor no los mire.

Los seis roles y las salidas gobernadas son los del protocolo, con su redaccion:
  motion          Regulate motion to protect the payload.
  sensor          Use sensing evidence within its operating envelope.
  object          Plan motion relative to a reliably identified object.
  energy          Adapt the delivery plan to available energy.
  lidar_coverage  Navigate under degraded spatial observation.
  illumination    Interpret or withhold RGB-based object semantics under degraded lighting.
  no_use / review / defer   salidas gobernadas (no son roles)

NADA DE ESTE MODULO TOCA EL MANDO. La particion de autoridad exige que la evidencia no se
convierta en permiso de control por si sola; aqui solo se interpreta y se resuelve.
"""

ROLES = ("motion", "sensor", "object", "energy", "lidar_coverage", "illumination")
GOBERNADAS = ("no_use", "review", "defer")

# --- I0: lo que la interfaz original expone (mapa global/local, lecturas ACTUALES, pose,
#     carga y bateria). Sin historia, sin calidad, sin incertidumbre, sin autoridad.
I0_CAMPOS = frozenset((
    "t", "x", "y", "yaw", "phase", "d", "goal_err", "carrot_err", "plan_n", "spd", "cmd", "sent",
    "c0", "c0_hard", "clearance_m", "clearL_m", "clearR_m", "clear_left", "clear_right", "balance",
    "nobs", "n_hard", "perc_n", "dets", "door_b", "bat",
))
# --- I1 = I0 + historia seleccionada, calidad, incertidumbre y autoridad.
I1_EXTRA = frozenset((
    "cov_def", "cov_blind", "cov_n",            # cobertura de lidar (prospectiva)
    "laser_trust", "laser_noise", "scan_churn", "scan_fresh", "filt_rej",   # calidad e historia
    "iface_q", "door_contra", "perc_age", "map_add", "map_del",
    "illum_q", "illum_state",                   # calidad visual (pendiente del contrato)
    "authority",                                # autoridad aplicable
))
I1_CAMPOS = I0_CAMPOS | I1_EXTRA

# Umbrales. Son parametros del RESOLUTOR (capa bajo prueba), no del robot: se calibran en
# material de desarrollo y se congelan antes de lo confirmatorio.
UMBRAL = {
    "cov_def": 0.20,        # deficit de cobertura que basta para invocar el rol (twin: p90 sano 0.07)
    "bat_baja": 45.0,       # % de bateria que hace relevante la energia
    "det_conf": 0.50,       # confianza minima para "objeto fiablemente identificado"
    "obj_cerca": 2.5,       # m: mas alla, un objeto no gobierna la marcha
    "obj_no_mapeado": 0.30, # m que el barrido se adelanta al mapa = algo que el mapa no explica
}


def vista(muestra, nivel):
    """Aplica la ventana de interfaz BORRANDO lo que esa interfaz no expone."""
    permitidos = I0_CAMPOS if nivel == "I0" else I1_CAMPOS
    return {k: v for k, v in muestra.items() if k in permitidos}


def _pregunta_de_objeto(v):
    """Se plantea AQUI una pregunta de objeto?

    Corregido tras el primer humo (17-ago): la regla de incertidumbre visual se disparaba en
    cada tick y devolvia review el 100% del tiempo, tragandose todos los demas roles. Eso
    confundia "no puedo responder a la pregunta del objeto" con "aqui no se plantea ninguna
    pregunta del objeto" -- un fallo de Semantic Locality por mi parte, de la misma clase que
    el protocolo advierte. La incertidumbre visual solo es decisiva cuando hay algo delante que
    habria que identificar, o cuando ya hay una deteccion cuya fiabilidad esta en duda."""
    # SEGUNDA correccion (mismo dia): "hay una pared a dos metros" NO es una pregunta de objeto.
    # Con c0<=2.5 la puerta quedaba abierta casi siempre en una oficina y todo seguia saliendo
    # review. La pregunta se plantea cuando hay algo delante QUE EL MAPA NO EXPLICA -- ahi si
    # importa que es -- o cuando ya hay una deteccion cuya fiabilidad esta en duda.
    c0 = v.get("c0"); ch = v.get("c0_hard")
    if (isinstance(c0, (int, float)) and isinstance(ch, (int, float))
            and c0 <= UMBRAL["obj_cerca"] and (ch - c0) > UMBRAL["obj_no_mapeado"]):
        return True
    return bool(v.get("dets"))


def _det_fiable(v):
    """Hay un objeto identificado con suficiente confianza y suficientemente cerca?"""
    for d in (v.get("dets") or []):
        try:
            conf = float(d[1]); rng = d[3]
        except (TypeError, IndexError, ValueError):
            continue
        if conf >= UMBRAL["det_conf"] and rng is not None and float(rng) <= UMBRAL["obj_cerca"]:
            return True
    return False


def resolve_distributed(v):
    """E-DCA: reconstruye los roles disponibles y resuelve el actual (C2 con I0, C4 con I1).

    Devuelve (salida, motivo). 'salida' es un rol o una gobernada. El orden de precedencia esta
    declarado a proposito y es revisable: se emite ademas la lista de fundamentos aplicables
    (ver grounds()) para poder re-resolver en replay sin repetir runs."""
    # 1) Energia: limita la CAPACIDAD, no el conocimiento. Va primero porque un plan que no cabe
    #    en la bateria no mejora por ver mejor.
    b = v.get("bat")
    if isinstance(b, (int, float)) and b <= UMBRAL["bat_baja"]:
        return ("energy", "bateria %.0f%% <= %.0f" % (b, UMBRAL["bat_baja"]))

    # 2) Cobertura degradada. SOLO visible con I1: con I0 este rol es literalmente inalcanzable,
    #    y esa imposibilidad es el resultado que el experimento quiere medir, no un defecto.
    cd = v.get("cov_def")
    if isinstance(cd, (int, float)) and cd >= UMBRAL["cov_def"]:
        return ("lidar_coverage", "cov_def %.2f >= %.2f" % (cd, UMBRAL["cov_def"]))

    # 3) Iluminacion. Requiere el contrato de calidad visual, que AUN NO EXISTE (la luminancia
    #    media no vale: la aplana la auto-exposicion). Mientras no exista, una deteccion negativa
    #    no puede leerse como "no hay objeto": eso seria exactamente el aliasing del protocolo.
    est = v.get("illum_state")
    if _pregunta_de_objeto(v):
        if est == "inadequate":
            return ("illumination", "iluminacion inadecuada con pregunta de objeto pendiente")
        if est is None and not _det_fiable(v):
            # hay algo que identificar y NO se puede saber si el RGB estaba en condiciones:
            # "no hay objeto" y "no puedo verlo" son indistinguibles -> no se fuerza
            return ("review", "pregunta de objeto sin evidencia de iluminacion: indistinguible")

    # 4) Objeto identificado con fiabilidad.
    if _det_fiable(v):
        return ("object", "deteccion fiable dentro de %.1f m" % UMBRAL["obj_cerca"])

    # 5) Sensado dentro de su envolvente.
    lt = v.get("laser_trust")
    if isinstance(lt, (int, float)) and lt < 0.6:
        return ("sensor", "laser_trust %.2f fuera de envolvente" % lt)

    # 6) Titular: regular la marcha para proteger la carga.
    return ("motion", "sin fundamento que desplace al titular")


def verify_incumbent(v):
    """Verificacion TEMPORAL del titular (C1 con I0, C3 con I1).

    Pregunta solo si la memoria movimiento->carga SIGUE siendo aplicable. Espacio de salida
    retain / reject / unresolved. No puede reconstruir un rol alternativo: esa incapacidad es la
    linea base del protocolo, no un fallo -- y por eso 'unresolved' es una respuesta responsable
    y no se penaliza como error."""
    b = v.get("bat")
    if isinstance(b, (int, float)) and b <= UMBRAL["bat_baja"]:
        return ("reject", "la capacidad disponible ya no sostiene el plan")
    cd = v.get("cov_def")
    if isinstance(cd, (int, float)) and cd >= UMBRAL["cov_def"]:
        return ("unresolved", "cobertura degradada: aplicabilidad no verificable")
    if _pregunta_de_objeto(v):
        if v.get("illum_state") == "inadequate":
            return ("unresolved", "iluminacion inadecuada: aplicabilidad no verificable")
        if not _det_fiable(v):
            return ("unresolved", "pregunta de objeto sin observacion fiable")
    lt = v.get("laser_trust")
    if isinstance(lt, (int, float)) and lt < 0.6:
        return ("unresolved", "sensado fuera de envolvente")
    return ("retain", "sigue aplicando")


def grounds(v):
    """Todos los fundamentos APLICABLES, no solo el que gana la precedencia.

    Semantic Locality exige mantenerlos distintos aunque varios recomienden frenar; emitirlos
    todos deja la precedencia auditable y permite re-resolver en replay sin repetir runs."""
    g = []
    b = v.get("bat")
    if isinstance(b, (int, float)) and b <= UMBRAL["bat_baja"]:
        g.append("energy")
    cd = v.get("cov_def")
    if isinstance(cd, (int, float)) and cd >= UMBRAL["cov_def"]:
        g.append("lidar_coverage")
    if v.get("illum_state") == "inadequate" and _pregunta_de_objeto(v):
        g.append("illumination")
    if _det_fiable(v):
        g.append("object")
    lt = v.get("laser_trust")
    if isinstance(lt, (int, float)) and lt < 0.6:
        g.append("sensor")
    return g


def authority_of(m):
    """Que autoridad limito el mando en este tick. Se DERIVA de lo ya emitido: la sesion del
    20-ago no lleva codigo nuevo sin validar en el gemelo.

    Authority Partitioning exige separar evidencia, interpretacion, meta-decision, seguridad y
    ejecucion. Sin este campo el par W4 -- evidencia ausente frente a autoridad no resuelta -- no
    es escenificable, porque las dos mitades se ven igual desde fuera: el robot no avanza.

    LA SENAL SON LAS MARCAS DE FASE, no 'sent' contra 'cmd'. Un primer intento comparo esos dos
    campos y dio 71% de 'safety', que es absurdo; la causa es que 'sent' es lo enviado el tick
    ANTERIOR y ya lleva la rampa dentro, asi que la comparacion mezcla la aceleracion normal con
    la accion del guardia. Las marcas, en cambio, las pone cada guardia al actuar:
        !H  guardia duro: frena segun holgura contra obstaculos persistentes
        !D  guardia de puerta
        !C  !c  bloqueo por camara
        !S  tope de la maquina META
        ~   limitador de RAMPA -- NO es autoridad, solo acota el cambio entre ticks
    Precedencia declarada: operador > seguridad > gobernanza > meta.
    """
    if m.get("meta_state") == "ASSIST":
        return ("operator", "mando cedido al operador")
    ph = str(m.get("phase", ""))
    for marca, motivo in (("!H", "guardia duro por holgura"),
                          ("!D", "guardia de puerta"),
                          ("!C", "bloqueo por camara"),
                          ("!c", "avance minimo por camara")):
        if marca in ph:
            return ("safety", motivo)
    if m.get("meta2_cap") is not None:
        return ("governance", "techo de gobernanza vigente")
    if "!S" in ph:
        return ("meta", "tope de la maquina META")
    return ("nav", "autoridad normal de navegacion")


def condicion(muestra, cond):
    """Resuelve una muestra bajo C1..C4. Devuelve dict con salida, motivo y fundamentos."""
    nivel = "I0" if cond in ("C1", "C2") else "I1"
    v = vista(muestra, nivel)
    if cond in ("C1", "C3"):
        out, why = verify_incumbent(v)
        modo = "incumbent"
    else:
        out, why = resolve_distributed(v)
        modo = "distributed"
    auth, auth_why = authority_of(muestra)      # la autoridad NO depende de la ventana:
    return {"cond": cond, "iface": nivel, "mode": modo,   # es un hecho del tick, no evidencia
            "out": out, "why": why, "grounds": grounds(v),
            "authority": auth, "authority_why": auth_why}
