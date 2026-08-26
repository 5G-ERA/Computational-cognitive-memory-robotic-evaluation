# -*- coding: utf-8 -*-
"""Las cuatro condiciones C1-C4 del protocolo DCC — paso 5 del §8.

El diseno 2x2 sale de DOS funciones y UNA restriccion de interfaz, no de cuatro
implementaciones separadas:

                        interfaz I0            interfaz I1 (revisada)
    verificador
    incumbente          C1                     C3
    temporal

    resolucion
    distribuida         C2                     C4

  - `vista(m, "I0"|"I1")` recorta la muestra a lo que esa interfaz expone.
  - `verifica_incumbente(v)` es el verificador temporal: retain / reject / unresolved
    sobre UNA sola memoria, la de movimiento->carga util (§6.1).
  - `resuelve_rol(v)` (dcc_roles.py) es la resolucion distribuida sobre todos los roles.

Que las cuatro condiciones compartan codigo NO es un atajo: es lo que garantiza que los
contrastes midan el factor declarado y no diferencias de implementacion. C4-C3 cambia solo
el proceso de decision; C4-C2 cambia solo la interfaz.

POR QUE LA RESTRICCION DE INTERFAZ NO NECESITA LOGICA APARTE
------------------------------------------------------------
`resuelve_rol` ya trata "fundamento no instrumentado" como fundamento no reclamable, y
resuelve review/defer cuando faltan a la vez el de lidar y el de RGB. Recortar la muestra a
I0 hace exactamente eso, asi que C2 = resolucion distribuida sobre una muestra sin campos de
calidad. Y el resultado es el que el protocolo predice para C2 en §6.2: "puede resolver
correctamente review o defer, pero puede fallar al distinguir ausencia de objeto de ausencia
de observacion fiable". No hay que programar ese fallo: emerge de quitar los campos.

MAPEO AL ESPACIO DE delta_t
---------------------------
A_meta = 1[Z_t = delta_t] necesita que las cuatro condiciones respondan en el MISMO espacio.
delta_t vive en el espacio de roles. El verificador temporal responde retain/reject/unresolved
(§6.1), asi que se declara el mapeo:

    retain      -> "motion"    la memoria incumbente sigue siendo la aplicable
    unresolved  -> "review", o "defer" si ademas el avance esta vetado
    reject      -> "no_use"    la incumbente no aplica y no hay otra que reconstruir

Consecuencia buscada, no defecto: C1 y C3 NUNCA pueden emitir lidar_quality, illumination,
object ni energy. Un verificador de una sola memoria no puede reconstruir un rol que esa
memoria no representa -- que es la tesis del §2.2 del protocolo. Por eso C4-C1 debe ser
grande justo en los episodios que exigen esos roles, y por eso el estado retain/reject/
unresolved se conserva aparte (`estado_incumbente`) como desenlace secundario propio.
"""

from dcc_roles import resuelve_rol, autoridad, C0_BLOQUEO

# --- que expone cada interfaz -------------------------------------------------
# I0 = <mapa global, mapa local, lecturas de sensor ACTUALES>. Nada de historia,
# calidad, incertidumbre ni autoridad.
I0_CAMPOS = {
    "t", "x", "y", "yaw", "d", "spd", "c0", "c0_hard", "c0_std", "nobs", "n_hard",
    "clearance", "clearance_m", "clear_left", "clear_right", "clearL_m", "clearR_m",
    "balance", "loc_conf", "loc_match", "perc_n", "color_near", "color_pts", "color_rmin",
    "carpet_pct", "bat", "cmd", "sent", "phase", "dets", "door_b", "goal_err",
    "carrot", "carrot_err", "plan_n", "progression", "progress_rate", "err",
}
# I1 anade historia, calidad, incertidumbre y autoridad. Lo que NO esta aqui es lo que
# todavia falta construir: lidar historico (paso 4) y RGB historico.
I1_EXTRA = {
    "laser_trust", "laser_noise", "scan_churn", "scan_fresh", "filt_rej", "reliability",
    "door_contra", "iface_q", "meta_state", "perc_age", "reloc_rate10s",
    "cov_blind", "cov_def", "cov_missing", "cov_n",
    "illum_b", "dvis_gate", "phase_sent", "map_add", "map_del",
}

INTERFACES = ("I0", "I1")
CONDICIONES = ("C1", "C2", "C3", "C4")
PROCESO = {"C1": "temporal", "C2": "distribuida", "C3": "temporal", "C4": "distribuida"}
INTERFAZ = {"C1": "I0", "C2": "I0", "C3": "I1", "C4": "I1"}


def vista(m, interfaz):
    """La muestra tal y como la ve esa interfaz. Recorte, no reescritura."""
    if interfaz == "I1":
        permitidos = I0_CAMPOS | I1_EXTRA
    elif interfaz == "I0":
        permitidos = I0_CAMPOS
    else:
        raise ValueError("interfaz desconocida: %r" % (interfaz,))
    return {k: v for k, v in m.items() if k in permitidos}


def _bloqueado(v):
    """El avance esta efectivamente vetado en este boundary."""
    c0h = v.get("c0_hard")
    if isinstance(c0h, (int, float)) and c0h < C0_BLOQUEO:
        return True
    if autoridad(v) == "safety":
        return True
    s = v.get("sent")
    if isinstance(s, (list, tuple)) and len(s) >= 1:
        try:
            return abs(float(s[0])) < 0.02
        except (TypeError, ValueError):
            return False
    return False


def verifica_incumbente(v, usa_pose=True):
    """Verificador temporal de UNA memoria: movimiento -> estabilidad de la carga util.

    Pregunta si la relacion incumbente sigue siendo APLICABLE aqui y ahora, no si es cierta.
    Verificarla exige poder establecer dos cosas: como de rapido se va, y si el entorno
    reclama una restriccion de velocidad. Si el entorno no se puede establecer con la
    evidencia que la interfaz expone, la conclusion honesta es `unresolved` -- y la memoria
    NO se marca falsa (§6.1).
    """
    razones = []

    # ¿se puede establecer la pose? sin ella no hay "aqui" que verificar.
    # Umbral en 0.80, que es el p5 real de ambos campos (38779 muestras): con 0.55 la
    # condicion no se disparaba NUNCA y C1 salia artificialmente confiado. Un baseline
    # debil infla C4-C1, y entonces el banco mide la implementacion en vez del factor.
    # DECISION (a), 24-ago: en el GEMELO esta pata se ignora, declarandolo. loc_match no
    # mide lo mismo en los dos sistemas -- en el robot las celdas vienen del buffer de
    # relocalizacion YA ALINEADO por el SLAM contra un mapa hecho con esos mismos retornos,
    # y en el gemelo de un ray-march contra geometria que el mapa nunca registro. Medido:
    # quitarla mueve C1 0.3 puntos en el material del 21-ago, porque loc_match esta saturado
    # tambien en el robot (mediana 0.94, p5 0.81). Se apaga EXPLICITAMENTE, nunca en silencio.
    if usa_pose:
        lc = v.get("loc_conf"); lm = v.get("loc_match")
        if isinstance(lc, (int, float)) and lc < 0.80:
            razones.append("pose:loc_conf=%.2f<0.80" % lc)
        elif isinstance(lm, (int, float)) and lm < 0.80:
            razones.append("pose:loc_match=%.2f<0.80" % lm)

    # INESTABILIDAD VISIBLE EN I0. c0_std es la dispersion de la propia holgura frontal y
    # esta en I0 (§2.1 la lista como "lidar actual"): si la medida salta, la relacion
    # incumbente no se puede verificar CON ESTA LECTURA, y eso se sabe sin campos de calidad.
    # p75=0.17, p95=0.71 -> 0.40 es cola clara, no ruido.
    cs = v.get("c0_std")
    if isinstance(cs, (int, float)) and cs >= 0.40:
        razones.append("lectura:c0_std=%.2f>=0.40 inestable" % cs)

    # el mundo no responde como el modelo de movimiento supone: se ordena avanzar y el
    # progreso es negativo. Tambien es I0 (pose actual + orden actual).
    pr = v.get("progress_rate"); cmd = v.get("cmd")
    ordenado = 0.0
    if isinstance(cmd, (list, tuple)) and len(cmd) >= 2:
        try:
            ordenado = abs(float(cmd[1]))
        except (TypeError, ValueError):
            ordenado = 0.0
    if isinstance(pr, (int, float)) and pr < -0.05 and ordenado > 0.10:
        razones.append("respuesta:progress_rate=%.2f con avance ordenado" % pr)

    # ¿se puede establecer el entorno? nobs/n_hard/plan_n son la evidencia de I0
    nobs = v.get("nobs"); nh = v.get("n_hard"); pn = v.get("plan_n")
    sin_entorno = (isinstance(nobs, (int, float)) and nobs <= 0)
    if sin_entorno:
        razones.append("entorno:nobs=0")
    if isinstance(pn, (int, float)) and pn <= 0 and isinstance(nh, (int, float)) and nh <= 0:
        razones.append("entorno:sin plan ni celdas duras")

    # con I1 hay ademas evidencia EXPLICITA de calidad. Un verificador de una sola memoria
    # no puede reconstruir otro rol, pero SI puede saber que no puede verificar el suyo:
    # esto es precisamente lo que el contraste C3-C1 debe capturar.
    lt = v.get("laser_trust")
    if isinstance(lt, (int, float)) and lt < 0.70:
        razones.append("calidad:laser_trust=%.2f<0.70" % lt)
    cb = v.get("cov_blind")
    if isinstance(cb, (int, float)) and cb >= 0.30:
        razones.append("calidad:cov_blind=%.2f>=0.30" % cb)
    ms = v.get("meta_state")
    if ms in ("BLIND", "DEGRADED"):
        razones.append("calidad:meta_state=%s" % ms)
    il = v.get("illum_b")
    if isinstance(il, (int, float)) and il > 99.0:
        razones.append("calidad:illum=%.0f>99 RGB inadmisible" % il)

    if razones:
        return "unresolved", "no verificable: " + "; ".join(razones)

    # `reject` queda reservado: la incumbente solo deja de aplicar si la tarea de movimiento
    # ha terminado. El protocolo insiste en que una memoria preservada no se declara falsa
    # porque su aplicabilidad actual no se pueda establecer.
    if v.get("err"):
        return "reject", "la tarea de movimiento no esta en curso (err=%s)" % v.get("err")
    return "retain", "verificable: pose y entorno establecidos, sin senal de calidad adversa"


def _a_rol(estado, v):
    if estado == "retain":
        return "motion"
    if estado == "reject":
        return "no_use"
    return "defer" if _bloqueado(v) else "review"


def evalua(m, cond, usa_pose=True):
    """Ejecuta UNA condicion sobre UNA muestra.

    Devuelve un dict con la decision en el espacio de delta_t (`Z`), su fundamento, la
    autoridad, y -- solo para las condiciones temporales -- el estado incumbente crudo,
    que el §5 de la pre-registracion puntua como desenlace secundario propio.
    """
    if cond not in CONDICIONES:
        raise ValueError("condicion desconocida: %r" % (cond,))
    v = vista(m, INTERFAZ[cond])
    aut = autoridad(v)
    if PROCESO[cond] == "temporal":
        estado, razon = verifica_incumbente(v, usa_pose=usa_pose)
        return {"cond": cond, "Z": _a_rol(estado, v), "razon": razon,
                "authority": aut, "estado_incumbente": estado,
                "pose_ignorada": (not usa_pose)}
    rol, razon, _a = resuelve_rol(v)
    return {"cond": cond, "Z": rol, "razon": razon,
            "authority": aut, "estado_incumbente": None}


def evalua_todas(m, usa_pose=True):
    """Las cuatro condiciones sobre la misma muestra: la fila del diseno 2x2."""
    return {c: evalua(m, c, usa_pose=usa_pose) for c in CONDICIONES}


def usa_pose_para(run):
    """¿Vale la pata de pose para ESTE run? Decision (a): en el gemelo no.

    `run` es el dict del fichero de dataset. El gemelo se identifica por `sim_id`, que el
    grabador ya escribe; no hace falta adivinar por nombre de fichero ni por fecha.
    """
    return (run or {}).get("sim_id") is None
