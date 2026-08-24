# -*- coding: utf-8 -*-
"""Resolucion de ROL cognitivo (Z_t) para el protocolo DCC de Renxi — paso 2 del §8.

    A_meta = 1[ Z_t = delta_t(y_t) ]

`delta_t` viene del guion del experimento (§4.2 del mapeo: lo que el robot DEBERIA resolver,
conocido por construccion). `Z_t` es lo que el robot resuelve DE VERDAD con su propia
evidencia, y es lo que faltaba: sin el, el desenlace primario no se puede computar.

DOS PROPIEDADES DELIBERADAS
---------------------------
1. **Funcion pura sobre una muestra.** No toca estado global ni el control. Se puede llamar
   en vivo desde g1_goto.py y tambien sobre muestras YA grabadas, asi que cualquier run con
   los campos necesarios se puede puntuar a posteriori sin repetirla.

2. **Observa, no actua.** El §5.3 del protocolo (Authority Partitioning) exige que "la
   confianza del sensor no se convierta en autoridad de control automaticamente". Este modulo
   EMITE el rol resuelto; no modifica ninguna orden. Que el rol influya en el control es la
   diferencia entre C1..C4 y se construye en el paso 5, no aqui.

EVIDENCIA AUSENTE NO ES EVIDENCIA DE AUSENCIA
---------------------------------------------
Si el fundamento de un rol no esta instrumentado en esa muestra, el rol NO puede reclamarse.
Cuando ademas hay una pregunta de objeto pendiente, esa ausencia es exactamente el fallo de
Contextual Span del §4.2 (confundir "no hay objeto" con "no hay observacion fiable") y se
resuelve `review`, nunca una conclusion de objeto.

UMBRALES
--------
Todos salen de medida sobre 132 runs reales (38779 muestras) o de un contrato ya congelado,
y viven en UN solo sitio para que una ablacion pueda permutarlos:

  bateria          p5=26 p25=48 mediana=72. Regla operativa ya vigente y medida: por debajo
                   del 60% la tasa de desplazamiento lateral cae un 46%. Critico a 35 (< p5).
  cov_blind        fraccion de rumbos informativos dentro del recorte ciego del fabricante.
                   p75=0.14, p95=0.54 -> 0.30 separa "algo de ceguera" de "ciego de verdad".
  cov_missing      celdas de la referencia de SESION ausentes 2 barridos. p50=1 p75=2 p95=4.
  laser_trust      validez retrospectiva del laser (Renxi). p5=0.85 -> por debajo de 0.70 es
                   una cola clara, no ruido.
  illum            EMA(0.2) de la luma media. Contrato congelado en
                   tasks/VISUAL_QUALITY_CONTRACT.md: >99 = RGB inadmisible.
  conf de objeto   detecciones reales no-puerta: p25=0.55 mediana=0.68. Se exige >=0.55.
"""

ROLES = ("motion", "lidar_quality", "illumination", "object", "energy",
         "review", "defer", "no_use")

# --- umbrales declarados (un solo sitio; una ablacion los permuta aqui) ---
BAT_CRIT      = 35.0    # por debajo: la capacidad limita la tarea entera
BAT_BAJA      = 60.0    # por debajo: replanificar, regla operativa medida
COV_BLIND_MAL = 0.30
COV_MISS_MAL  = 3
TRUST_MAL     = 0.70
ILLUM_MAX     = 99.0    # contrato visual congelado
CONF_MIN      = 0.55
C0_BLOQUEO    = 0.35    # holgura dura por debajo de la cual el avance esta efectivamente vetado

# Precedencia declarada. La energia entra DOS veces a proposito: critica manda sobre los
# fundamentos de sensado (si no se puede completar la tarea, interpretar mejor no ayuda),
# y baja queda por debajo de ellos (una limitacion del plan no gobierna un boundary concreto
# por encima de no poder ver). Es una decision de diseno, no una verdad: esta escrita aqui
# para que Renxi la pueda cambiar y para que la ablacion "role-level removal" la permute.
PRECEDENCIA = ("no_use", "energy_crit", "defer", "review",
               "lidar_quality", "illumination", "object", "energy_baja", "motion")


def _num(m, k):
    v = m.get(k)
    return float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) else None


def autoridad(m):
    """Que capa autorizo LA ORDEN EMITIDA, leida de los marcadores de guardia de phase_sent.

    Los marcadores los pone el propio control cuando una guardia modifica la orden:
      !H HARD-GUARD (obstaculo duro)     !C/!c COLOR-BRAKE (obstaculo solo-camara)  -> safety
      !S META-SM (DEGRADED/BLIND)        !M META2 (techo de la analogia DCE)        -> meta
      !D DOOR-GUARD (suelo de zona-vano)                                            -> nav
    Sin marcador manda el controlador. La seguridad es la ultima palabra cuando coinciden.
    """
    ph = m.get("phase_sent") or m.get("phase") or ""
    if not isinstance(ph, str):
        return "controller"
    if "ASSIST" in ph:
        return "human"
    if "!H" in ph or "!C" in ph or "!c" in ph:
        return "safety"
    if "!S" in ph or "!M" in ph:
        return "meta"
    if "!D" in ph:
        return "nav"
    return "controller"


def resuelve_rol(m, cfg=None):
    """Devuelve (role, role_reason, authority) para UNA muestra.

    `m` es el dict de muestra tal cual se graba en dataset/*.json.
    """
    c = cfg or {}
    bat_crit = c.get("BAT_CRIT", BAT_CRIT); bat_baja = c.get("BAT_BAJA", BAT_BAJA)
    cb_mal = c.get("COV_BLIND_MAL", COV_BLIND_MAL); cm_mal = c.get("COV_MISS_MAL", COV_MISS_MAL)
    tr_mal = c.get("TRUST_MAL", TRUST_MAL); il_max = c.get("ILLUM_MAX", ILLUM_MAX)
    cf_min = c.get("CONF_MIN", CONF_MIN); c0_blk = c.get("C0_BLOQUEO", C0_BLOQUEO)

    aut = autoridad(m)
    cand = {}          # nombre de precedencia -> (rol emitido, razon)

    # --- no_use: el problema cae fuera de todo rol preservado ---
    if aut == "human":
        cand["no_use"] = ("no_use", "assist:humano interviene")
    elif m.get("err"):
        cand["no_use"] = ("no_use", "fault:err=%s" % m.get("err"))

    # --- energia ---
    bat = _num(m, "bat")
    if bat is not None:
        if bat < bat_crit:
            cand["energy_crit"] = ("energy", "bat=%.0f<%.0f critico" % (bat, bat_crit))
        elif bat < bat_baja:
            cand["energy_baja"] = ("energy", "bat=%.0f<%.0f replanificar" % (bat, bat_baja))

    # --- calidad de lidar / cobertura ---
    cov_blind = _num(m, "cov_blind"); cov_miss = _num(m, "cov_missing")
    trust = _num(m, "laser_trust"); cov_n = _num(m, "cov_n")
    lidar_evid = any(v is not None for v in (cov_blind, cov_miss, trust))
    lidar_mal = False; r_lidar = []
    if cov_blind is not None and cov_blind >= cb_mal:
        lidar_mal = True; r_lidar.append("cov_blind=%.2f>=%.2f" % (cov_blind, cb_mal))
    if cov_miss is not None and cov_miss >= cm_mal:
        lidar_mal = True; r_lidar.append("cov_missing=%.0f>=%d" % (cov_miss, cm_mal))
    if trust is not None and trust < tr_mal:
        lidar_mal = True; r_lidar.append("laser_trust=%.2f<%.2f" % (trust, tr_mal))
    if cov_n is not None and cov_n <= 0:
        lidar_mal = True; r_lidar.append("cov_n=0 sin rumbos informativos")
    if lidar_mal:
        cand["lidar_quality"] = ("lidar_quality", "cobertura:" + ",".join(r_lidar))

    # --- iluminacion: el contrato visual decide si la semantica RGB es admisible ---
    illum = _num(m, "illum_b")
    rgb_inadmisible = illum is not None and illum > il_max
    if rgb_inadmisible:
        cand["illumination"] = ("illumination",
                                "contrato:illum=%.0f>%.0f RGB inadmisible" % (illum, il_max))

    # --- objeto: identificado con fiabilidad Y con la semantica RGB admitida ---
    dets = [z for z in (m.get("dets") or []) if z and z[0] != "door"]
    mejor = None
    for z in dets:
        if len(z) >= 2 and isinstance(z[1], (int, float)) and z[1] >= cf_min:
            if mejor is None or z[1] > mejor[1]:
                mejor = z
    if mejor is not None and not rgb_inadmisible and illum is not None:
        cand["object"] = ("object", "objeto:%s conf=%.2f admisible" % (mejor[0], mejor[1]))

    # --- insuficiencia CONJUNTA: nunca una conclusion de objeto forzada (§4.2) ---
    # "inadecuado" incluye NO INSTRUMENTADO: ausencia de evidencia no es evidencia de ausencia.
    lidar_inadecuado = lidar_mal or not lidar_evid
    rgb_inadecuado = rgb_inadmisible or illum is None
    if lidar_inadecuado and rgb_inadecuado:
        por = []
        por.append("lidar " + ("degradado" if lidar_mal else "sin instrumentar"))
        por.append("rgb " + ("inadmisible" if rgb_inadmisible else "sin instrumentar"))
        c0h = _num(m, "c0_hard")
        enviado = m.get("sent") or []
        parado = (isinstance(enviado, (list, tuple)) and len(enviado) >= 1
                  and abs(float(enviado[0])) < 0.02)
        bloqueado = (c0h is not None and c0h < c0_blk) or aut == "safety" or parado
        if bloqueado:
            cand["defer"] = ("defer", "conjunta:" + "+".join(por) + ";avance vetado")
        else:
            cand["review"] = ("review", "conjunta:" + "+".join(por))

    # --- resolucion por precedencia ---
    for k in PRECEDENCIA:
        if k in cand:
            rol, razon = cand[k]
            return rol, razon, aut
    return "motion", "defecto:sin fundamento activo", aut


def anota(muestras, cfg=None):
    """Anota una lista de muestras en sitio. Devuelve el recuento por rol."""
    from collections import Counter
    cnt = Counter()
    for m in muestras:
        r, why, a = resuelve_rol(m, cfg)
        m["role"], m["role_reason"], m["authority"] = r, why, a
        cnt[r] += 1
    return cnt


# ---------------------------------------------------------------------------------------
# ESTABILIZACION DE ROL (24-ago). La primera transicion escenificada (T3) mostro el rol
# alternando object <-> motion veinte veces en 47 s, en saltos de 0.2-0.3 s: ruido de
# deteccion cruzando un umbral sin histeresis. Con las FRONTERAS DE DECISION como unidad
# del banco, ese tableteo fabrica fronteras que no existen y contamina cualquier medida de
# retardo de conmutacion.
#
# LOS UMBRALES SON UNA ELECCION, NO UN HALLAZGO. Se midio la duracion de 121 episodios y la
# distribucion decae suave (27% <0.5 s, 17% 0.5-1 s, 6% 1-1.5 s): NO hay valle donde cortar.
# Lo que si decide es una asimetria de costes: el retardo de confirmacion es una constante
# DECLARADA que puede restarse del retardo de conmutacion medido, mientras que el tableteo
# es ruido que no se puede quitar despues. Por eso se confirma antes de adoptar.
#
# La abstencion NO se confirma. Adoptar review/defer/no_use es la respuesta prudente ante
# evidencia insuficiente, y retrasarla seria exactamente el error que el protocolo llama
# fallo y no ineficiencia (§5: una respuesta confiada en un control no resuelto es un fallo).
# Se retrasa afirmar, nunca abstenerse.
#
# resuelve_rol() SIGUE SIENDO PURA: la histeresis vive aqui, en un objeto con estado, para
# que la funcion se pueda seguir aplicando a muestras sueltas y a material ya grabado.

CONF_TICKS = 2       # confirmaciones para ADOPTAR un rol que permite actuar (~0.6 s a 3.3 Hz)
DWELL_S = 1.0        # permanencia minima antes de ceder a otro rol afirmativo
INMEDIATOS = ("review", "defer", "no_use")   # la abstencion se adopta sin esperar


class EstabilizadorRol(object):
    """Envuelve resuelve_rol anadiendo confirmacion y permanencia minima.

    Uso:  est = EstabilizadorRol(); rol, razon, aut, crudo = est.paso(muestra)
    `crudo` es lo que habria dicho el resolutor sin estabilizar, para poder medir el efecto.
    """

    def __init__(self, conf=CONF_TICKS, dwell=DWELL_S, cfg=None):
        self.conf = int(conf); self.dwell = float(dwell); self.cfg = cfg
        self.rol = None; self.razon = ""; self.t_adopcion = None
        self._cand = None; self._n = 0

    def paso(self, m, t=None):
        crudo, razon, aut = resuelve_rol(m, self.cfg)
        if t is None:
            t = m.get("t")
        if self.rol is None:
            self.rol, self.razon, self.t_adopcion = crudo, razon, t
            return self.rol, self.razon, aut, crudo
        if crudo == self.rol:
            self._cand = None; self._n = 0
            return self.rol, self.razon, aut, crudo

        # la abstencion no espera
        if crudo in INMEDIATOS:
            self.rol, self.razon, self.t_adopcion = crudo, razon, t
            self._cand = None; self._n = 0
            return self.rol, self.razon, aut, crudo

        # permanencia minima del rol vigente (salvo que el vigente sea abstencion:
        # salir de una abstencion cuando la evidencia vuelve no debe penalizarse)
        if (self.t_adopcion is not None and t is not None
                and self.rol not in INMEDIATOS and (t - self.t_adopcion) < self.dwell):
            return self.rol, self.razon, aut, crudo

        # confirmacion antes de adoptar
        if crudo == self._cand:
            self._n += 1
        else:
            self._cand = crudo; self._n = 1
        if self._n >= self.conf:
            self.rol, self.razon, self.t_adopcion = crudo, razon, t
            self._cand = None; self._n = 0
        return self.rol, self.razon, aut, crudo
