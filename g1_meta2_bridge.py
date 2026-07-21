#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
g1_meta2_bridge.py — Puente entre la navegación del G1 y Meta-Reasoner 2.0 (DCA/DCE runtime).

Empaqueta las métricas por tick que YA loguea g1_goto.py (clearance, progression,
reliability, laser_noise, batería) como 'shared experience readings' del Meta-Reasoner 2.0
(paquete meta-reasoner-2.0/, guía en su docx), lo consulta a ~2 Hz (la runtime_config del
reasoner asume frequency_hz=2.0 para la memoria semántica), y devuelve la decisión de
gobernanza: KEEP / SWITCH / FALLBACK / HELP / INSUFFICIENT + la analogía activa.

Mapeo métrica -> meta-parámetro (config_meta2_g1door.json, fronteras QoE calibradas con
las distribuciones REALES de las runs del 2026-07-02: crucero clearance~1.0 prog~0.48;
puerta clearance p25=0.43 mediana 0.60, prog mediana 0.10; pre-colisión clearance
mediana 1.0 -> el láser NO ve venir los golpes: safety no puede ser solo láser):

    safety              <- clearance (SEI, 0..1, c0/1.5m)
    progression         <- progression (SEI, 0..1, tasa de acercamiento/0.3 m/s)
    battery_consumption <- bat/100 (atención 0 en la tarea de puerta: se ignora, ver
                           normalización de atención-cero del reasoner)
    reliability/uncertainty de cada lectura <- SensingMonitor.reliability y laser_noise

Modos (env G1_META2 en g1_goto.py):
    G1_META2=1  SHADOW : decide y loguea, NO toca el control (validación primero).
    G1_META2=2  ACTIVO : además aplica el perfil de la analogía como techo de velocidad
                (Efficient_Nav = sin techo; Cautious_Nav = 0.28; FALLBACK = 0.24;
                HELP = 0.0 en avance — retroceso/giro de las recuperaciones NO se tocan).

Uso desde g1_goto (ya integrado tras el flag):
    from g1_meta2_bridge import Meta2Bridge
    m2 = Meta2Bridge()                       # config por defecto: config_meta2_g1door.json
    out = m2.tick(now, clearance, progression, reliability, laser_noise, bat)
    # out = None (throttle) o dict(action, active, switch_to, tension, fulfillment, cap, changed)

Standalone (probar contra un dataset grabado, sin robot):
    python g1_meta2_bridge.py dataset/20260702_171431_ours_B.json
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, "meta-reasoner-2.0")
if PKG not in sys.path:
    sys.path.insert(0, PKG)

from meta_reasoner_2_0 import MetaReasoner20   # noqa: E402

DEFAULT_CONFIG = os.path.join(HERE, "config_meta2_g1door.json")

# perfil de velocidad por analogía (techo de avance, m/s de stick ly; None = sin techo)
PROFILE_CAP = {
    "Efficient_Nav": None,
    "Cautious_Nav": 0.28,
}
ACTION_CAP = {          # techos por acción cuando NO hay analogía desplegable
    "FALLBACK": 0.24,
    "HELP": 0.0,
    "INSUFFICIENT": None,   # sin grounding: no frenar por sí solo (se loguea)
}

# ===== EXTENSION B (rama analogy-profiles): PERFILES MULTI-PARAMETRO (tesis Tabla 16) =====
# El perfil de la analogia deja de ser solo techo de avance: incluye techo de GIRO (los giros
# de arranque dominan el derrame con taza llena, medido en la sesion real 2026-07-08) y
# holgura del DWA robot_r (la "inflacion" local; rango seguro [0.22, 0.30] — por debajo de
# 0.24 en agresivo vuelven los roces P2). La L3 (Pl_env) modula robot_r = la semantica EXACTA
# de la Layer 3 de la tesis. Opt-in: G1_M2_PROFILES=1. Giro nunca < 0.35 (deadzone ~0.3).
PROFILES_ON = os.environ.get("G1_M2_PROFILES", "0") == "1"
PROFILE_FULL = {
    "Efficient_Nav": {"turn": None, "robot_r": 0.24},
    "Cautious_Nav": {"turn": 0.38, "robot_r": 0.28},
}
ACTION_FULL = {
    "FALLBACK": {"turn": 0.36, "robot_r": 0.28},
    "HELP": {"turn": None, "robot_r": 0.28},
    "INSUFFICIENT": {"turn": None, "robot_r": None},
}

# ===== EXTENSION A (rama analogy-profiles): BIBLIOTECA DE VARIANTES DE PUERTA =====
# Variantes que PARAMETRIZAN el engagement existente (no lo sustituyen). Diseno DATA-DRIVEN
# (mineria 2026-07-08 sobre 279 cruces: limpios |lat| p50=0.06 / yaw_err p50=4 grados; con
# colision p50=0.14/14 — entrar torcido o descentrado predice el golpe; sesgo sistematico
# a -lat en B->A). Trust DST L2 por variante persistido en G1_M2_STATE (clave "_door"):
# cruce limpio -> match; colision en fase puerta o aborto de engagement -> mismatch.
# Seleccion al arranque: mayor Pl (empate -> Door_Direct). Opt-in: G1_M2_DOORLIB=1.
DOORLIB_ON = os.environ.get("G1_M2_DOORLIB", "0") == "1"

# ===== EXTENSION D (rama analogy-profiles): ACOPLAMIENTO GRADUAL fragility -> velocidad =====
# Propuesta de Adrian (2026-07-08): "un algoritmo inteligente deberia reducir la velocidad si
# hay spills". Hoy la respuesta es ESCALONADA (region QoE -> FALLBACK 0.24 -> HELP 0): este
# acoplamiento la hace PROPORCIONAL — un derrame frena ya (escala continua sobre techo de
# avance Y de giro), y se recupera con la fragility (tau ~25s). Los gates/vetos siguen encima
# (3+ derrames -> HELP igual). Tambien frena a Efficient (sin techo -> base 0.40).
# frag>=0.80 -> escala 1.0; frag=0.12 (frontera dangerous) -> escala 0.55. Suelos: avance
# 0.18 (sigue avanzando ~0.08 m/s), giro 0.35 (deadzone). Opt-in: G1_M2_FRAGSPEED=1.
FRAGSPEED_ON = os.environ.get("G1_M2_FRAGSPEED", "0") == "1"
DOOR_VARIANTS = {
    "Door_Direct": {"eng_d": 0.85, "align_tol": 8.0, "lat_bias": 0.0},
    "Door_Far": {"eng_d": 1.20, "align_tol": 6.0, "lat_bias": 0.0},
    "Door_BiasPlus": {"eng_d": 0.85, "align_tol": 8.0, "lat_bias": 0.12},
}


class Meta2Bridge:
    """Suavizado + persistencia BRIDGE-SIDE (el reasoner de Renxi no se toca):
    - lecturas = MEDIANA de las últimas 5 muestras (~2.5 s): la progression SEI instantánea
      parpadea 0.0<->0.5 tick a tick y hacía FALLBACK/SWITCH nerviosos (12 switches en 88 s
      en replay crudo de la run 171431).
    - SWITCH solo se CONFIRMA tras PERSIST decisiones seguidas con el mismo ganador (1.5 s);
      mientras, se mantiene la analogía anterior (el reasoner interno ya conmutó, pero el
      techo de velocidad aplicado espera a la confirmación).
    - FALLBACK/HELP solo actúan (cap) tras 2 seguidas; 1 suelta se loguea como aviso."""

    PERSIST_SWITCH = 3
    PERSIST_RELAX = 6   # vuelta al perfil PREFERIDO: mas lenta (3s) que la huida a Cautious (1.5s).
                        # Run 093510: el stop-and-go del arranque daba 4 switches en 10s con persistencia
                        # simetrica. Histeresis clasica: rapido hacia la cautela, lento hacia la confianza.
    SWITCH_MARGIN = 0.08
    PERSIST_ACTION = 2
    WARMUP = 4          # primeras decisiones: solo observar (arranque con métricas a 0)

    # --- LAYER 2 de la tesis (Cap.5, 5.2.6.1): creencia DST por analogia PERSISTIDA entre runs.
    # G1_M2_STATE=<fichero.json> la activa (vacio = sin memoria entre runs, comportamiento previo).
    # Evidencia al final de cada run: derrame bajo la analogia A -> masa a 'mismatch' de A;
    # run limpia -> masa a 'match' de la analogia mayoritaria. Combinacion por regla de Dempster.
    # La plausibilidad Pl(match)=m_match+m_theta gobierna (bridge-side, reasoner intacto):
    #   Pl < PL_MIN   -> la analogia NO se despliega (se fuerza la mas conservadora desplegable)
    #   Pl < BLEND_PL -> su techo de velocidad se FUNDE hacia el conservador (blend tesis 5.3.4)
    STATE_FILE = os.environ.get("G1_M2_STATE", "")
    # EXTENSION C (rama analogy-profiles): TRANSFERENCIA SIM->REAL del trust analogico.
    # G1_M2_STATE_INIT=<estado_aprendido_en_el_gemelo.json>: si el STATE_FILE del robot aun
    # no tiene evidencia para esta tarea, se SIEMBRA desde el estado del gemelo, remapeando
    # su task_id al de la config actual (p.ej. g1_door_crossing_A_B_sim -> ..._A_B) y
    # trayendo tambien las confianzas de variantes de puerta (_door). El robot real arranca
    # con los priors aprendidos en simulacion = zero-shot con experiencia transferida.
    STATE_INIT = os.environ.get("G1_M2_STATE_INIT", "")
    PL_MIN = float(os.environ.get("G1_M2_PL_MIN", "0.45"))
    # --- CALIBRACION DE LA FRAGILIDAD A LA PLATAFORMA (hallazgo campana real, 2026-07-14) ---
    # El mapeo derrame->fragility y la atribucion DST se calibraron con la estadistica del
    # GEMELO (baseline 0.5 derrames/run). El G1 REAL derrama ~12/run (73 marcas / 6 runs) y
    # aun asi completa la tarea: 3 derrames NO son una catastrofe, son lo normal. Con los
    # valores del gemelo, el 3er derrame -> region dangerous -> veto duro -> HELP -> aborto,
    # SIEMPRE (4/4 runs gobernadas abortaron en la salida, a 246 g y a 200 g).
    # G1_M2_REALCAL=1 aplica la calibracion medida en el laboratorio real.
    _RC = os.environ.get("G1_M2_REALCAL", "") == "1"
    FRAG_DIP = float(os.environ.get("G1_M2_FRAG_DIP", "0.20" if _RC else "0.50"))
    FRAG_LOAD = float(os.environ.get("G1_M2_FRAG_LOAD", "0.06" if _RC else "0.20"))
    FRAG_MAXEXTRA = int(os.environ.get("G1_M2_FRAG_MAXEXTRA", "14" if _RC else "3"))
    # Atribucion DST RELATIVA a la expectativa de la plataforma (0 = semantica absoluta previa):
    # derrames por encima de la referencia -> mismatch; claramente por debajo -> match.
    # Sin esto, en el robot real 'match' es inalcanzable (exige run de 0 derrames) y la
    # analogia conservadora solo puede perder confianza hasta ser vetada -> el veto promociona
    # la analogia MAS ARRIESGADA (observado: run 20260714_163627 desplego Efficient_Nav).
    SPILL_REF = float(os.environ.get("G1_M2_SPILL_REF", "12" if _RC else "0"))
    MATCH_MIN_S = float(os.environ.get("G1_M2_MATCH_MIN_S", "30" if _RC else "10"))
    BLEND_PL = 0.70
    CONSERVATIVE = "Cautious_Nav"          # analogia refugio (el M1 del blend)
    CONSERVATIVE_CAP = 0.28

    def __init__(self, config_path=None, period=0.5):
        self.config_path = config_path or DEFAULT_CONFIG
        self.reasoner = MetaReasoner20(self.config_path)
        self.period = float(period)
        self.last_t = -1e9
        self.last = None          # última salida completa (dict)
        self.n_calls = 0
        self.n_switch = 0
        self.hist = {"safety": [], "progression": [], "rel": [], "nz": [], "mobility": []}
        self.applied = self.reasoner.active         # analogía CONFIRMADA (la que gobierna el cap)
        self.pend = None; self.pend_n = 0            # candidato a switch pendiente
        self.act_run = ("", 0)                       # racha de la misma acción cruda
        # --- canal FRAGILITY (payload): solo si la config declara el meta-parametro ---
        try:
            _cfg = json.load(open(self.config_path))
        except Exception:
            _cfg = {}
        self._has_frag = "fragility" in (_cfg.get("overall_meta_parameters") or [])
        self._task_id = ((_cfg.get("task_information") or {}).get("task_id")) or "task"
        self._analogies = list((_cfg.get("analogies") or {}).keys())
        self.hist["fragility"] = []
        self._last_spill_n = 0
        # --- rho_DCA runtime (paper Secc. VI-VII): margen de arbitraje / presupuesto de
        # perturbacion, instanciacion runtime DOCUMENTADA: margen = stable_ful(mejor desplegable)
        # - stable_ful(2o desplegable) (o distancia al umbral si solo hay uno); presupuesto =
        # task_uncertainty_gap del ganador (proxy de theta_QoE) + 0.5*theta_mismatch (norma L2
        # entre atenciones de tarea y de la analogia, estatico de config; B=1, xi_rep=0 declarado
        # no medible en runtime). rho>=1 estable, ~1 frontera, <1 no certificable.
        _ti = (_cfg.get("task_information") or {})
        _ta = dict(_ti.get("task_meta_attentions") or {})
        self._theta_mm = {}
        self._safety_exp = {}
        for _a, _ad in (_cfg.get("analogies") or {}).items():
            _aa = dict(_ad.get("meta_attentions") or {})
            keys = set(_ta) | set(_aa)
            st_ = sum(abs(_ta.get(k, 0.0)) for k in keys) or 1.0
            sa_ = sum(abs(_aa.get(k, 0.0)) for k in keys) or 1.0
            self._theta_mm[_a] = sum((_ta.get(k, 0.0) / st_ - _aa.get(k, 0.0) / sa_) ** 2
                                     for k in keys) ** 0.5
            try:
                self._safety_exp[_a] = float(_ad["qoe"]["safety"]["expected"])
            except Exception:
                self._safety_exp[_a] = 0.6
        # --- LAYER 3 (tesis 5.2.6.1, Pl_env online): escala de entorno de la media movil del
        # clearance RELATIVA al 'expected' de la analogia activa; modula el techo de velocidad
        # (relaja si el entorno es mas abierto de lo que la analogia asume; aprieta si mas
        # angosto), CLAMPEADA [0.85, 1.15] y solo sobre techos de PERFIL (no FALLBACK/HELP).
        # Opt-in: G1_M2_L3=1 (default 0: no altera las campanas ya publicadas).
        self.L3 = os.environ.get("G1_M2_L3", "0") == "1"
        self._env = 1.0
        # --- LAYER 4 (tesis 5.2.6.1, Meta-Attention): si en las ultimas L4_WIN runs el
        # fulfillment medio fue bajo PESE a plausibilidad L2 alta -> el PERFIL de tarea esta
        # mal elegido (mismatch de perfil, no de analogia) -> arrancar en la alternativa.
        # Requiere memoria entre runs (G1_M2_STATE). Opt-in: G1_M2_L4=1.
        self.L4 = os.environ.get("G1_M2_L4", "0") == "1"
        self.L4_WIN = int(os.environ.get("G1_M2_L4_WIN", "3"))
        self.L4_FUL = float(os.environ.get("G1_M2_L4_FUL", "0.45"))
        self._ful_sum = 0.0
        self._ful_n = 0
        # --- estado Layer 2 (cross-run) ---
        self._l2 = None
        self._run_spills = {}      # analogia -> derrames atribuidos en ESTA run
        self._run_time = {}        # analogia -> segundos gobernando en ESTA run
        self.door_variant = None   # EXTENSION A (requiere G1_M2_STATE + G1_M2_DOORLIB)
        self._door_results = []    # [(ok, cols)] de la run en curso
        if self.STATE_FILE:
            self._l2 = self._l2_load()
            # arranque ZERO-SHOT corregido: si la analogia inicial esta desacreditada
            # (Pl<PL_MIN) y hay otra con mas plausibilidad, EMPEZAR en la mejor (la firma
            # M2+Wrong->DST de la tesis: tras k runs malas, la k+1 arranca ya corregida).
            pl0 = self._l2_pl(self.applied)
            best = max(self._analogies, key=lambda a: self._l2_pl(a)) if self._analogies else None
            if best and pl0 < self.PL_MIN and self._l2_pl(best) > pl0:
                print(f"  [META2-L2] analogia inicial '{self.applied}' desacreditada "
                      f"(Pl={pl0:.2f}); arrancando en '{best}' (Pl={self._l2_pl(best):.2f})")
                self.applied = best
            # LAYER 4: mismatch de PERFIL — fulfillment cronicamente bajo con confianza alta
            if self.L4:
                hist = self._l2.get(self._task_id, {}).get("_ful_hist", [])
                if len(hist) >= self.L4_WIN:
                    w = hist[-self.L4_WIN:]
                    low_ful = all(h.get("ful", 1.0) < self.L4_FUL for h in w)
                    pl_ok = all(self._l2_pl(h.get("applied", "")) >= self.PL_MIN for h in w)
                    same = {h.get("applied") for h in w}
                    if low_ful and pl_ok and len(same) == 1:
                        cur = w[-1].get("applied")
                        alts = [x for x in self._analogies if x != cur]
                        if alts:
                            alt = max(alts, key=lambda x: self._l2_pl(x))
                            print(f"  [META2-L4] PERFIL equivocado: ful medio <{self.L4_FUL} "
                                  f"en {self.L4_WIN} runs con Pl alta bajo '{cur}' -> "
                                  f"re-seleccion: arrancando en '{alt}'")
                            self.applied = alt
            # EXTENSION A: seleccionar la variante de puerta con mas confianza (Pl)
            if DOORLIB_ON:
                dst_ = self._l2.setdefault("_door", {})
                for v in DOOR_VARIANTS:
                    dst_.setdefault(v, {"m_match": 0.5, "m_mismatch": 0.0, "m_theta": 0.5,
                                        "crossings": 0, "fails": 0})
                def _plv(v):
                    m = dst_[v]
                    return m["m_match"] + m["m_theta"]
                order = list(DOOR_VARIANTS)          # empate -> orden de definicion (Direct 1o)
                if os.environ.get("G1_M2_DOOR_EXPLORE", "") == "1":
                    # EXPLORACION ACTIVA (Renxi 2026-07-21: "exploration" del high-level policy).
                    # La seleccion por Pl es optimista (Th cuenta a favor) pero DEGENERA: si la
                    # incumbente nunca falla, las alternativas quedan a 0 ensayos (A/B formal:
                    # Direct 8/8, Far/BiasPlus 0). En el GEMELO fallar es gratis -> se elige la
                    # variante MENOS ensayada (round-robin auto-balanceado via estado persistido)
                    # y el trust poblado se transfiere al robot real por Ext C (G1_M2_STATE_INIT).
                    # SOLO para sesiones de exploracion en sim; el real siempre explota por Pl.
                    self.door_variant = min(order, key=lambda v: (dst_[v]["crossings"] + dst_[v]["fails"],
                                                                  order.index(v)))
                    print(f"  [META2-DOOR] EXPLORACION: variante menos ensayada -> {self.door_variant} "
                          f"(ensayos={{{', '.join('%s:%d' % (v, dst_[v]['crossings'] + dst_[v]['fails']) for v in order)}}})")
                else:
                    self.door_variant = max(order, key=lambda v: (round(_plv(v), 3), -order.index(v)))
                    print(f"  [META2-DOOR] variante de engagement: {self.door_variant} "
                          f"(Pl={{{', '.join('%s:%.2f' % (v, _plv(v)) for v in order)}}})")
            import atexit
            atexit.register(self.end_run)

    # ---------- Layer 2: persistencia DST ----------
    def _l2_load(self):
        try:
            st = json.load(open(self.STATE_FILE))
        except Exception:
            st = {}
        # EXTENSION C: sembrar desde el estado del gemelo si esta tarea aun no tiene evidencia
        if self.STATE_INIT and not st.get(self._task_id):
            try:
                ini = json.load(open(self.STATE_INIT))
                src_tasks = [k for k in ini if not k.startswith("_")]
                if src_tasks:
                    st[self._task_id] = ini[src_tasks[0]]        # remapeo de task_id
                if "_door" in ini and "_door" not in st:
                    st["_door"] = ini["_door"]
                pls = {a_: round(v.get("m_match", 0) + v.get("m_theta", 0), 2)
                       for a_, v in st.get(self._task_id, {}).items()
                       if isinstance(v, dict) and "m_match" in v}
                print(f"  [META2-C] trust TRANSFERIDO del gemelo ({self.STATE_INIT}): "
                      f"{src_tasks[0] if src_tasks else '?'} -> {self._task_id}, Pl={pls}"
                      + (", _door incluido" if "_door" in ini else ""))
            except Exception as e:
                print("  [META2-C] transferencia fallida:", repr(e))
        st.setdefault(self._task_id, {})
        for a in self._analogies:
            st[self._task_id].setdefault(a, {"m_match": 0.5, "m_mismatch": 0.0,
                                             "m_theta": 0.5, "runs": 0, "spills": 0})
        return st

    def _l2_pl(self, analogy):
        # plausibilidad de 'la analogia sigue siendo valida' = m_match + m_theta
        if not self._l2:
            return 1.0
        m = self._l2.get(self._task_id, {}).get(analogy)
        return (m["m_match"] + m["m_theta"]) if m else 1.0

    @staticmethod
    def _dempster(m, ev):
        # combinacion de Dempster sobre {match, mismatch, theta}; ev = masas de la evidencia
        k = m["m_match"] * ev.get("mismatch", 0.0) + m["m_mismatch"] * ev.get("match", 0.0)
        if k >= 0.999:
            return m
        n = 1.0 / (1.0 - k)
        th_e = 1.0 - ev.get("match", 0.0) - ev.get("mismatch", 0.0)
        return {"m_match": n * (m["m_match"] * ev.get("match", 0.0) + m["m_match"] * th_e
                                + m["m_theta"] * ev.get("match", 0.0)),
                "m_mismatch": n * (m["m_mismatch"] * ev.get("mismatch", 0.0) + m["m_mismatch"] * th_e
                                   + m["m_theta"] * ev.get("mismatch", 0.0)),
                "m_theta": n * (m["m_theta"] * th_e)}

    def door_result(self, ok, cols=0):
        """EXTENSION A: resultado de una fase de puerta (llamado por g1_goto en door_crossed
        o en un aborto de engagement). Evidencia DST inmediata a la variante activa."""
        self._door_results.append((bool(ok), int(cols)))
        if not (self._l2 and self.door_variant):
            return
        m = self._l2.get("_door", {}).get(self.door_variant)
        if not m:
            return
        if ok and cols == 0:
            ev = {"match": 0.30}
            m["crossings"] = m.get("crossings", 0) + 1
        else:
            ev = {"mismatch": min(0.5, 0.30 + 0.10 * max(0, cols - 1))}
            m["fails"] = m.get("fails", 0) + 1
        m.update(self._dempster(m, ev))
        pl = m["m_match"] + m["m_theta"]
        print(f"  [META2-DOOR] {self.door_variant}: {'CRUZADA' if ok and cols==0 else 'FALLO'} "
              f"(cols={cols}) -> Pl={pl:.2f}")

    def end_run(self):
        # cierre de run (atexit con G1_M2_STATE): convertir el resultado en evidencia DST.
        # Derrames bajo A -> mismatch de A; run totalmente limpia -> match de la mayoritaria.
        if not self._l2 or self._l2.get("_closed"):
            return
        st = self._l2[self._task_id]
        total_spills = sum(self._run_spills.values())
        for a, s in self._run_spills.items():
            if a not in st:
                continue
            if self.SPILL_REF > 0:
                # RELATIVA: la evidencia se mide contra lo que la plataforma hace de serie.
                # Exposicion minima (MATCH_MIN_S) para premiar: un aborto temprano no informa.
                if s > self.SPILL_REF:
                    ev = {"mismatch": min(0.5, 0.10 + 0.05 * (s - self.SPILL_REF))}
                    st[a].update(self._dempster(st[a], ev))
                elif s <= 0.5 * self.SPILL_REF and self._run_time.get(a, 0.0) >= self.MATCH_MIN_S:
                    st[a].update(self._dempster(st[a], {"match": 0.25}))
                if s:
                    st[a]["spills"] = st[a].get("spills", 0) + s
            elif s > 0:
                ev = {"mismatch": min(0.5, 0.30 + 0.10 * (s - 1))}
                st[a].update(self._dempster(st[a], ev))
                st[a]["spills"] = st[a].get("spills", 0) + s
        if self.SPILL_REF <= 0 and total_spills == 0 and self._run_time:
            maj = max(self._run_time, key=self._run_time.get)
            if maj in st and self._run_time[maj] >= self.MATCH_MIN_S:
                st[maj].update(self._dempster(st[maj], {"match": 0.25}))
        for a in self._run_time:
            if a in st:
                st[a]["runs"] = st[a].get("runs", 0) + 1
        if self._ful_n:
            maj = max(self._run_time, key=self._run_time.get) if self._run_time else None
            h = st.setdefault("_ful_hist", [])
            h.append({"ful": round(self._ful_sum / self._ful_n, 3),
                      "applied": maj, "pl": round(self._l2_pl(maj), 3) if maj else None,
                      "spills": total_spills})
            del h[:-10]
        self._l2["_closed"] = True
        try:
            out = {k: v for k, v in self._l2.items() if k != "_closed"}
            json.dump(out, open(self.STATE_FILE, "w"), indent=1)
            pls = {a: round(self._l2_pl(a), 2) for a in self._analogies}
            print(f"  [META2-L2] estado guardado ({self.STATE_FILE}): Pl={pls} "
                  f"derrames_run={total_spills}")
        except Exception as e:
            print("  [META2-L2] no se pudo guardar estado:", repr(e))

    # peso de la CONFIANZA HISTORICA en la incertidumbre DST (Renxi 2026-07-03: "the robot might
    # feel the analogy is highly plausible but still not good enough in reality... add historical
    # confidence into the task; currently one-shot decision"). La incertidumbre de cada lectura ya
    # no es solo el ruido instantaneo: suma k * desviacion tipica de la PROPIA metrica en la ventana
    # (~2.5s) -> una metrica que OSCILA sobre una frontera QoE ensancha su intervalo belief/
    # plausibility, cae la belief_fulfillment (la base del gate) y crece el uncertainty_gap que
    # penaliza el ranking: la plausibilidad exige CONSISTENCIA, no una lectura afortunada =
    # decision few-shot. G1_M2_HIST_K=0 revierte al comportamiento one-shot.
    HIST_K = float(os.environ.get("G1_M2_HIST_K", "0.4"))
    # ^ 0.4 elegido por A/B offline sobre las 7 runs del 07-03: con 0.8 el FALLBACK en runs limpias
    #   subia demasiado (093703: 15->34%); con 0.4 sube moderado (15->24%), la run mala queda
    #   claramente separada (72%) y la escalada P9 mantiene 0 abortos falsos / aborta la mala a t=93s.
    UNC_CAP = 0.35            # techo del margen (que una rafaga no anule toda la evidencia)

    @staticmethod
    def _med(v):
        s = sorted(v); return s[len(s) // 2]

    @staticmethod
    def _std(v):
        if len(v) < 3:
            return 0.0
        m = sum(v) / len(v)
        return (sum((x - m) ** 2 for x in v) / len(v)) ** 0.5

    def _push(self, k, v):
        h = self.hist[k]; h.append(float(v))
        if len(h) > 5:
            h.pop(0)
        return self._med(h)

    def tick(self, now, clearance, progression, reliability, laser_noise=None, bat=None,
             hold_progression=False, mobility=None, spill_dt=None, spill_count=0):
        """Devuelve dict de decisión a ~1/period Hz, o None si toca esperar (throttle).
        hold_progression=True cuando el robot esta PARADO A PROPOSITO (alineando en el
        engagement, girando en el sitio): la progression=0 de ese momento es comandada, no
        un atasco -> se CONGELA en su ultima mediana real en vez de reportar estancamiento
        falso (run 093841: 41%% de ticks FALLBACK, casi todos durante ENG-AL/giros)."""
        if now - self.last_t < self.period:
            return None
        self.last_t = now
        if hold_progression and self.hist["progression"]:
            progression = self._med(self.hist["progression"])
        # reliability/noise tambien con MEDIANA de 5: la rel real tiene rafagas de 1-2 ticks a 0.0
        # (nz se dispara al girar) y el valor instantaneo inflaba el margen DST a golpes.
        rel = self._push("rel", max(0.0, min(1.0, float(reliability if reliability is not None else 1.0))))
        unc = 0.02 + 0.10 * self._push("nz", max(0.0, min(1.0, float(laser_noise or 0.0))))

        def reading(key, value):
            """Lectura con margen FEW-SHOT: incertidumbre = ruido instantaneo + k*std historica."""
            med = self._push(key, value)
            u = min(self.UNC_CAP, unc + self.HIST_K * self._std(self.hist[key]))
            return {"value": med, "reliability": rel, "uncertainty": u}

        readings = {
            "safety": reading("safety", clearance),
            "progression": reading("progression", progression),
        }
        # --- canal de RESISTENCIA (supervisor 2026-07-03: "meta-attention to clearance but not to
        # resistance"). mobility = velocidad real / velocidad comandada (1=se mueve como se le pide,
        # ~0=EMPUJANDO algo que el laser no ve: la firma pre-impacto medida es 2-3.5s a ratio <0.3
        # con clearance 1.0). Va como meta-parametro con HARD VETO: presion sostenida = analogia
        # invalida -> HELP -> (con la escalada) ABORT. Sin comando de avance no hay evidencia:
        # deriva suavemente hacia 1.0 (sin resistencia conocida).
        if mobility is None:
            last = self.hist["mobility"][-1] if self.hist["mobility"] else 1.0
            mobility = last + 0.15 * (1.0 - last)
        _mmed = self._push("mobility", max(0.0, min(1.0, float(mobility))))
        readings["mobility"] = {"value": _mmed, "reliability": 0.95,
                                "uncertainty": min(self.UNC_CAP,
                                                   0.03 + self.HIST_K * self._std(self.hist["mobility"]))}
        if bat is not None:
            try:
                readings["battery_consumption"] = {"value": float(bat) / 100.0, "reliability": 0.99,
                                                   "uncertainty": 0.01}
            except Exception:
                pass
        # --- canal FRAGILITY (payload, tesis Cap.5): el derrame ES el observable de fragilidad.
        # 1.0 = carga intacta; cada derrame hunde la lectura (recupera con tau~25s); derrames
        # repetidos la hunden mas (3+ recientes -> region dangerous -> hard veto -> HELP).
        # Solo si la config declara 'fragility' (si no, ni se envia: grounding limpio).
        if self._has_frag:
            frag = 1.0
            if spill_dt is not None:
                dt_ = max(0.0, spill_dt)
                extra = max(0, int(spill_count) - 1)
                # dip del ultimo derrame (tau=25s) + carga acumulada por reincidencia (tau=50s):
                # 1 derrame -> ~0.51 (precaucion), 2 -> ~0.31 (FALLBACK), 3+ -> <0.12 (HELP)
                frag -= self.FRAG_DIP * math.exp(-dt_ / 25.0)
                frag -= self.FRAG_LOAD * min(extra, self.FRAG_MAXEXTRA) * math.exp(-dt_ / 50.0)
            frag = max(0.05, frag)
            # SIN mediana: el derrame es un EVENTO cierto (ground truth), no ruido de sensor.
            # La historia se guarda solo para el margen few-shot (std) del intervalo DST.
            self._push("fragility", frag)
            readings["fragility"] = {"value": frag, "reliability": 0.98,
                                     "uncertainty": min(self.UNC_CAP,
                                                        0.02 + self.HIST_K * self._std(self.hist["fragility"]))}
        # --- Layer 2: atribuir derrames NUEVOS a la analogia que gobierna ahora ---
        if self._l2 is not None:
            new_sp = max(0, int(spill_count) - self._last_spill_n)
            if new_sp:
                self._run_spills[self.applied] = self._run_spills.get(self.applied, 0) + new_sp
            self._last_spill_n = int(spill_count)
            self._run_time[self.applied] = self._run_time.get(self.applied, 0.0) + self.period
        out = self.reasoner.decide({"timestamp": now, "readings": readings})
        self.n_calls += 1
        raw_active = out.active_after
        s = out.candidate_scores.get(raw_active)
        tens = round(getattr(s, "task_projected_tension", 0.0), 3) if s else None
        ful = round(getattr(s, "task_stable_fulfillment", 0.0), 3) if s else None
        rej = {a: sc.rejection_reason for a, sc in out.candidate_scores.items()
               if sc.rejection_reason}
        # --- rho_DCA runtime (ver __init__): margen de arbitraje / presupuesto de perturbacion
        rho = None
        try:
            dep = sorted(((a, sc.task_stable_fulfillment) for a, sc in out.candidate_scores.items()
                          if sc.deployable), key=lambda x: -x[1])
            thr_ = float(self.reasoner.task.get("task_fulfillment_threshold", 0.35))
            if len(dep) >= 2:
                margin = dep[0][1] - dep[1][1]
            elif len(dep) == 1:
                margin = dep[0][1] - thr_
            else:
                margin = 0.0
            win_ = dep[0][0] if dep else raw_active
            gap_ = float(getattr(out.candidate_scores.get(win_), "task_uncertainty_gap", 0.0) or 0.0)
            budget = gap_ + 0.5 * self._theta_mm.get(win_, 0.0) + 0.02
            rho = round(max(0.0, margin) / budget, 2)
        except Exception:
            pass
        # acumulador de fulfillment de la analogia APLICADA (Layer 4, media por run)
        _sap = out.candidate_scores.get(self.applied)
        if _sap is not None:
            self._ful_sum += float(_sap.task_stable_fulfillment or 0.0)
            self._ful_n += 1
        # --- Layer 2: veto por plausibilidad cross-run (la analogia desacreditada no se
        # despliega aunque el reasoner instantaneo la prefiera; se redirige a la desplegable
        # con mas Pl — normalmente la conservadora). Bridge-side: el reasoner no se toca.
        if self._l2 is not None and out.action in ("KEEP", "SWITCH"):
            if self._l2_pl(raw_active) < self.PL_MIN:
                cands = [a for a, sc in out.candidate_scores.items()
                         if sc.deployable and self._l2_pl(a) >= self.PL_MIN]
                if cands:
                    alt = max(cands, key=lambda a: self._l2_pl(a))
                    rej = dict(rej); rej[raw_active] = "layer2_trust"
                    raw_active = alt
        # --- persistencia de accion (FALLBACK/HELP) ---
        act = out.action
        self.act_run = (act, self.act_run[1] + 1) if act == self.act_run[0] else (act, 1)
        act_firm = self.act_run[1] >= self.PERSIST_ACTION or act in ("KEEP", "SWITCH")
        # --- margen de switch (switch_sensitivity de la config v1 de Renxi, aplicado en el bridge):
        # si la analogía CONFIRMADA sigue desplegable y la candidata no le saca >= SWITCH_MARGIN de
        # stable_fulfillment, NO hay switch pendiente (mata el ping-pong por empates en abierto).
        ap = out.candidate_scores.get(self.applied)
        cs = out.candidate_scores.get(raw_active)
        if (act in ("KEEP", "SWITCH") and raw_active != self.applied and ap is not None
                and ap.deployable and cs is not None
                and (cs.task_stable_fulfillment - ap.task_stable_fulfillment) < self.SWITCH_MARGIN):
            raw_active = self.applied
            act = "KEEP"
        # --- retorno al perfil PREFERIDO de la tarea: si estamos en otro y el preferido vuelve a
        # estar desplegable con fulfillment empatado (<=0.02 de diferencia), candidatearlo (pasa por
        # la persistencia normal). Sin esto, tras un switch a Cautious el empate en abierto lo dejaba
        # clavado en 0.28 para siempre.
        pref = self.reasoner.task.get("preferred_analogy")
        if act == "KEEP" and pref and pref != self.applied:
            ps_ = out.candidate_scores.get(pref)
            if (ps_ is not None and ps_.deployable and ap is not None and ap.deployable
                    and (ap.task_stable_fulfillment - ps_.task_stable_fulfillment) <= 0.02):
                raw_active = pref
        # --- persistencia de switch: confirmar solo tras PERSIST_SWITCH ganadores iguales ---
        if act in ("KEEP", "SWITCH") and raw_active != self.applied:
            if self.pend == raw_active:
                self.pend_n += 1
            else:
                self.pend, self.pend_n = raw_active, 1
            need = self.PERSIST_RELAX if raw_active == pref else self.PERSIST_SWITCH
            if self.pend_n >= need:
                self.applied = raw_active; self.pend = None; self.pend_n = 0
                self.n_switch += 1
                confirmed_switch = True
            else:
                confirmed_switch = False
        else:
            self.pend = None; self.pend_n = 0
            confirmed_switch = False
        # --- techo de velocidad efectivo ---
        if self.n_calls <= self.WARMUP:
            cap = None; act_eff = "WARMUP"
        elif act in ("FALLBACK", "HELP", "INSUFFICIENT"):
            cap = ACTION_CAP.get(act) if act_firm else PROFILE_CAP.get(self.applied)
            act_eff = act if act_firm else act + "?"
        else:
            cap = PROFILE_CAP.get(self.applied)
            act_eff = "SWITCH" if confirmed_switch else ("KEEP" if raw_active == self.applied else "PEND")
        # --- Layer 2: BLEND del techo hacia el conservador segun plausibilidad (tesis 5.3.4:
        # politica = interpolacion continua analogia<->M1 pesada por la confianza en la analogia).
        pl_a = self._l2_pl(self.applied) if self._l2 is not None else None
        if (pl_a is not None and pl_a < self.BLEND_PL and act_eff not in ("WARMUP",)
                and self.applied != self.CONSERVATIVE):
            a_cap = cap if cap is not None else 0.40          # sin techo ~ stick 0.40
            w = max(0.0, (pl_a - self.PL_MIN) / (self.BLEND_PL - self.PL_MIN))
            blended = self.CONSERVATIVE_CAP + w * (a_cap - self.CONSERVATIVE_CAP)
            cap = min(a_cap, blended)
        # --- LAYER 3: relevancia de entorno (tesis Pl_env). EMA del clearance mediano relativo
        # al 'expected' de la analogia activa; entorno abierto (>1) relaja el techo de perfil,
        # angosto (<1) lo aprieta. Clamp [0.85, 1.15]; nunca toca FALLBACK/HELP ni supera 0.40.
        env_scale = None
        if self.L3:
            med_c = self._med(self.hist["safety"]) if self.hist["safety"] else 1.0
            exp_b = self._safety_exp.get(self.applied, 0.6) or 0.6
            self._env += 0.10 * (med_c / exp_b - self._env)
            env_scale = max(0.85, min(1.15, self._env))
            if (cap is not None and cap > 0.0
                    and act_eff not in ("WARMUP",) and act not in ("FALLBACK", "HELP", "INSUFFICIENT")):
                cap = min(0.40, max(0.22, round(cap * env_scale, 3)))
        # EXTENSION B: perfil multi-parametro (giro + holgura DWA), con L3 sobre robot_r
        _turn = None; _rr = None
        if PROFILES_ON:
            if act_firm and act in ("FALLBACK", "HELP", "INSUFFICIENT"):
                _pf = ACTION_FULL.get(act, {})
            else:
                _pf = PROFILE_FULL.get(self.applied, {})
            _turn = _pf.get("turn")
            _rr = _pf.get("robot_r")
            if _rr is not None and env_scale is not None:
                # entorno abierto (env>1) -> holgura mas fina (rutas mas directas); angosto -> mas ancha
                _rr = max(0.22, min(0.30, round(_rr / env_scale, 3)))
        # EXTENSION D: escala gradual por fragility sobre avance y giro
        if FRAGSPEED_ON and self._has_frag:
            _fr = readings.get("fragility", {}).get("value", 1.0)
            if _fr < 0.80:
                _sc = max(0.55, 0.55 + 0.45 * (_fr - 0.12) / (0.80 - 0.12))
                _base = cap if cap is not None else 0.40
                if _base > 0.0:
                    cap = max(0.18, round(_base * _sc, 3))
                if _turn is not None:
                    _turn = max(0.35, round(_turn * _sc, 3))
        changed = (self.last is None or act_eff != self.last["action"]
                   or self.applied != self.last["active"])
        self.last = {"action": act_eff, "raw_action": act, "active": self.applied,
                     "raw_active": raw_active, "switch_to": out.switch_to,
                     "tension": tens, "fulfillment": ful, "cap": cap,
                     "rejections": rej, "changed": changed,
                     "pl": round(pl_a, 3) if pl_a is not None else None,
                     "frag": readings.get("fragility", {}).get("value") if self._has_frag else None,
                     "rho": rho,
                     "env": round(env_scale, 3) if env_scale is not None else None,
                     "turn": _turn, "robot_r": _rr,
                     # incertidumbre DST por parametro (la dispersion empirica que juega el papel
                     # de la covarianza en los intervalos): se loguea por muestra (meta2_unc)
                     "unc": {k: round(v.get("uncertainty", 0.0), 3) for k, v in readings.items()},
                     "door": (dict(name=self.door_variant, **DOOR_VARIANTS[self.door_variant])
                              if (DOORLIB_ON and self.door_variant) else None)}
        return self.last

    def summary(self):
        return {"meta2_calls": self.n_calls, "meta2_switches": self.n_switch,
                "meta2_last": (self.last or {}).get("active")}


def _replay(path):
    """Replay offline: pasa las samples de una run grabada por el reasoner y resume decisiones."""
    d = json.load(open(path))
    br = Meta2Bridge()
    seq = []
    for s in d.get("samples", []):
        c = s.get("clearance"); p = s.get("progression")
        if c is None or p is None:
            continue
        php = str(s.get("phase", "")).replace("AGR-", "")
        hold = php.startswith(("ENG-T", "ENG-AL", "ENG-RE", "ENG-WT", "ENG-C",
                               "DOOR-AL", "DOOR-WT", "DOOR-CTR", "DWA-T", "SEEK-T"))
        o = br.tick(s["t"], c, p, s.get("reliability"), s.get("laser_noise"), s.get("bat"),
                    hold_progression=hold)
        if o and (o["changed"] or o["action"] not in ("KEEP",)):
            seq.append((round(s["t"], 1), o["action"], o["active"], o["tension"], o["fulfillment"]))
    print(f"{os.path.basename(path)}: {br.n_calls} decisiones, {br.n_switch} switches")
    for row in seq[:60]:
        print("  t=%6.1fs %-12s activo=%-14s tens=%s ful=%s" % row)
    cols = [e["t"] for e in d.get("events", []) if e.get("kind") == "collision"]
    if cols:
        print("  colisiones reales en t=", [round(t, 1) for t in cols])


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(0)
    for p in sys.argv[1:]:
        _replay(p)
