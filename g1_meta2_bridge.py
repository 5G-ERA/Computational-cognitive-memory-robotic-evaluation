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
    PL_MIN = float(os.environ.get("G1_M2_PL_MIN", "0.45"))
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
        # --- estado Layer 2 (cross-run) ---
        self._l2 = None
        self._run_spills = {}      # analogia -> derrames atribuidos en ESTA run
        self._run_time = {}        # analogia -> segundos gobernando en ESTA run
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
            import atexit
            atexit.register(self.end_run)

    # ---------- Layer 2: persistencia DST ----------
    def _l2_load(self):
        try:
            st = json.load(open(self.STATE_FILE))
        except Exception:
            st = {}
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

    def end_run(self):
        # cierre de run (atexit con G1_M2_STATE): convertir el resultado en evidencia DST.
        # Derrames bajo A -> mismatch de A; run totalmente limpia -> match de la mayoritaria.
        if not self._l2 or self._l2.get("_closed"):
            return
        st = self._l2[self._task_id]
        total_spills = sum(self._run_spills.values())
        for a, s in self._run_spills.items():
            if s > 0 and a in st:
                ev = {"mismatch": min(0.5, 0.30 + 0.10 * (s - 1))}
                st[a].update(self._dempster(st[a], ev))
                st[a]["spills"] = st[a].get("spills", 0) + s
        if total_spills == 0 and self._run_time:
            maj = max(self._run_time, key=self._run_time.get)
            if maj in st and self._run_time[maj] >= 10.0:
                st[maj].update(self._dempster(st[maj], {"match": 0.25}))
        for a in self._run_time:
            if a in st:
                st[a]["runs"] = st[a].get("runs", 0) + 1
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
                frag -= 0.50 * math.exp(-dt_ / 25.0)
                frag -= 0.20 * min(extra, 3) * math.exp(-dt_ / 50.0)
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
        changed = (self.last is None or act_eff != self.last["action"]
                   or self.applied != self.last["active"])
        self.last = {"action": act_eff, "raw_action": act, "active": self.applied,
                     "raw_active": raw_active, "switch_to": out.switch_to,
                     "tension": tens, "fulfillment": ful, "cap": cap,
                     "rejections": rej, "changed": changed,
                     "pl": round(pl_a, 3) if pl_a is not None else None,
                     "frag": readings.get("fragility", {}).get("value") if self._has_frag else None}
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
