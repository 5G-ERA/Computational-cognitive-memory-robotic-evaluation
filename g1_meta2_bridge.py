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
    SWITCH_MARGIN = 0.08
    PERSIST_ACTION = 2
    WARMUP = 4          # primeras decisiones: solo observar (arranque con métricas a 0)

    def __init__(self, config_path=None, period=0.5):
        self.reasoner = MetaReasoner20(config_path or DEFAULT_CONFIG)
        self.period = float(period)
        self.last_t = -1e9
        self.last = None          # última salida completa (dict)
        self.n_calls = 0
        self.n_switch = 0
        self.hist = {"safety": [], "progression": []}
        self.applied = self.reasoner.active         # analogía CONFIRMADA (la que gobierna el cap)
        self.pend = None; self.pend_n = 0            # candidato a switch pendiente
        self.act_run = ("", 0)                       # racha de la misma acción cruda

    @staticmethod
    def _med(v):
        s = sorted(v); return s[len(s) // 2]

    def _push(self, k, v):
        h = self.hist[k]; h.append(float(v))
        if len(h) > 5:
            h.pop(0)
        return self._med(h)

    def tick(self, now, clearance, progression, reliability, laser_noise=None, bat=None):
        """Devuelve dict de decisión a ~1/period Hz, o None si toca esperar (throttle)."""
        if now - self.last_t < self.period:
            return None
        self.last_t = now
        rel = max(0.0, min(1.0, float(reliability if reliability is not None else 1.0)))
        unc = 0.02 + 0.10 * max(0.0, min(1.0, float(laser_noise or 0.0)))
        readings = {
            "safety": {"value": self._push("safety", clearance), "reliability": rel, "uncertainty": unc},
            "progression": {"value": self._push("progression", progression), "reliability": rel, "uncertainty": unc},
        }
        if bat is not None:
            try:
                readings["battery_consumption"] = {"value": float(bat) / 100.0, "reliability": 0.99,
                                                   "uncertainty": 0.01}
            except Exception:
                pass
        out = self.reasoner.decide({"timestamp": now, "readings": readings})
        self.n_calls += 1
        raw_active = out.active_after
        s = out.candidate_scores.get(raw_active)
        tens = round(getattr(s, "task_projected_tension", 0.0), 3) if s else None
        ful = round(getattr(s, "task_stable_fulfillment", 0.0), 3) if s else None
        rej = {a: sc.rejection_reason for a, sc in out.candidate_scores.items()
               if sc.rejection_reason}
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
            if self.pend_n >= self.PERSIST_SWITCH:
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
        changed = (self.last is None or act_eff != self.last["action"]
                   or self.applied != self.last["active"])
        self.last = {"action": act_eff, "raw_action": act, "active": self.applied,
                     "raw_active": raw_active, "switch_to": out.switch_to,
                     "tension": tens, "fulfillment": ful, "cap": cap,
                     "rejections": rej, "changed": changed}
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
        o = br.tick(s["t"], c, p, s.get("reliability"), s.get("laser_noise"), s.get("bat"))
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
