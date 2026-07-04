#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_campaign.py — Agregado de la campana payload para la tabla del paper (seccion VII).

Agrupa campaign_results.json por brazo (excluye runs marcadas 'invalid'), y reporta por brazo:
media +- IC95% (1.96*s/sqrt(n), como la tesis 5.3.1.4) de tiempo, derrames, E[derrames],
%riesgo; tasas de reached/abort; y para los brazos *dst* la SECUENCIA por run (curva de
recuperacion) + el estado Layer 2 final.

USO: python3 analyze_campaign.py
"""
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
ORDER = ["base", "conservfix", "conserv", "meta2", "wrong", "wrongsim", "wrongdst",
         "wrongdstsim", "baseclosed", "m2closed", "m2closedsim"]
LABEL = {
    "base": "M0 base (0.30, sin gobernanza)",
    "conservfix": "M1 conservador fijo (0.28 techo, sin gobernanza)",
    "conserv": "Cautious-only gobernado (mono-analogia)",
    "meta2": "M2 payload (META2 activo, config payload)",
    "wrong": "M2+Wrong-Locked (prior puerta real, sin memoria)",
    "wrongsim": "M2+Wrong sim-calibrado (sin memoria)",
    "wrongdst": "M2+Wrong->DST (prior puerta real + Layer2)",
    "wrongdstsim": "M2+Wrong->DST sim-calibrado (Layer2)",
    "baseclosed": "M0 tapa CERRADA",
    "m2closed": "M2 tapa CERRADA (config puerta real)",
    "m2closedsim": "M2 tapa CERRADA (config sim)",
}


def ci(vals):
    n = len(vals)
    if n == 0:
        return (float("nan"), 0.0, 0)
    m = sum(vals) / n
    if n < 2:
        return (m, 0.0, n)
    s = (sum((x - m) ** 2 for x in vals) / (n - 1)) ** 0.5
    return (m, 1.96 * s / math.sqrt(n), n)


def main():
    rows = json.load(open(os.path.join(HERE, "campaign_results.json")))
    rows = [r for r in rows if not r.get("invalid")]
    arms = {}
    for r in rows:
        arms.setdefault(r["arm"], []).append(r)
    print("%-46s %2s  %-9s %-14s %-12s %-13s %-10s" %
          ("brazo", "n", "reached", "t (s)", "derrames/run", "E[derr]/run", "riesgo %"))
    print("-" * 115)
    for a in ORDER:
        rs = arms.get(a)
        if not rs:
            continue
        reached = sum(1 for r in rs if r.get("result") == "reached")
        aborts = sum(1 for r in rs if str(r.get("result", "")).startswith("aborted"))
        # tiempos: solo runs reached (los abortos van aparte)
        t = ci([r["time_s"] for r in rs if r.get("result") == "reached" and r.get("time_s")])
        sp = ci([r.get("spills") or 0 for r in rs])
        en = ci([r.get("spill_expected") or 0.0 for r in rs])
        rk = ci([r.get("spill_risk_pct") or 0.0 for r in rs])
        print("%-46s %2d  %d/%d%s  %6.1f±%-5.1f  %5.2f±%-5.2f  %5.2f±%-5.2f  %5.1f±%-4.1f" %
              (LABEL.get(a, a), len(rs), reached, len(rs),
               (" (%dH)" % aborts if aborts else "    "), t[0], t[1],
               sp[0], sp[1], en[0], en[1], rk[0], rk[1]))
    # curva de recuperacion de los brazos *dst* (secuencia por run)
    for a in ("wrongdst", "wrongdstsim"):
        rs = arms.get(a)
        if not rs:
            continue
        print("\nSECUENCIA %s (curva de recuperacion, orden temporal):" % a)
        for i, r in enumerate(rs):
            print("  run %d: t=%6.1fs derrames=%d E[N]=%.2f riesgo=%.1f%% %s" %
                  (i + 1, r.get("time_s") or -1, r.get("spills") or 0,
                   r.get("spill_expected") or 0, r.get("spill_risk_pct") or 0,
                   r.get("result")))
        st_file = {"wrongdst": "meta2_state_wrongdst.json",
                   "wrongdstsim": "meta2_state_wrongdstsim.json"}[a]
        p = os.path.join(HERE, st_file)
        if os.path.exists(p):
            st = json.load(open(p))
            for task, an in st.items():
                pls = {k: round(v["m_match"] + v["m_theta"], 2) for k, v in an.items()}
                print("  estado Layer2 final [%s]: Pl=%s  (runs/spills: %s)" %
                      (task, pls, {k: (v.get("runs"), v.get("spills")) for k, v in an.items()}))


if __name__ == "__main__":
    main()
