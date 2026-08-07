#!/usr/bin/env python3
"""REPLAY EN SOMBRA de la maquina de estados META (rama tutor-feedback-metareasoner)
sobre datasets reales capturados. La linea temporal registrada es el contrafactual
"sin actuacion": reconstruimos tick a tick laser_trust, BLIND, DEGRADED, los puntos
donde RETREAT habria disparado y las ventanas donde ASSIST habria pedido ayuda.

Paridad con el codigo de la rama (g1_goto.py lote 2), con UNA conversion deliberada:
la recuperacion de trust (+0.001/tick a 10 Hz) pasa a +0.01/s para ser independiente
de la tasa de muestreo (los runs reales van a ~2 Hz).

Aproximacion documentada: tras un disparo de RETREAT en sombra no hay reversa real,
asi que aplicamos cooldown de 20 s (14 s de ejecucion tipica + 6 s de cool) para
aproximar la cadencia de disparos. RECOVERY no ocupa muestras en sombra.
"""
import json, sys, math, os

def replay(path):
    d = json.load(open(path))
    ss = d.get("samples", [])
    ev = d.get("events", [])
    if len(ss) < 10:
        return None
    cols = sorted(e["t"] for e in ev if e.get("kind") in ("collision", "human_collision"))
    assists = sorted(e["t"] for e in ev if e.get("kind") == "human_assist")
    goal = d.get("goal") or {}
    gx = goal.get("x") if isinstance(goal, dict) else (goal[0] if goal else None)
    gy = goal.get("y") if isinstance(goal, dict) else (goal[1] if len(goal) > 1 else None)

    trust = 1.0; trust_min = 1.0; lied = []
    crumbs = []; span_hist = []
    rt_trigs = []; rt_cool_until = -1.0; rt_count = 0
    assist_t0 = None; assist_secs = 0.0
    occ = {"NORMAL": 0.0, "BLIND": 0.0, "DEGRADED": 0.0, "ASSIST": 0.0}
    ncol_seen = 0
    c0_prev = 2.5
    prev_t = None
    perc_dead_secs = 0.0
    have_perc = any(s.get("perc_age") is not None for s in ss[:20])

    for i, s in enumerate(ss):
        t = s["t"]; x = s["x"]; y = s["y"]; c0 = s.get("c0", 2.5)
        dt = (t - prev_t) if prev_t is not None else 0.5
        if dt <= 0: dt = 0.5
        prev_t = t
        # --- colisiones nuevas hasta t ---
        new_cols = 0
        while ncol_seen < len(cols) and cols[ncol_seen] <= t:
            ncol_seen += 1; new_cols += 1
        # --- laser_trust (validez retrospectiva) ---
        if new_cols and c0_prev > 0.6:
            trust = max(0.2, trust - 0.25)
            lied.append((t, c0_prev))
        trust = min(1.0, trust + 0.01 * dt)
        trust_min = min(trust_min, trust)
        c0_prev = c0
        # --- migas / atasco ---
        if not crumbs or math.hypot(x - crumbs[-1][0], y - crumbs[-1][1]) >= 0.15:
            crumbs.append((x, y)); del crumbs[:-30]
        span = math.hypot(x - crumbs[0][0], y - crumbs[0][1]) if crumbs else 0.0
        span_hist.append((t, x, y))
        old = None
        for (t_, x_, y_) in reversed(span_hist):
            if t - t_ >= 3.8:
                old = (t_, x_, y_); break
        stuck = (old is not None and math.hypot(x - old[1], y - old[2]) < 0.08 and c0 < 0.42)
        ncol_20s = sum(1 for tc in cols[:ncol_seen] if t - tc <= 20.0)
        colhit = (ncol_20s >= 2) or (new_cols and stuck)
        # --- asistencia humana registrada -> presupuestos re-armados (paridad gemelo) ---
        while assists and assists[0] <= t:
            assists.pop(0); rt_count = 0; rt_cool_until = t; assist_t0 = assist_t0
        # --- disparo RETREAT (sombra) ---
        if (colhit or stuck) and t > 12.0 and span >= 0.5 and rt_count < 3 and t > rt_cool_until and len(crumbs) >= 3:
            rt_trigs.append({"t": round(t, 1), "x": round(x, 2), "y": round(y, 2),
                             "por": "col" if colhit else "atasco", "c0": round(c0, 2)})
            rt_count += 1
            rt_cool_until = t + 20.0
        # --- iface_q (solo componente percepcion en replay) ---
        iface = 1.0
        if have_perc:
            pa = s.get("perc_age")
            iface = 1.0 if (pa is not None and pa < 1.5) else 0.0
            if iface == 0.0: perc_dead_secs += dt
        # --- estado (sombra, sin RECOVERY) ---
        if rt_count >= 3 and (stuck or c0 < 0.45):
            st = "ASSIST"
            if assist_t0 is None: assist_t0 = t
            assist_secs += dt
        elif trust < 0.6 or iface < 0.5:
            st = "DEGRADED"
        elif c0 < 1.0 or t < rt_cool_until:
            st = "BLIND"
        else:
            st = "NORMAL"
        occ[st] += dt

    last = ss[-1]
    dur = last["t"]
    reached = None
    if gx is not None:
        reached = math.hypot(last["x"] - gx, last["y"] - gy) < 0.45
    tot = sum(occ.values()) or 1.0
    return {
        "file": os.path.basename(path), "dur_s": round(dur, 1), "reached": reached,
        "n_col": len(cols), "laser_lied": len(lied), "lied_pts": [round(t,1) for t,_ in lied[:6]],
        "trust_min": round(trust_min, 2),
        "occ_pct": {k: round(100.0 * v / tot) for k, v in occ.items()},
        "rt_trigs": rt_trigs, "assist_t0": (round(assist_t0,1) if assist_t0 else None),
        "assist_secs": round(assist_secs, 1),
        "perc_dead_secs": round(perc_dead_secs, 1) if have_perc else None,
    }

if __name__ == "__main__":
    for p in sys.argv[1:]:
        r = replay(p)
        if r: print(json.dumps(r, ensure_ascii=False))
