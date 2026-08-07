#!/usr/bin/env python3
"""
g1_replay.py — counterfactual simulator over logged runs (no robot needed).

Replays safety/decision rules tick-by-tick against every run in dataset/ and reports,
for each rule: when it WOULD have fired vs. what actually happened (collisions, stucks).
A fix is accepted when it (a) fires BEFORE the real failures it targets, and (b) never
fires in runs that reached the goal cleanly ("do no harm" check).

Limitation (honest): this is open-loop replay — once a rule changes motion, the real
trajectory would diverge. It validates trigger correctness and false-positive rate,
which is what we can measure without a physics sim.

Rules replayed:
  ESCAPE       start-of-run boxed-in escape (c0<0.45 or carpet<0.50, first 5 s, not moved)
  HARD-GUARD   walls are non-negotiable: c0_hard<0.22 stop / <0.45 slow, only while advancing
  PRESS-GUARD  commanded forward but body barely moves AND vision says "on top" -> back off
               (the 152030/152330/152532 signature: 3-4 s scraping before the IMU noticed)
  SCRAPE       vision-independent fallback: sustained cmd-vs-motion mismatch -> early contact

Usage:
  python g1_replay.py                # all runs in dataset/
  python g1_replay.py dataset/20260702_152330_ours_B.json   # one run, verbose
"""
import glob
import json
import math
import os
import sys

# --- thresholds (mirror g1_goto.py; tune here first, then port) ---
ESC_TRIG, ESC_CARPET, ESC_WIN, ESC_MOVED = 0.45, 0.50, 5.0, 0.30
HG_STOP, HG_SLOW = 0.22, 0.45
PG_CMD, PG_SPD, PG_CNEAR, PG_SEC = 0.20, 0.10, 12, 2.0      # press-guard
SC_CMD, SC_SPD, SC_SEC = 0.25, 0.10, 3.5                     # scrape (no-vision fallback)


def fnum(v):
    return v if isinstance(v, (int, float)) else None


def replay_run(path, verbose=False):
    d = json.load(open(path))
    s = d.get("samples", [])
    ev = d.get("events", [])
    if not s:
        return None
    cols = [e for e in ev if e.get("kind") == "collision"]
    x0, y0 = s[0]["x"], s[0]["y"]
    out = {"file": os.path.basename(path), "result": d.get("result", "?"),
           "time_s": round(s[-1]["t"], 1), "collisions": len(cols),
           "col_ts": [round(e["t"], 1) for e in cols]}

    # --- ESCAPE ---
    esc = None
    for r in s:
        if r["t"] >= ESC_WIN:
            break
        c0, cp, dg = r.get("c0"), fnum(r.get("carpet_pct")), r.get("d")
        if c0 is None or dg is None or dg <= 1.5:
            continue
        if math.hypot(r["x"] - x0, r["y"] - y0) >= ESC_MOVED:
            continue
        if c0 < ESC_TRIG or (cp is not None and cp < ESC_CARPET):
            esc = (round(r["t"], 1), "laser" if c0 < ESC_TRIG else "vision", c0, cp)
            break
    out["escape"] = esc

    # --- HARD-GUARD (needs c0_hard column; older runs lack it) ---
    hg_stop = [r["t"] for r in s
               if fnum(r.get("c0_hard")) is not None and r["c0_hard"] < HG_STOP
               and (r.get("cmd") or [0, 0])[1] > 0.05]
    hg_slow = [r["t"] for r in s
               if fnum(r.get("c0_hard")) is not None and HG_STOP <= r["c0_hard"] < HG_SLOW
               and (r.get("cmd") or [0, 0])[1] > 0.05]
    out["hg_stop_n"], out["hg_slow_n"] = len(hg_stop), len(hg_slow)
    out["hg_stop_ts"] = [round(t, 1) for t in hg_stop[:6]]

    # --- PRESS-GUARD / SCRAPE: sustained windows ---
    def sustained(cond, need_sec):
        """Return fire times: each instant when `cond` has held for >= need_sec."""
        fires, t_on = [], None
        for r in s:
            if cond(r):
                if t_on is None:
                    t_on = r["t"]
                if r["t"] - t_on >= need_sec:
                    fires.append(round(r["t"], 1))
                    t_on = None                      # re-arm (one fire per window)
            else:
                t_on = None
        return fires

    press = sustained(lambda r: ((r.get("cmd") or [0, 0])[1] >= PG_CMD
                                 and (fnum(r.get("spd")) or 0) <= PG_SPD
                                 and (fnum(r.get("color_near")) or 0) >= PG_CNEAR), PG_SEC)
    scrape = sustained(lambda r: ((r.get("cmd") or [0, 0])[1] >= SC_CMD
                                  and (fnum(r.get("spd")) or 0) <= SC_SPD), SC_SEC)
    out["press"], out["scrape"] = press, scrape

    # lead time of first press/scrape fire before each real collision (negative = after)
    leads = []
    for e in cols:
        prior = [t for t in press + scrape if t <= e["t"] + 0.2]
        leads.append(round(e["t"] - max(prior), 1) if prior else None)
    out["lead_s"] = leads

    if verbose:
        print(json.dumps(out, indent=2))
    return out


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    if args:
        replay_run(args[0], verbose=True)
        return
    rows = [r for r in (replay_run(p) for p in sorted(glob.glob("dataset/*_ours_*.json"))
                        if not any(t in p for t in ("_col", "_end", "_noise"))) if r]
    print(f"{'run':30s} {'result':8s} {'col@':14s} {'ESCAPE':22s} {'HG stop/slow':12s} "
          f"{'PRESS fires':16s} {'SCRAPE fires':16s} {'lead(s)':10s}")
    for r in rows:
        e = r["escape"]
        etxt = f"t={e[0]} {e[1]}" if e else "-"
        print(f"{r['file'][:30]:30s} {r['result']:8s} {str(r['col_ts'])[:14]:14s} {etxt:22s} "
              f"{r['hg_stop_n']}/{r['hg_slow_n']:<10d} {str(r['press'])[:16]:16s} "
              f"{str(r['scrape'])[:16]:16s} {str(r['lead_s'])[:10]:10s}")
    # --- verdict summary ---
    clean = [r for r in rows if r["result"] == "reached" and r["collisions"] == 0]
    dirty = [r for r in rows if r["collisions"] > 0]
    fp = [r["file"] for r in clean if r["press"] or r["scrape"]]
    covered = sum(1 for r in dirty for l in r["lead_s"] if l is not None and l >= 0)
    total = sum(r["collisions"] for r in dirty)
    print(f"\ncollisions with a PRESS/SCRAPE fire at or before impact: {covered}/{total}")
    print(f"false fires in clean reached runs: {len(fp)} {fp}")


if __name__ == "__main__":
    main()
