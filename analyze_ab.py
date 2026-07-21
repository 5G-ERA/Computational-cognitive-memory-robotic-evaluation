#!/usr/bin/env python3
"""Analisis del A/B formal pre-merge (rama analogy-profiles-door).

Une las lineas RUN del campaign_ab.log (arm/result/time) con los ficheros de
dataset/ (posicion final, eventos) y los estados L2 por brazo. Veredicto del
watch-item: tasa de 'abortos de bolsillo' (no-reached con dist a B < 1.5 m)
en brazos con robot_r gobernado (abprof/abfull) vs control (abctrl/abdoor).
"""
import json, math, os, re, sys

REPO = os.path.dirname(os.path.abspath(__file__))
BX, BY = -4.73, 3.04          # waypoint B
DOOR = (-3.7, 1.3)            # marco de la puerta (colisiones del run 6 real)
POCKET_R = 1.5                # umbral 'cerca de B'

runs = []
for line in open(os.path.join(REPO, "campaign_ab.log")):
    m = re.search(r"RUN (\{.*\})", line)
    if m:
        runs.append(eval(m.group(1)))  # dict literal impreso por sim_campaign

arms = {}
for r in runs:
    arms.setdefault(r["arm"], []).append(r)

def ci95(xs):
    n = len(xs)
    if n < 2:
        return 0.0
    mu = sum(xs) / n
    sd = (sum((x - mu) ** 2 for x in xs) / (n - 1)) ** 0.5
    return 1.96 * sd / n ** 0.5

print("=" * 100)
print("A/B FORMAL PRE-MERGE — %d runs" % len(runs))
print("=" * 100)
hdr = "%-8s %2s %8s %14s %7s %7s %6s | %s"
print(hdr % ("brazo", "n", "reached", "t_reached(s)", "spills", "cols", "pocket", "abortos (detalle)"))
verdict = {}
for arm in ("abctrl", "abprof", "abdoor", "abfull"):
    rs = arms.get(arm, [])
    if not rs:
        continue
    reached = [r for r in rs if r["result"] == "reached"]
    ts = [r["time_s"] for r in reached]
    pocket = 0
    details = []
    for r in rs:
        if r["result"] == "reached":
            continue
        f = os.path.join(REPO, "dataset", r["file"])
        note = r["result"]
        try:
            d = json.load(open(f))
            last = d.get("samples", [])[-1]
            dB = math.hypot(last["x"] - BX, last["y"] - BY)
            dD = math.hypot(last["x"] - DOOR[0], last["y"] - DOOR[1])
            cols = [(round(e.get("t", 0)), round(math.hypot(e.get("x", 9) - BX, e.get("y", 9) - BY), 1))
                    for e in d.get("events", []) if e.get("kind") == "collision"]
            zone = "POCKET" if dB < POCKET_R else ("door" if dD < 1.2 else "otro")
            if dB < POCKET_R:
                pocket += 1
            note += " dB=%.2f zona=%s cols(t,dB)=%s" % (dB, zone, cols)
        except Exception as e:
            note += " (sin fichero: %r)" % e
        details.append("%s: %s" % (r["file"][9:15], note))
    verdict[arm] = (len(rs), len(reached), pocket)
    print(hdr % (arm, len(rs), "%d/%d" % (len(reached), len(rs)),
                 "%.1f±%.1f" % (sum(ts) / len(ts), ci95(ts)) if ts else "—",
                 "%.2f" % (sum(r["spills"] for r in rs) / len(rs)),
                 sum(r["collisions"] for r in rs),
                 pocket, "; ".join(details) or "—"))

print()
print("--- estados L2 por brazo (trust final tras 8 runs) ---")
for arm in ("abctrl", "abprof", "abdoor", "abfull"):
    f = os.path.join(REPO, "meta2_state_%s.json" % arm)
    if os.path.exists(f):
        st = json.load(open(f))
        for k, sec in st.items():
            pls = {a: round(v.get("m_match", 0) + v.get("m_theta", 0), 2)
                   for a, v in sec.items() if isinstance(v, dict)}
            extra = {a: {kk: v[kk] for kk in ("crossings", "fails", "runs", "spills") if kk in v}
                     for a, v in sec.items() if isinstance(v, dict)}
            print("%-8s %-28s Pl=%s  %s" % (arm, k, pls, extra))

print()
print("--- VEREDICTO WATCH-ITEM (abortos de bolsillo, robot_r gobernado vs fijo) ---")
gov = sum(verdict.get(a, (0, 0, 0))[2] for a in ("abprof", "abfull"))
gov_n = sum(verdict.get(a, (0, 0, 0))[0] for a in ("abprof", "abfull"))
ctl = sum(verdict.get(a, (0, 0, 0))[2] for a in ("abctrl", "abdoor"))
ctl_n = sum(verdict.get(a, (0, 0, 0))[0] for a in ("abctrl", "abdoor"))
# pool con priors de la rama: extsmoke 1/6 gobernado (run 124415), prereal 0/4 control
print("gobernado (abprof+abfull): %d/%d  [+extsmoke 1/6 => %d/%d]" % (gov, gov_n, gov + 1, gov_n + 6))
print("control   (abctrl+abdoor): %d/%d  [+prereal  0/4 => %d/%d]" % (ctl, ctl_n, ctl, ctl_n + 4))
