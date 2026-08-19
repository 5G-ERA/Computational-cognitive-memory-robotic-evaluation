#!/usr/bin/env python3
"""Analiza un dataset run de la rama: ocupacion de estados meta, transiciones,
retiradas, asistencias, laser_trust e iface_q. Uso: analyze_msm.py <dataset.json>"""
import json, sys, collections

d = json.load(open(sys.argv[1]))
ss = d.get("samples", [])
ev = d.get("events", [])
print("run:", sys.argv[1].split("/")[-1])
gx, gy = d.get("goal", [0, 0])[:2] if isinstance(d.get("goal"), list) else (d.get("goal", {}).get("x", 0), d.get("goal", {}).get("y", 0))
import math
dfin = math.hypot(ss[-1]["x"] - gx, ss[-1]["y"] - gy) if ss else 99
print("d_final al goal: %.2fm (%s) | t_total: %.1fs | muestras: %d" % (dfin, "REACHED" if dfin < 0.45 else "NO llegado", ss[-1]["t"] if ss else 0, len(ss)))

occ = collections.Counter(s.get("meta_state") for s in ss)
tot = max(1, len(ss))
print("\nOCUPACION DE ESTADOS META:")
for k, v in occ.most_common():
    print("  %-10s %4d muestras (%.0f%%)" % (k, v, 100.0 * v / tot))

iq = [s["iface_q"] for s in ss if s.get("iface_q") is not None]
lt = [s["laser_trust"] for s in ss if s.get("laser_trust") is not None]
dc = [s["door_contra"] for s in ss if s.get("door_contra") is not None]
if iq: print("\niface_q: min %.2f / mediana %.2f" % (min(iq), sorted(iq)[len(iq)//2]))
if lt: print("laser_trust: min %.2f / final %.2f" % (min(lt), lt[-1]))
if dc: print("door_contra max en ventana 10s:", max(dc))

print("\nEVENTOS meta (transiciones, retiradas, humano):")
for e in ev:
    if e.get("kind") in ("meta_state", "retreat_start", "retreat_end", "human_assist",
                          "human_collision", "assist_recall", "laser_lied", "help", "col"):
        x = {k: v for k, v in e.items() if k not in ("kind", "t", "x", "y")}
        print("  t=%6.1fs %-15s (%.2f,%.2f) %s" % (e.get("t", -1), e.get("kind"),
              e.get("x", 0), e.get("y", 0), json.dumps(x, ensure_ascii=False) if x else ""))
ncol = sum(1 for e in ev if e.get("kind") == "col")
print("\ncolisiones (eventos col):", ncol)
