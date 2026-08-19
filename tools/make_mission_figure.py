#!/usr/bin/env python3
"""Figura 'la misión': trayectorias REALES sobre el plano escaneado del piso.
Izquierda: la ventana golden (5 travesías limpias). Derecha: una travesía patológica
(bucle en el marco de la puerta) de la misma campaña. Datos reales, sin retoque."""
import json, glob, os, math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.lines import Line2D

HERE = "/Users/adrianlendinezibanez/Claude/Projects/G1 ROBOT"
SCR = os.path.dirname(os.path.abspath(__file__))
walls = json.load(open(os.path.join(SCR, "walls.json")))

INK = "#0f1c2b"; WALL = "#c3ccd6"; MUTED = "#7c8899"
GOOD = "#1a7a2e"; GOOD2 = "#2f9e4f"; BAD = "#8a2f2f"; ACC = "#b45309"

DOOR = (-3.90, 1.25)
A = (0.99, 0.57); B = (-4.73, 3.04)


def draw_map(ax):
    for (x, y, w, h) in walls:
        ax.add_patch(Rectangle((x - w / 2, y - h / 2), w, h, facecolor=WALL,
                               edgecolor="none", zorder=1))
    ax.add_patch(plt.Circle(DOOR, 0.42, fill=False, ec=ACC, lw=2.0, zorder=4, alpha=.9))
    for (px, py), lab in ((A, "A"), (B, "B")):
        ax.plot(px, py, "o", ms=9, mfc="#fff", mec=INK, mew=2.0, zorder=6)
        ax.text(px + 0.28, py + 0.22, lab, fontsize=13, weight="bold", color=INK, zorder=6)
    ax.set_xlim(-6.6, 3.2); ax.set_ylim(-1.6, 5.0)
    ax.set_aspect("equal"); ax.axis("off")


def traj(path):
    d = json.load(open(path))
    ss = d["samples"]
    return [s["x"] for s in ss], [s["y"] for s in ss], d, ss[-1]["t"]


fig, axes = plt.subplots(1, 2, figsize=(15.2, 6.4), dpi=200)
fig.patch.set_facecolor("white")

# ---------------- IZQUIERDA: la ventana golden ----------------
ax = axes[0]
draw_map(ax)
runs = sorted(g for g in glob.glob(os.path.join(HERE, "dataset/20260724_16*_ours_*.json"))
              if "_col" not in g and "_end" not in g)
ok = [r for r in runs if os.path.basename(r)[9:13] >= "1629"]
for i, r in enumerate(ok):
    xs, ys, d, t = traj(r)
    ax.plot(xs, ys, lw=2.2, color=GOOD if i % 2 == 0 else GOOD2, alpha=.85, solid_capstyle="round", zorder=5)
ax.set_title("The golden window — five clean crossings in a row",
             fontsize=13.5, weight="bold", color=INK, pad=12, loc="left")
ax.text(0.0, -0.04, "52–65 s per leg · 0 collisions · door bearing measured by vision",
        transform=ax.transAxes, fontsize=11, color=MUTED)

# ---------------- DERECHA: la travesía patológica ----------------
ax = axes[1]
draw_map(ax)
bad = os.path.join(HERE, "dataset/20260708_104520_ours_A.json")
xs, ys, d, t = traj(bad)
ax.plot(xs, ys, lw=1.7, color=BAD, alpha=.75, solid_capstyle="round", zorder=5)
cols = [(e["x"], e["y"]) for e in d.get("events", []) if e.get("kind") == "collision"]
if cols:
    cx = [c[0] for c in cols]; cy = [c[1] for c in cols]
    ax.plot(cx, cy, "o", ms=15, mfc="white", mec="white", zorder=7)     # halo
    ax.plot(cx, cy, "x", ms=12, mew=3.2, color="#5c1414", zorder=8)      # aspa encima
ax.set_title("What it looks like when the laser lies", fontsize=13.5, weight="bold",
             color=INK, pad=12, loc="left")
ax.text(0.0, -0.04, "%.0f s · %.1f m walked · %d collisions, all at the same door frame"
        % (t, sum(math.dist((xs[i], ys[i]), (xs[i+1], ys[i+1])) for i in range(len(xs)-1)), len(cols)),
        transform=ax.transAxes, fontsize=11, color=MUTED)

leg = [Line2D([], [], color=GOOD, lw=2.4, label="successful crossing"),
       Line2D([], [], color=BAD, lw=2.0, label="failed run"),
       Line2D([], [], color=BAD, marker="x", ls="", mew=2.4, ms=9, label="collision"),
       Line2D([], [], color=ACC, marker="o", ls="", mfc="none", mew=2, ms=10, label="the door (0.8 m)"),
       Line2D([], [], color=INK, marker="o", ls="", mfc="white", mew=2, ms=8, label="waypoints A / B")]
fig.legend(handles=leg, loc="lower center", ncol=5, frameon=False, fontsize=11,
           bbox_to_anchor=(0.5, -0.005))

fig.suptitle("Same robot, same flat, same door — real recorded trajectories",
             fontsize=17, weight="bold", color=INK, x=0.038, ha="left", y=0.985)
fig.text(0.038, 0.925, "A humanoid carrying an open cup of water between two waypoints. "
         "The laser cannot see below one metre, so the door frame is invisible to it.",
         fontsize=11.5, color=MUTED, ha="left")
fig.subplots_adjust(top=0.855, bottom=0.085, left=0.02, right=0.985, wspace=0.02)
out = os.path.join(SCR, "mission.png")
fig.savefig(out, facecolor="white")
print("guardado:", out)
PYEOF_MARKER = None
