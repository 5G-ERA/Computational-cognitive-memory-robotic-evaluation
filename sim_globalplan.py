#!/usr/bin/env python3
"""
sim_globalplan.py — offline demo: global A* on LIVE laser vs on the STATIC map.

Reconstructs a real moment from logged data (a postmortem cloud = live map with its real
noise, plus the loaded reference map) and plans start->goal both ways. The Livox Mid-360
scans non-repetitively, so consecutive "live maps" differ frame to frame; we emulate that
by planning over several random 70% subsamples of the same cloud and measuring how much
the path wobbles. The static-map plan is invariant by construction; the live plan is not —
that wobble is what the robot was chasing (green zigzag line in the viewer, trembling door
axis, DOOR-AL thrash).

Usage:
  python sim_globalplan.py [cloud_col.json] [nav_map.json] [out.png]
"""
import heapq
import json
import math
import random
import sys

OCELL = 0.2
INFLATE = 1          # cells of safety dilation around obstacles (approximates build_costmap)


def load_refmap(path):
    m = json.load(open(path))
    return {tuple(c) for c in m["cells"]}, m.get("hband", [-0.9, 0.6])


def load_cloud_cells(path):
    c = json.load(open(path))
    lo, hi = c.get("hband", [-0.5, 0.6])
    raw = c["points"]
    if raw and isinstance(raw[0], (int, float)):               # lista PLANA [x,y,z,x,y,z,...]
        pts = [raw[i:i + 3] for i in range(0, len(raw) - 2, 3)]
    else:
        pts = raw
    cells = {(round(p[0] / OCELL), round(p[1] / OCELL))
             for p in pts if lo <= p[2] <= hi}
    return cells, c["pose"], pts


def inflate(cells, r=INFLATE):
    out = set()
    for (x, y) in cells:
        for i in range(-r, r + 1):
            for j in range(-r, r + 1):
                out.add((x + i, y + j))
    return out


def astar(start, goal, blocked, protect=2):
    """8-connected A*; cells within `protect` of start/goal are never blocked (standard unstick)."""
    blk = {c for c in blocked
           if max(abs(c[0] - start[0]), abs(c[1] - start[1])) > protect
           and max(abs(c[0] - goal[0]), abs(c[1] - goal[1])) > protect}
    openq = [(0, start)]
    gsc = {start: 0.0}
    came = {}
    while openq:
        _, cur = heapq.heappop(openq)
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return path[::-1]
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == dy == 0:
                    continue
                nb = (cur[0] + dx, cur[1] + dy)
                if nb in blk:
                    continue
                ng = gsc[cur] + math.hypot(dx, dy)
                if ng < gsc.get(nb, 1e18):
                    gsc[nb] = ng
                    came[nb] = cur
                    heapq.heappush(openq, (ng + math.hypot(goal[0] - nb[0], goal[1] - nb[1]), nb))
    return None


def path_dev(paths):
    """Mean point-to-nearest-point deviation (m) between every pair of paths."""
    devs = []
    for i in range(len(paths)):
        for j in range(i + 1, len(paths)):
            a, b = paths[i], paths[j]
            if not a or not b:
                continue
            d = sum(min(math.hypot(p[0] - q[0], p[1] - q[1]) for q in b) for p in a) / len(a)
            devs.append(d * OCELL)
    return sum(devs) / len(devs) if devs else float("nan")


def main():
    cloud_f = sys.argv[1] if len(sys.argv) > 1 else "dataset/20260702_152330_ours_B_col1.json"
    map_f = sys.argv[2] if len(sys.argv) > 2 else "nav_map.json"
    out_f = sys.argv[3] if len(sys.argv) > 3 else "sim_globalplan.png"
    refmap, _ = load_refmap(map_f)
    live, pose, pts = load_cloud_cells(cloud_f)
    start = (round(pose[0] / OCELL), round(pose[1] / OCELL))
    # goal REAL de esa run (el json principal hermano del _colN)
    import re
    main_f = re.sub(r"_col\d+\.json$", ".json", cloud_f)
    gj = json.load(open(main_f)).get("goal", {})
    goal = (round(gj.get("x", -4.73) / OCELL), round(gj.get("y", 3.04) / OCELL))

    rng = random.Random(7)
    live_paths, hard_paths = [], []
    for k in range(6):                    # 6 'frames' del Livox: el barrido no repetitivo cubre ~65%
        sub = {c for c in live if rng.random() < 0.65}          # de las celdas por frame (flicker medido)
        live_paths.append(astar(start, goal, inflate(sub)))
        hard_paths.append(astar(start, goal, inflate(refmap)))  # static: same input -> same path
    dev_live, dev_hard = path_dev(live_paths), path_dev(hard_paths)
    print(f"live-map plan   : mean path deviation between frames = {dev_live:.3f} m")
    print(f"static-map plan : mean path deviation between frames = {dev_hard:.3f} m")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 2, figsize=(13, 6), sharex=True, sharey=True)
    for ax, paths, title, col in ((axs[0], live_paths, f"GLOBAL on LIVE laser (old)\n6 frames -> 6 paths, dev {dev_live:.2f} m", "#2ca02c"),
                                  (axs[1], hard_paths, f"GLOBAL on STATIC map (new, G1_GLOBALMAP=hard)\n6 frames -> 1 path, dev {dev_hard:.2f} m", "#1f77b4")):
        ax.plot([c[0] * OCELL for c in refmap], [c[1] * OCELL for c in refmap], ".", ms=2, color="#999999", label="loaded map")
        ax.plot([c[0] * OCELL for c in live], [c[1] * OCELL for c in live], "s", ms=2.2, color="#d62728", alpha=0.45, label="live laser cells")
        for p in paths:
            if p:
                ax.plot([c[0] * OCELL for c in p], [c[1] * OCELL for c in p], "-", lw=1.6, color=col, alpha=0.7)
        ax.plot(pose[0], pose[1], "b^", ms=10, label="robot")
        ax.plot(goal[0] * OCELL, goal[1] * OCELL, "*", ms=16, color="orange", label="goal B")
        ax.set_title(title, fontsize=10)
        ax.set_aspect("equal")
        ax.grid(alpha=0.25)
        ax.legend(fontsize=7, loc="lower right")
    import os as _os
    fig.suptitle(f"Same moment ({_os.path.basename(cloud_f)}), same goal — plan stability", fontsize=11)
    fig.tight_layout()
    fig.savefig(out_f, dpi=110)
    print(f"-> {out_f}")


if __name__ == "__main__":
    main()
