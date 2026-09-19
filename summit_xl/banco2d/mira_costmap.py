#!/usr/bin/env python3
"""Mira el costmap global vivo: coste bajo el robot, corredor libre en el vano y si hay camino de celdas no letales.
17-sep-2026. Responde por qué NavFn dice «failed to create plan» sin tener que adivinar."""
import math, sys, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

# El topic `costmap` publica un OccupancyGrid reescalado 0–100 (no el 0–254 interno): letal 100, inscrito 99,
# desconocido -1. Medir con 254/253 da «0 letales» y engaña.
LETAL, INSCRITO = 100, 99


class Mira(Node):
    def __init__(self):
        super().__init__("mira_costmap")
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.cm = None; self.pose = None
        self.create_subscription(OccupancyGrid, "/robot/global_costmap/costmap", self.cb, q)
        self.create_subscription(PoseStamped, "/sim/pose_verdad", self.cb_p, 10)

    def cb(self, m): self.cm = m
    def cb_p(self, m): self.pose = (m.pose.position.x, m.pose.position.y)


def main():
    rclpy.init(); n = Mira()
    for _ in range(200):
        rclpy.spin_once(n, timeout_sec=0.05)
        if n.cm is not None and n.pose is not None:
            break
    if n.cm is None:
        print("sin costmap"); return
    g = n.cm; res = g.info.resolution; ox, oy = g.info.origin.position.x, g.info.origin.position.y
    d = np.array(g.data, dtype=np.int16).reshape(g.info.height, g.info.width)
    d[d < 0] = 127   # desconocido
    def cel(x, y): return int((y - oy) / res), int((x - ox) / res)
    print("costmap %dx%d res %.3f origen (%.2f, %.2f) · letales %d · inscritas %d · desconocidas %d"
          % (g.info.width, g.info.height, res, ox, oy, (d == LETAL).sum(), (d == INSCRITO).sum(), (d == 127).sum()))
    px, py = n.pose; i, j = cel(px, py)
    print("robot en (%.2f, %.2f) → celda coste %d" % (px, py, d[i, j]))
    for nom, (x, y) in (("vano_antes", (1.830, -3.418)), ("medio vano", (1.738, -4.212)), ("vano_despues", (1.646, -5.006)), ("B", (0.362, -6.076))):
        i, j = cel(x, y)
        print("  %-13s (%.2f, %.2f) coste %d" % (nom, x, y, d[i, j] if 0 <= i < g.info.height and 0 <= j < g.info.width else -1))
    # corredor libre (coste < 253) perpendicular al eje del vano
    ax, ay, bx, by = 1.830, -3.418, 1.646, -5.006
    th = math.atan2(by - ay, bx - ax); vx, vy = -math.sin(th), math.cos(th)
    mx, my = (ax + bx) / 2, (ay + by) / 2
    peor = None
    for a in np.arange(-0.8, 0.8, 0.02):
        cx, cy = mx + a * math.cos(th), my + a * math.sin(th)
        der = izq = 0.0
        for b in np.arange(0, 1.2, res):
            i, j = cel(cx + b * vx, cy + b * vy)
            if not (0 <= i < g.info.height and 0 <= j < g.info.width) or d[i, j] >= INSCRITO: break
            der = b
        for b in np.arange(0, 1.2, res):
            i, j = cel(cx - b * vx, cy - b * vy)
            if not (0 <= i < g.info.height and 0 <= j < g.info.width) or d[i, j] >= INSCRITO: break
            izq = b
        if peor is None or der + izq < peor[0]: peor = (der + izq, a)
    print("corredor con coste < %d en el vano: %.3f m (a %+.2f m del centro)" % (INSCRITO, peor[0], peor[1]))
    print("coste máximo en el costmap: %d" % d.max())
    # ¿hay camino de celdas no letales del robot a B? (inundación 8-vecinos sobre coste < 254)
    libre = d < INSCRITO
    i0, j0 = cel(px, py); i1, j1 = cel(0.362, -6.076)
    vis = np.zeros_like(libre); pila = [(i0, j0)]; vis[i0, j0] = True; ok = False
    while pila:
        i, j = pila.pop()
        if (i, j) == (i1, j1): ok = True; break
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                a, b = i + di, j + dj
                if 0 <= a < libre.shape[0] and 0 <= b < libre.shape[1] and not vis[a, b] and libre[a, b]:
                    vis[a, b] = True; pila.append((a, b))
    print("¿camino de celdas por debajo de la banda inscrita robot→B? %s" % ("SÍ" if ok else "NO"))
    rclpy.shutdown()


main()
