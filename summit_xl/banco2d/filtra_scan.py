#!/usr/bin/env python3
"""Quita del LaserScan los retornos atípicos aislados antes de que lleguen al costmap. 18-sep-2026.

Por qué: medido en el banco, el recto por la puerta (DriveOnHeading) sale 4/4 sin ruido de láser y 0/4 con σ = 2 cm,
con el robot perfectamente alineado y 50 mm de holgura real. El costmap local sólo tiene la capa de obstáculos a
2 cm: **un** retorno que caiga 3 cm dentro del vano marca una celda letal, la huella con padding la toca y el
behavior server aborta. Y el láser real (bolsa del 17-sep, robot parado) tiene σ típica 4 mm pero cola pesada:
p95 22 mm, p99 112 mm — píxeles mixtos, que salen justo en los bordes, o sea en las jambas.

Regla (conserva los bordes): un rayo se descarta (→ inf, ni marca ni limpia) sólo si discrepa más de TAU de la mediana
de sus 3 vecinos de la izquierda **y** de la de sus 3 de la derecha. Un borde de verdad coincide con uno de los dos
lados; un atípico aislado, con ninguno. Sin latencia: es espacial, no temporal.

  python3 filtra_scan.py [--entrada /robot/top_laser/scan_crudo] [--salida /robot/top_laser/scan] [--tau 0.03]
"""
import argparse, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class Filtro(Node):
    def __init__(self, a):
        super().__init__("filtra_scan")
        be = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.tau = a.tau; self.n = 0; self.quitados = 0
        self.pub = self.create_publisher(LaserScan, a.salida, be)
        self.create_subscription(LaserScan, a.entrada, self.cb, be)
        self.create_timer(10.0, self.informa)

    def cb(self, m):
        r = np.array(m.ranges, dtype=np.float64); r[~np.isfinite(r)] = np.nan
        n = len(r); idx = np.arange(n)
        def lado(d):   # mediana de 3 vecinos a un lado, con el scan tratado como circular (cubre ±180°)
            v = np.stack([r[(idx + k * d) % n] for k in (1, 2, 3)])
            with np.errstate(all="ignore"):
                return np.nanmedian(v, axis=0)
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            izq, der = lado(-1), lado(1)
        # El umbral crece con la distancia: en una pared lejana vista de refilón dos rayos vecinos difieren
        # legítimamente r·Δθ·tan(incidencia) —9 cm a 3 m y 60°—; con 3 cm fijos el filtro tiraba 174 rayos buenos por
        # scan. Cerca (las jambas, < 1 m) sigue valiendo TAU.
        tau = np.maximum(self.tau, 0.03 * np.nan_to_num(r, nan=0.0))
        malo = np.isfinite(r) & (np.abs(r - izq) > tau) & (np.abs(r - der) > tau)
        malo &= np.isfinite(izq) | np.isfinite(der)
        r[malo] = np.nan
        self.n += 1; self.quitados += int(malo.sum())
        m.ranges = [float(v) if np.isfinite(v) else float("inf") for v in r]
        self.pub.publish(m)

    def informa(self):
        if self.n:
            self.get_logger().info("scans %d · atípicos quitados por scan: %.1f" % (self.n, self.quitados / self.n))


ap = argparse.ArgumentParser()
ap.add_argument("--entrada", default="/robot/top_laser/scan_crudo"); ap.add_argument("--salida", default="/robot/top_laser/scan")
ap.add_argument("--tau", type=float, default=0.03)
rclpy.init(); rclpy.spin(Filtro(ap.parse_args()))
