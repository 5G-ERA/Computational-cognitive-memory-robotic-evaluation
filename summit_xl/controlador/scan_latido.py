#!/usr/bin/env python3
"""Latido del láser y de AMCL a fichero. 17-sep-2026, 18:55.

Un nodo PERSISTENTE (descubre a los publicadores una vez) que escribe cada 0,5 s en /tmp/latido.json la época del
último scan, del último map→odom y del último odom→base. Sustituye a las sondas `ros2 topic echo` de 4–6 s con un nodo
nuevo cada vez, que hoy dieron 0 con el scan fluyendo (el vigilante v1/v2 reinició el conversor por eso).
Lectura: python3 ~/scan_latido.py edad  → "scan 0.3 · map_odom 0.2 · odom 0.1" (segundos desde el último mensaje).
"""
import json, os, sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
F = "/tmp/latido.json"

if len(sys.argv) > 1 and sys.argv[1] == "edad":
    try:
        d = json.load(open(F)); t = time.time()
        print(" · ".join("%s %.1f" % (k, t - d[k]) for k in ("scan", "map_odom", "odom")), "· latido %.1f" % (t - d["escrito"]))
    except Exception as e:
        print("sin latido (%s)" % e)
    sys.exit(0)

class Latido(Node):
    def __init__(self):
        super().__init__("latido_laser")
        self.t = {"scan": 0.0, "map_odom": 0.0, "odom": 0.0}
        be = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, "/robot/top_laser/scan", lambda m: self.marca("scan"), be)
        self.create_subscription(TFMessage, "/tf", self.tf, 50)
        self.create_timer(0.5, self.escribe)
    def marca(self, k): self.t[k] = time.time()
    def tf(self, m):
        for tr in m.transforms:
            if tr.child_frame_id == "robot_odom": self.marca("map_odom")
            elif tr.header.frame_id == "robot_odom": self.marca("odom")
    def escribe(self):
        d = dict(self.t); d["escrito"] = time.time()
        tmp = F + ".tmp"; json.dump(d, open(tmp, "w")); os.replace(tmp, F)

rclpy.init(); rclpy.spin(Latido())
