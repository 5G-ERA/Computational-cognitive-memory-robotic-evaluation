#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
spill_mark.py — Marcador MANUAL de derrames (ground truth de la condicion payload).

En las runs REALES con el G1: lanza esto en otra terminal del Ubuntu (o de cualquier maquina
de la LAN apuntando a la IP del Ubuntu) y pulsa ENTER cada vez que caiga agua de la taza.
Cada pulsacion manda un datagrama UDP al listener de g1_goto (G1_SPILL_GT_PORT, 7777 por
defecto), que registra el evento kind='spill_human' en el dataset con el timestamp de la run.

En SIM tambien vale, o alternativamente:
    ros2 topic pub --once /spill_event std_msgs/msg/Empty
(el adaptador reenvia /spill_event al mismo puerto UDP).

USO:  python spill_mark.py [host] [puerto]        (default 127.0.0.1 7777)
"""
import socket
import sys
import time

host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
port = int(sys.argv[2]) if len(sys.argv) > 2 else 7777
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
n = 0
print(f"Marcador de derrames -> {host}:{port}")
print("ENTER = 1 derrame · Ctrl+C para salir")
try:
    while True:
        input()
        s.sendto(b"spill", (host, port))
        n += 1
        print(f"  derrame #{n} marcado ({time.strftime('%H:%M:%S')})")
except KeyboardInterrupt:
    print(f"\n{n} derrames marcados en total.")
