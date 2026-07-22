#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
spill_mark.py v2 — Marcador MANUAL de derrames + latido + pesos + anotacion de invalidez.

Canal de VERDAD DE CAMPO de la condicion payload. Lanza esto en otra terminal y dejalo
corriendo TODA la sesion. El listener de g1_goto (G1_SPILL_GT_PORT, 7777) registra todo
en el dataset del run en curso.

COMANDOS (auditoria 2026-07-22):
  ENTER                 = 1 derrame (una rafaga de chapoteo = UNA marca)
  w <gramos>            = registrar peso del vaso (ej: 'w 200' antes de salir, 'w 153' al llegar)
  i [motivo]            = marcar la RUN EN CURSO como INVALIDA (ej: 'i marcador caido')
  h                     = ayuda
Ademas envia un LATIDO cada 2 s: el run sabe si este marcador esta vivo y lo escribe en el
dataset (gt_hb_seen / gt_alive_at_end / gt_dropouts) — se acabo el "¿cero derrames o
marcador muerto?".

USO:  python spill_mark.py [host] [puerto]        (default 127.0.0.1 7777)
"""
import socket
import sys
import threading
import time

host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
port = int(sys.argv[2]) if len(sys.argv) > 2 else 7777
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def _heartbeat():
    while True:
        try:
            s.sendto(b"hb", (host, port))
        except Exception:
            pass
        time.sleep(2.0)


threading.Thread(target=_heartbeat, daemon=True).start()
n = 0
print(f"Marcador de derrames v2 -> {host}:{port}  (latido cada 2s)")
print("ENTER=derrame · 'w <gramos>'=peso · 'i [motivo]'=run invalida · Ctrl+C=salir")
try:
    while True:
        line = input().strip()
        if not line:
            s.sendto(b"spill", (host, port))
            n += 1
            print(f"  derrame #{n} marcado ({time.strftime('%H:%M:%S')})")
        elif line.lower().startswith("w"):
            try:
                grams = float(line.split(None, 1)[1])
                s.sendto(f"weigh:{grams:g}".encode(), (host, port))
                print(f"  peso registrado: {grams:g} g")
            except (IndexError, ValueError):
                print("  uso: w <gramos>   (ej: w 200)")
        elif line.lower().startswith("i"):
            why = line.split(None, 1)[1] if len(line.split(None, 1)) > 1 else ""
            s.sendto(f"invalid:{why}".encode()[:120], (host, port))
            print(f"  RUN MARCADA INVALIDA ({why or 'sin motivo'})")
        elif line.lower() in ("h", "?", "help", "ayuda"):
            print("  ENTER=derrame · w <g>=peso · i [motivo]=invalida · Ctrl+C=salir")
        else:
            print("  (comando no reconocido; 'h' para ayuda — ENTER a secas marca derrame)")
except KeyboardInterrupt:
    print(f"\n{n} derrames marcados en total.")
