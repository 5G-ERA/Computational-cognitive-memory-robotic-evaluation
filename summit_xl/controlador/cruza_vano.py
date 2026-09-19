#!/usr/bin/env python3
"""Cruce de un vano guiado por la HOLGURA medida en el láser, no por el costmap ni por AMCL. 18-sep-2026.

Por qué existe. Medido en el banco 2D: el `DriveOnHeading` de Nav2 cruza 4/4 sin ruido de láser y 0/6 con un ruido
como el del láser real (σ 5 mm + 3 % de atípicos), con el robot perfectamente alineado y 50 mm reales por lado. Su
chequeo de colisión mira un costmap instantáneo a 2 cm: **un** retorno atípico dentro del vano es una celda letal.
Aquí la holgura de cada lado sale de un percentil sobre decenas de puntos, así que un atípico no la mueve.

Qué hace. Avanza a velocidad baja manteniéndose centrado entre lo que tiene a izquierda y derecha:
  · holgura_izq / holgura_der = percentil 15 de |y| de los puntos que flanquean el cuerpo, menos media anchura;
  · error lateral = (izq − der)/2 en una ventana por delante; error de rumbo = cómo cambia ese error con la distancia;
    cuando por delante ya no hay jambas (se sale a la sala), mantiene el rumbo de la odometría;
  · PARA si: el pasillo frontal tiene ≥ 5 retornos en 2 scans seguidos (obstáculo de verdad, no un atípico), el scan
    lleva > 1 s mudo, la holgura robusta de un lado baja de 15 mm, o se cumple la distancia.
Cada ciclo escribe una fila en un CSV: eso es el replay de holgura del cruce.

  python3 cruza_vano.py --metros 1.35 [--v 0.06] [--cmd /robot/cmd_vel] [--csv FICHERO]
      en el robot real: --cmd el topic de velocidad que acepte la base, y arrancar en la marca de antes del vano.
"""
import argparse, csv, math, sys, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32

MEDIO_ANCHO, MEDIO_LARGO, LASER_X = 0.3065, 0.361, -0.01924
# Amortiguamiento crítico de ÿ + K_TH·ẏ + v·K_Y·y = 0 a v = 0,06 m/s: K_Y = K_TH²/(4v) ≈ 9 con K_TH = 1,5.
W_MAX, K_Y, K_TH = 0.25, 9.0, 1.5
HOLGURA_MIN, PASILLO_N, SCAN_MUDO = 0.015, 5, 1.0
FRENTE = 0.25      # m de pasillo libre exigidos por delante del morro: a 6 cm/s y 3 Hz de scan son > 4 s de aviso


class Cruce(Node):
    def __init__(self, a):
        super().__init__("cruza_vano")
        self.a = a; self.pts = None; self.t_scan = 0.0; self.odom = None; self.odom0 = None
        self.bloqueos = 0; self.verdad_holg = float("nan"); self.choque = False; self.filas = []
        be = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, a.scan, self.cb_scan, be)
        self.create_subscription(Odometry, a.odom, self.cb_odom, 20)
        self.create_subscription(Float32, "/sim/holgura", lambda m: setattr(self, "verdad_holg", m.data), 10)   # sólo en el banco
        self.create_subscription(Bool, "/sim/choque", lambda m: setattr(self, "choque", self.choque or m.data), 10)
        self.cmd = self.create_publisher(Twist, a.cmd, 10)

    def cb_scan(self, m):
        r = np.array(m.ranges, dtype=np.float64); th = m.angle_min + m.angle_increment * np.arange(len(r))
        ok = np.isfinite(r) & (r > m.range_min) & (r < 6.0)
        self.pts = np.stack([r[ok] * np.cos(th[ok]) + LASER_X, r[ok] * np.sin(th[ok])], axis=1); self.t_scan = time.time()

    def cb_odom(self, m):
        q = m.pose.pose.orientation
        self.odom = (m.pose.pose.position.x, m.pose.pose.position.y, 2 * math.atan2(q.z, q.w))

    def lado(self, x0, x1, signo):
        """Distancia lateral al BORDE más cercano de un lado, en la franja x0..x1: el 3.er punto más próximo.

        No un percentil: la jamba aporta pocos puntos y la pared de detrás muchos, así que el percentil 15 caía detrás
        del borde — daba 43 mm de holgura con la huella ya tocando (medido en el banco: 7 choques de 12 con la holgura
        «medida» sana). El 3.º más cercano sigue siendo robusto a un atípico suelto, que es lo que hay (3 % de rayos)."""
        p = self.pts; y = signo * p[:, 1]
        m = (p[:, 0] > x0) & (p[:, 0] < x1) & (y > 0.15) & (y < 0.9)
        return float(np.sort(y[m])[2]) if m.sum() >= 6 else None

    def holgura(self, signo):
        """Holgura de un lado contra el RECTÁNGULO de la huella, no sólo de costado: con el robot guiñado lo que toca
        es una esquina, y |y| − media anchura no lo ve (medido: un choque de 12 con 42 mm «medidos» de costado).
        Distancia punto–rectángulo; 3.er punto más próximo, por los atípicos."""
        p = self.pts; m = (signo * p[:, 1] > 0.10) & (np.abs(p[:, 0]) < MEDIO_LARGO + 0.5) & (np.abs(p[:, 1]) < 1.0)
        if m.sum() < 6: return float("nan")
        q = p[m]; d = np.hypot(np.maximum(np.abs(q[:, 0]) - MEDIO_LARGO, 0.0), np.maximum(np.abs(q[:, 1]) - MEDIO_ANCHO, 0.0))
        return float(np.sort(d)[2])

    def detecta_vano(self):
        """Busca el vano por delante: rebanadas de 5 cm en x con algo a cada lado y anchura de puerta (0,62–1,0 m).
        Devuelve (x_vano, y_centro, ancho, rumbo_del_eje) en el marco del robot, o None.

        El rumbo del eje sale de la PARED que contiene el vano (perpendicular a ella), ajustada por mínimos cuadrados
        recortados sobre los puntos de los dos lados: son decenas de puntos, así que el ruido por rayo no la mueve."""
        p = self.pts; reb = []
        for x0 in np.arange(0.0, 1.6, 0.05):
            # rebanadas de 15 cm solapadas: con la puerta vista 4° girada las dos jambas distan 5 cm en x, y en una
            # rebanada de 5 cm sólo cabía una → «SIN_VANO» arrancando torcido
            m = (p[:, 0] >= x0 - 0.05) & (p[:, 0] < x0 + 0.10) & (np.abs(p[:, 1]) < 1.0)
            yi = p[m & (p[:, 1] > 0.05), 1]; yd = -p[m & (p[:, 1] < -0.05), 1]
            if len(yi) >= 2 and len(yd) >= 2:
                li, ld = (float(np.sort(yi)[1]) if len(yi) > 2 else float(yi.min())), (float(np.sort(yd)[1]) if len(yd) > 2 else float(yd.min()))   # el borde: 2.º más cercano
                if 0.62 <= li + ld <= 1.0:
                    reb.append((x0 + 0.025, (li - ld) / 2, li + ld))
        if len(reb) < 2:
            return None
        reb = np.array(reb); xv = float(np.median(reb[:, 0])); yc = float(np.median(reb[:, 1])); ancho = float(np.median(reb[:, 2]))
        m = (np.abs(p[:, 0] - xv) < 0.25) & (np.abs(p[:, 1] - yc) > ancho / 2 - 0.03) & (np.abs(p[:, 1] - yc) < ancho / 2 + 0.7)
        q = p[m]
        if (q[:, 1] > yc).sum() < 6 or (q[:, 1] < yc).sum() < 6:
            return None
        pend = 0.0
        for _ in range(3):                                   # x = c + pend·y, recortando lo que se aleje de la recta
            A = np.stack([np.ones(len(q)), q[:, 1]], axis=1); c, pend = np.linalg.lstsq(A, q[:, 0], rcond=None)[0]
            res = np.abs(q[:, 0] - (c + pend * q[:, 1])); q = q[res <= max(0.03, np.percentile(res, 75))]
            if len(q) < 8: return None
        return xv, yc, ancho, -math.atan(pend)

    def ciclo(self, eje):
        """eje = (px, py, psi) del eje del vano en el marco odom, o None. Devuelve el eje actualizado y el mando."""
        hi, hd = self.holgura(+1), self.holgura(-1)
        x, y, yaw = self.odom; d = self.detecta_vano(); modo = "memoria"
        # El eje sólo se actualiza con el vano a más de 0,6 m: más cerca las jambas se ven de refilón y con pocas
        # rebanadas, la estimación empeora justo cuando menos margen hay, y un guiño de 3° dentro del marco son 2 cm
        # de esquina (medido en el banco: 6 choques de 12 actualizando hasta 0,25 m; 0 de 8 sin hacerlo).
        if d is not None and d[0] > 0.6:
            xv, yc, ancho, e_th = d; c, s_ = math.cos(yaw), math.sin(yaw)
            nuevo = (x + xv * c - yc * s_, y + xv * s_ + yc * c, yaw + e_th)
            if eje is None: eje = nuevo
            else:                                            # paso bajo: cada scan mueve el eje un 30 %
                k = 0.2; dpsi = (nuevo[2] - eje[2] + math.pi) % (2 * math.pi) - math.pi
                eje = (eje[0] + k * (nuevo[0] - eje[0]), eje[1] + k * (nuevo[1] - eje[1]), eje[2] + k * dpsi)
            modo = "vano"
        if eje is None:
            return eje, hi, hd, None, 0.0, 0.0, 0, "sin_vano"
        px, py, psi = eje
        e_lat = -(px - x) * math.sin(psi) + (py - y) * math.cos(psi)      # + = el eje queda a la izquierda del robot
        e_th = (psi - yaw + math.pi) % (2 * math.pi) - math.pi
        w_tope = W_MAX if modo == "vano" else 0.10           # con el eje congelado (cerca o dentro), giros suaves
        w = max(-w_tope, min(w_tope, K_Y * e_lat + K_TH * e_th))
        p = self.pts
        n_front = int(((p[:, 0] > MEDIO_LARGO + 0.03) & (p[:, 0] < MEDIO_LARGO + FRENTE) & (np.abs(p[:, 1]) < MEDIO_ANCHO - 0.03)).sum())
        return eje, hi, hd, e_lat, e_th, w, n_front, modo

    def corre(self):
        a = self.a; t0 = time.time()
        while (self.pts is None or self.odom is None) and time.time() - t0 < 8:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.pts is None or self.odom is None:
            return "SIN_DATOS", 0.0
        self.odom0 = self.odom; eje = None; motivo = "TIEMPO"; ultimo_scan = 0.0; front_prev = 0
        v_ahora = 0.0
        while time.time() - t0 < a.tope:
            rclpy.spin_once(self, timeout_sec=0.05)
            dist = math.hypot(self.odom[0] - self.odom0[0], self.odom[1] - self.odom0[1])
            if self.choque: motivo = "CHOQUE"; break
            if dist >= a.metros: motivo = "LLEGÓ"; break
            if time.time() - self.t_scan > SCAN_MUDO: motivo = "LASER_MUDO"; break
            if self.t_scan == ultimo_scan:
                self.publica(v_ahora, self.w_prev if hasattr(self, "w_prev") else 0.0); continue
            ultimo_scan = self.t_scan
            eje, hi, hd, e_y, e_th, w, n_front, modo = self.ciclo(eje)
            if modo == "sin_vano":
                if time.time() - t0 > 6: motivo = "SIN_VANO"; break
                continue
            if n_front >= PASILLO_N and front_prev >= PASILLO_N: motivo = "OBSTACULO"; break
            front_prev = n_front
            hmin = np.nanmin([hi, hd]) if not (math.isnan(hi) and math.isnan(hd)) else float("nan")
            if not math.isnan(hmin) and hmin < HOLGURA_MIN:
                self.sin_holg = getattr(self, "sin_holg", 0) + 1
                if self.sin_holg >= 2: motivo = "SIN_HOLGURA"; break      # dos scans seguidos: uno solo puede ser un atípico
            else:
                self.sin_holg = 0
            # primero se orienta y se centra; sólo avanza a velocidad plena cuando va por el eje
            v_ahora = a.v if abs(e_th) < math.radians(3) and abs(e_y) < 0.02 else a.v * 0.4
            self.w_prev = w; self.publica(v_ahora, w)
            self.filas.append([round(time.time() - t0, 2), round(dist, 3), hi, hd, e_y, math.degrees(e_th), v_ahora, w, n_front, modo, self.verdad_holg])
        for _ in range(5):
            self.publica(0.0, 0.0); rclpy.spin_once(self, timeout_sec=0.05)
        return motivo, math.hypot(self.odom[0] - self.odom0[0], self.odom[1] - self.odom0[1])

    def publica(self, v, w):
        m = Twist(); m.linear.x = float(v); m.angular.z = float(w); self.cmd.publish(m)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--metros", type=float, default=1.35); ap.add_argument("--v", type=float, default=0.06)
    ap.add_argument("--tope", type=float, default=90.0); ap.add_argument("--csv", default=None)
    ap.add_argument("--scan", default="/robot/top_laser/scan"); ap.add_argument("--odom", default="/robot/robotnik_base_control/odom")
    ap.add_argument("--cmd", default="/robot/move_base/cmd_vel")   # /robot/cmd_vel NO tiene suscriptores en el robot real (medido 18-sep)
    a = ap.parse_args(); rclpy.init(); n = Cruce(a)
    try:
        motivo, dist = n.corre()
    finally:
        n.publica(0.0, 0.0)
    f = n.filas
    hi = [x[2] for x in f if not math.isnan(x[2])]; hd = [x[3] for x in f if not math.isnan(x[3])]
    vh = [x[10] for x in f if not math.isnan(x[10])]
    print("cruce: %s · %.2f m de %.2f · holgura MEDIDA mín izq %.0f mm / der %.0f mm%s · ciclos %d (viendo el vano %d, de memoria %d)"
          % (motivo, dist, a.metros, 1000 * min(hi) if hi else float("nan"), 1000 * min(hd) if hd else float("nan"),
             (" · holgura VERDAD mín %.0f mm" % (1000 * min(vh))) if vh else "", len(f),
             sum(1 for x in f if x[9] == "vano"), sum(1 for x in f if x[9] == "memoria")))
    if a.csv:
        with open(a.csv, "w", newline="") as fh:
            w = csv.writer(fh); w.writerow(["t", "dist", "holg_izq", "holg_der", "e_lat", "e_rumbo_deg", "v", "w", "n_front", "modo", "holg_verdad"]); w.writerows(f)
    rclpy.shutdown(); sys.exit(0 if motivo == "LLEGÓ" else 1)


if __name__ == "__main__":
    main()
