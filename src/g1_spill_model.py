#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
g1_spill_model.py — Modelo FISICO de derrame de la taza para la simulacion (condicion payload).

Fisica (no un umbral arbitrario): el agua en una taza cilindrica tiene un primer modo de
sloshing con frecuencia natural  w0 = sqrt(g*k1*tanh(k1*h)),  k1 = 1.841/R  (Ibrahim, Liquid
Sloshing Dynamics). Se modela la elevacion de la superficie en el borde (eta, en metros) como
un oscilador lineal amortiguado 2D excitado por la aceleracion HORIZONTAL de la taza:

    eta_ss = -R*a/g          (inclinar la superficie: elevacion estacionaria en el borde)
    eta''  = -2*z*w0*eta' - w0^2*(eta - eta_ss)      (por eje del cuerpo, con overshoot ~1.8x)

La taza va en la mano (~0.25 m por delante del eje): girar rapido anade el termino de brazo
a = a_com + dw x r + w x (w x r). La MARCHA BIPEDA anade ruido de aceleracion proporcional a
la velocidad (bounce del paso, ~1.4 Hz + banda ancha): andar RAPIDO agita per se. Una PARADA
brusca (colision) mete un impulso directo eta += k_c*dv (la energia cinetica del agua pasa a
slosh). La aceleracion comandada se filtra con tau=0.25 s (rampa real del G1 + complianza del
brazo: el planar_move de la sim escalona la velocidad instantaneamente y eso NO es fisico).

Derrame = proceso de Poisson no homogeneo: lambda = L0*max(0, |eta|/h_f - 1)^2 con h_f el
francobordo (agua 12 mm bajo el borde). Cada derrame pierde agua (h_f += 4 mm) y disipa
energia (eta *= 0.5). Se registran ademas metricas CONTINUAS (mejores con N pequeno):
E[derrames] = integral de lambda dt, ratio maximo |eta|/h_f y % de tiempo en riesgo (>0.7).

USO en la sim: lo engancha g1_sim_adapter (thread sobre /odom; eventos kind='spill' al
RunRecorder, resumen spills_sim/spill_expected/... en summary). Solo OBSERVA: no toca fisica.
Reproducible: semilla = nombre del run (G1_SPILL_SEED la fija).

Replay offline (calibracion):  python g1_spill_model.py dataset/<run>.json [...]
"""
import json
import math
import os
import random
import sys

G = 9.81

# --- parametros fisicos (env-ajustables, defaults realistas taza de agua) ---
R_CUP = float(os.environ.get("G1_SPILL_R", "0.04"))          # radio de la taza (m)
H_FILL = float(os.environ.get("G1_SPILL_H", "0.08"))         # altura de agua (m)
FREEBOARD = float(os.environ.get("G1_SPILL_FB", "0.012"))    # francobordo inicial (m)
ZETA = float(os.environ.get("G1_SPILL_ZETA", "0.06"))        # amortiguamiento del agua
ARM_R = float(os.environ.get("G1_SPILL_ARM", "0.25"))        # offset de la mano (m, adelante)
TAU_A = float(os.environ.get("G1_SPILL_TAU", "0.25"))        # filtro de aceleracion (rampa G1 + brazo)
K_GAIT = float(os.environ.get("G1_SPILL_KGAIT", "2.2"))      # ruido de marcha: sigma_a = K*v^2/0.30 (la energia
                                                              # del paso crece ~v^2: andar rapido agita MUCHO mas)
F_GAIT = 1.4                                                  # Hz del paso (componente periodica)
K_COLL = float(os.environ.get("G1_SPILL_KC", "0.08"))        # impulso de parada brusca: eta += K*dv (s)
A_COLL = float(os.environ.get("G1_SPILL_ACOLL", "4.0"))      # umbral de IMPACTO (m/s^2 brutos). OJO: el
                                                              # planar_move escalona 0->0.3 en un tick (~3
                                                              # m/s^2): eso es arranque, no golpe. Impacto
                                                              # real = parada en <0.1 s -> >4.
V_SANE = 0.6                                                  # v > esto = glitch de pose (el G1 no corre)
WARMUP_S = 1.5                                                # ignorar el arranque del run (spawn/aligns)
LAM0 = float(os.environ.get("G1_SPILL_LAM0", "40.0"))         # tasa base de derrame (1/s) al doblar h_f
LID = os.environ.get("G1_SPILL_LID", "open").strip().lower()  # open | closed (travel mug sellada)
LID_FACTOR = 0.25 if LID.startswith("c") else 1.0             # tapa cerrada: riesgo x0.25 (tesis 5.3.6.4)
FB_LOSS = 0.004                                               # agua perdida por derrame (m de francobordo)


class SpillModel:
    """Integra el slosh sobre la pose del odom; emite eventos de derrame y metricas continuas."""

    def __init__(self, seed=None):
        k1 = 1.841 / R_CUP
        self.w0 = math.sqrt(G * k1 * math.tanh(k1 * H_FILL))   # ~21 rad/s (3.4 Hz) para R=4cm
        self.rng = random.Random(seed)
        self.fb = FREEBOARD
        self.ex = self.ev_x = 0.0     # eta y eta' por eje del CUERPO (x=avance, y=lateral)
        self.ey = self.ev_y = 0.0
        self.ax_f = self.ay_f = 0.0   # aceleracion filtrada (drive)
        self.prev = None              # (t, x, y, yaw) anterior
        self.prev_v = None            # (vx, vy, w) anterior (mundo)
        self.t_start = None           # primer t visto (warmup)
        self.cmd_speed = 0.0          # velocidad COMANDADA actual (set_cmd): distingue impacto
                                      # real (decelera con comando de avance vivo) de parada
                                      # comandada (el planar_move escalona a 0 y parece golpe)
        self.t_walk = 0.0             # fase de marcha acumulada
        # salidas
        self.spills = []              # [{t, eta_ratio, v, note}]
        self.expected = 0.0           # integral de lambda dt
        self.eta_max_ratio = 0.0
        self.t_risk = 0.0             # tiempo con |eta| > 0.7*h_f
        self.t_total = 0.0

    def set_cmd(self, vx, vy, wz=0.0):
        """Velocidad comandada actual (m/s). El canal de impulso solo dispara si el robot
        decelera MIENTRAS el comando sigue pidiendo avance (= golpe externo)."""
        self.cmd_speed = math.hypot(vx, vy)

    def step(self, t, x, y, yaw_rad):
        """Alimentar con la pose del odom (t en s; yaw en rad). Devuelve derrame o None."""
        if self.prev is None:
            self.prev = (t, x, y, yaw_rad)
            self.t_start = t
            return None
        t0, x0, y0, yaw0 = self.prev
        dt = t - t0
        if dt <= 1e-4 or dt > 1.0:                 # hueco de datos: resetear referencia
            self.prev = (t, x, y, yaw_rad)
            self.prev_v = None
            return None
        vx, vy = (x - x0) / dt, (y - y0) / dt
        if t - self.t_start < WARMUP_S or math.hypot(vx, vy) > V_SANE:
            # arranque del run o glitch de pose (v imposible para el G1): no alimentar el modelo
            self.prev = (t, x, y, yaw_rad)
            self.prev_v = None
            return None
        dyaw = (yaw_rad - yaw0 + math.pi) % (2 * math.pi) - math.pi
        w = dyaw / dt
        spill = None
        if self.prev_v is not None:
            vx0, vy0, w0_ = self.prev_v
            ax, ay = (vx - vx0) / dt, (vy - vy0) / dt          # acel del cuerpo (mundo, SIN filtrar)
            dw = (w - w0_) / dt
            # taza en la mano: a_cup = a_com + dw x r + w x (w x r), r = ARM_R*(cos yaw, sin yaw)
            rx, ry = ARM_R * math.cos(yaw_rad), ARM_R * math.sin(yaw_rad)
            acx = ax - dw * ry - w * w * rx
            acy = ay + dw * rx - w * w * ry
            # a body-frame (la superficie se inclina respecto al cuerpo que sostiene la taza)
            c, s = math.cos(yaw_rad), math.sin(yaw_rad)
            ab_x = c * acx + s * acy
            ab_y = -s * acx + c * acy
            # parada brusca (colision): impulso directo al slosh ANTES del filtro
            araw = math.hypot(ax, ay)
            v_now = math.hypot(vx, vy)
            if araw > A_COLL and self.cmd_speed > 0.15 and v_now < 0.5 * self.cmd_speed:
                # IMPACTO real: decelera bruscamente con el comando aun pidiendo avance.
                # (Una parada COMANDADA tambien escalona la v en la sim, pero cmd_speed ~0
                #  la suprime; el slosh de frenar ya entra por el canal filtrado TAU_A.)
                self.ex += K_COLL * ab_x * dt
                self.ey += K_COLL * ab_y * dt
            # filtro de rampa (el G1 real no escalona velocidad; la sim si)
            k = dt / (TAU_A + dt)
            self.ax_f += k * (ab_x - self.ax_f)
            self.ay_f += k * (ab_y - self.ay_f)
            # bounce de la marcha: periodico 1.4 Hz + banda ancha, amplitud ~ velocidad
            v = math.hypot(vx, vy)
            self.t_walk += dt * F_GAIT * 2 * math.pi
            a_gait = K_GAIT * v * v / 0.30                    # cuadratico en v (fisica del paso)
            gx = a_gait * (0.6 * math.sin(self.t_walk) + 0.8 * self.rng.gauss(0, 1))
            gy = a_gait * (0.6 * math.cos(self.t_walk * 0.5) + 0.8 * self.rng.gauss(0, 1))
            # integrar el oscilador (substeps: w0~21 rad/s pide dt<=0.02)
            n = max(1, int(dt / 0.02))
            h = dt / n
            for _ in range(n):
                tx = -R_CUP * (self.ax_f + gx) / G             # eta objetivo (estacionario)
                ty = -R_CUP * (self.ay_f + gy) / G
                aex = -2 * ZETA * self.w0 * self.ev_x - self.w0 ** 2 * (self.ex - tx)
                aey = -2 * ZETA * self.w0 * self.ev_y - self.w0 ** 2 * (self.ey - ty)
                self.ev_x += aex * h
                self.ev_y += aey * h
                self.ex += self.ev_x * h
                self.ey += self.ev_y * h
            eta = math.hypot(self.ex, self.ey)
            ratio = eta / self.fb
            self.eta_max_ratio = max(self.eta_max_ratio, ratio)
            self.t_total += dt
            if ratio > 0.7:
                self.t_risk += dt
            lam = LID_FACTOR * LAM0 * max(0.0, ratio - 1.0) ** 2
            self.expected += lam * dt
            if lam > 0 and self.rng.random() < 1.0 - math.exp(-lam * dt):
                spill = {"t": round(t, 2), "eta_ratio": round(ratio, 2),
                         "v": round(v, 3), "a": round(araw, 2)}
                self.spills.append(spill)
                self.fb += FB_LOSS                              # se pierde agua
                self.ex *= 0.5; self.ey *= 0.5                  # el salpicon disipa
        self.prev = (t, x, y, yaw_rad)
        self.prev_v = (vx, vy, w)
        return spill

    def summary(self):
        return {"spills_sim": len(self.spills),
                "spill_expected": round(self.expected, 3),
                "spill_eta_max": round(self.eta_max_ratio, 2),
                "spill_risk_pct": round(100.0 * self.t_risk / self.t_total, 1) if self.t_total else 0.0,
                "spill_lid": LID,
                "spill_params": {"R": R_CUP, "h_fill": H_FILL, "fb": FREEBOARD, "zeta": ZETA,
                                 "arm": ARM_R, "tau_a": TAU_A, "k_gait": K_GAIT,
                                 "k_coll": K_COLL, "lam0": LAM0}}


def replay(fname, seed=0):
    """Calibracion: pasar el modelo por la trayectoria de un run ya logueado (samples a ~10 Hz)."""
    d = json.load(open(fname))
    S = [s for s in d.get("samples", []) if s.get("x") is not None]
    m = SpillModel(seed=seed)
    for s in S:
        cmd = s.get("cmd") or [0, 0, 0]
        dz = lambda v: max(0.0, abs(v) - 0.10)
        m.set_cmd(dz(cmd[1] if len(cmd) > 1 else 0), dz(cmd[0] if cmd else 0))
        m.step(s["t"], s["x"], s["y"], math.radians(s.get("yaw", 0.0)))
    sm = m.summary()
    cols = sum(1 for e in d.get("events", []) if e.get("kind") == "collision")
    print("%-42s dur=%5.0fs col=%2d | spills=%d E[N]=%.2f eta_max=%.2f riesgo=%4.1f%%"
          % (os.path.basename(fname), S[-1]["t"] if S else 0, cols,
             sm["spills_sim"], sm["spill_expected"], sm["spill_eta_max"], sm["spill_risk_pct"]))
    return sm


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(0)
    for f in sys.argv[1:]:
        replay(f)
