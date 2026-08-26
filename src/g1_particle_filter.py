#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Filtro de particulas de REGIMEN para el stack META2 del G1 (Renxi 2026-07-21).

Marco acordado: Manager POMDP / Worker POMDP. Este modulo es el ESTIMADOR DE ESTADO del
Manager: mantiene un posterior sobre las variables de regimen que el meta-razonador no
observa directamente, y lo expone como creencias al pipeline QoE-analogias — que sigue
siendo LA POLITICA (el punto de Renxi 11:00: "combining the decision-making policy directly
with QoE metrics into analogies... without relying on RL"). Aqui NO hay RL ni optimizacion
de MDP: el PF estima; la QoE decide.

Estado por particula (hibrido continuo x discreto — por esto es un PF y no un HMM exacto):
  lid   en {open, closed}   tapa del vaso (Renxi: "is the lid open or close") — INFERIDA
  fill  en [0, 1]           nivel de llenado (fisica del francobordo: lleno derrama mas;
                            baja ~0.033 por derrame ~ 7 g / 200 g medidos en el real)
  sens  en {good, degraded} salud del sensado laser (ruido +65% girando, medido)
  bat   en {ok, low}        suficiencia de bateria (Renxi: "is the battery sufficient")
  stuck en {advancing, stuck}  progreso (Renxi: "is the robot stuck" — un ESTADO, como
                            apunto Adrian; la accion que licencia es explorar/escalar)

Observaciones por paso (todas ya logueadas por run): derrames nuevos (GT humana/sim),
velocidad y giro comandados/medidos, laser_noise, bateria, progression.

MODO SOMBRA (v1): el bridge calcula y loguea el posterior (meta2_pf por muestra) pero NO
altera ninguna decision — comparabilidad de brazos intacta. v2 (tras validar contra R4
tapa-sellada): el posterior condiciona expectativas QoE (p.ej. lid_open<0.2 -> la config
'covered_delivery' del paper).

Calibracion desde datos reales (2026-07-08/14): ~12 derrames/run abierto a ~90 s con
excitacion mixta; giros dominan el derrame de copa fresca; front-loading 3/3 reproducido.
CLI de replay: python g1_particle_filter.py replay dataset/<run>.json
"""
import json
import math
import random
import sys

N_DEFAULT = 400

# --- emision de derrame: tasa Poisson lambda(excitacion, fill, lid) [derrames/s] ---
BASE_RATE = 0.20      # a excitacion 1, fill~0.9, tapa abierta (≈12 derrames / 90 s reales)
LID_MULT = 0.15       # tapa cerrada: x0.15 (sellada real puede ser menos; recalibrar con R4)
EX_V = 0.30           # m/s que normalizan la excitacion lineal
EX_W = 0.40           # rad/s que normalizan el giro (dominante en copa fresca: peso 1.2)

# --- emision de laser_noise (gaussianas por regimen; medido: reposo~0.2, girando ~0.55+) ---
SENS_MU = (0.20, 0.55)
SENS_SD = (0.15, 0.20)

# --- transiciones por segundo ---
P_LID_FLIP = 0.001
P_SENS_DEG = 0.02     # + 0.15 * turning
P_SENS_REC = 0.10     # * (1 - turning)
P_STUCK_IN = 0.03
P_STUCK_OUT = 0.15
FILL_PER_SPILL = 0.033
FILL_NOISE = 0.002


def _gauss(x, mu, sd):
    return math.exp(-0.5 * ((x - mu) / sd) ** 2) / sd


class RegimePF:
    def __init__(self, n=N_DEFAULT, seed=7, lid_prior_open=0.5, fill_prior=0.90):
        self.rng = random.Random(seed)
        self.n = n
        self.p = []
        for _ in range(n):
            self.p.append({
                "lid": 0 if self.rng.random() < lid_prior_open else 1,   # 0=open 1=closed
                "fill": min(1.0, max(0.0, self.rng.gauss(fill_prior, 0.06))),
                "sens": 0 if self.rng.random() < 0.9 else 1,
                "bat": 0,                                                # 0=ok 1=low
                "stuck": 0 if self.rng.random() < 0.95 else 1,
            })
        self.w = [1.0 / n] * n
        self.t_last = None

    # ---------- paso ----------
    def step(self, now, spill_new=0, spd=None, wz=None, laser_noise=None,
             bat=None, progression=None):
        dt = 0.5 if self.t_last is None else max(0.05, min(3.0, now - self.t_last))
        self.t_last = now
        v = abs(spd) if spd is not None else 0.0
        w_ = abs(wz) if wz is not None else 0.0
        turning = min(1.0, w_ / EX_W)
        excitation = min(1.5, v / EX_V) + 1.2 * min(1.5, w_ / EX_W)

        # --- transicion ---
        r = self.rng.random
        for q in self.p:
            if r() < P_LID_FLIP * dt:
                q["lid"] ^= 1
            q["fill"] = min(1.0, max(0.0, q["fill"] - FILL_PER_SPILL * spill_new
                                     + self.rng.gauss(0.0, FILL_NOISE)))
            if q["sens"] == 0:
                if r() < (P_SENS_DEG + 0.15 * turning) * dt:
                    q["sens"] = 1
            else:
                if r() < P_SENS_REC * (1.0 - turning) * dt:
                    q["sens"] = 0
            if bat is not None:
                try:
                    q["bat"] = 1 if float(bat) < 25.0 else q["bat"]
                except Exception:
                    pass
            if q["stuck"] == 0:
                if r() < P_STUCK_IN * dt:
                    q["stuck"] = 1
            else:
                if r() < P_STUCK_OUT * dt:
                    q["stuck"] = 0

        # --- pesos (emision) ---
        for i, q in enumerate(self.p):
            wgt = self.w[i]
            # derrames: Poisson(lam*dt)
            lam = BASE_RATE * max(0.02, excitation) * max(0.05, q["fill"] ** 2)
            if q["lid"] == 1:
                lam *= LID_MULT
            mu = max(1e-6, lam * dt)
            k = int(spill_new)
            pk = math.exp(-mu) * mu ** k / math.factorial(min(k, 10))
            wgt *= max(1e-9, pk)
            if laser_noise is not None:
                wgt *= max(1e-9, _gauss(float(laser_noise), SENS_MU[q["sens"]], SENS_SD[q["sens"]]))
            if bat is not None:
                try:
                    b = float(bat)
                    pl = 1.0 / (1.0 + math.exp((b - 25.0) / 4.0))   # P(low | bat)
                    wgt *= (pl if q["bat"] == 1 else (1.0 - pl)) + 1e-9
                except Exception:
                    pass
            if progression is not None:
                pr = float(progression)
                wgt *= max(1e-9, _gauss(pr, 0.05, 0.10) if q["stuck"] else _gauss(pr, 0.60, 0.30))
            self.w[i] = wgt

        s = sum(self.w) or 1e-30
        self.w = [x / s for x in self.w]
        ess = 1.0 / sum(x * x for x in self.w)
        if ess < self.n / 2:
            self._resample()
        return self.posterior()

    def _resample(self):
        n = self.n
        pos = (self.rng.random() + 0) / n
        cum = 0.0
        idx = []
        j = 0
        c = self.w[0]
        for i in range(n):
            u = pos + i / n
            while u > c and j < n - 1:
                j += 1
                c += self.w[j]
            idx.append(j)
        self.p = [dict(self.p[j]) for j in idx]
        self.w = [1.0 / n] * n

    def posterior(self):
        n = self.n
        lid_open = sum(w for w, q in zip(self.w, self.p) if q["lid"] == 0)
        fill_m = sum(w * q["fill"] for w, q in zip(self.w, self.p))
        fill_s = math.sqrt(max(0.0, sum(w * (q["fill"] - fill_m) ** 2
                                        for w, q in zip(self.w, self.p))))
        return {"lid_open": round(lid_open, 3),
                "fill": round(fill_m, 3), "fill_std": round(fill_s, 3),
                "sens_good": round(sum(w for w, q in zip(self.w, self.p) if q["sens"] == 0), 3),
                "bat_ok": round(sum(w for w, q in zip(self.w, self.p) if q["bat"] == 0), 3),
                "stuck": round(sum(w for w, q in zip(self.w, self.p) if q["stuck"] == 1), 3)}


# ---------- replay offline sobre un run logueado ----------
def replay(fname, verbose=True):
    d = json.load(open(fname))
    sm = d.get("samples", [])
    spills = sorted(e.get("t", 0) for e in d.get("events", [])
                    if e.get("kind") in ("spill", "spill_human"))
    pf = RegimePF()
    out = []
    si = 0
    for s in sm:
        t = s.get("t", 0)
        k = 0
        while si < len(spills) and spills[si] <= t:
            k += 1
            si += 1
        cmd = s.get("cmd") or [0, 0, 0]
        post = pf.step(t, spill_new=k, spd=s.get("spd"),
                       wz=(cmd[2] if len(cmd) > 2 else None),
                       laser_noise=s.get("laser_noise"), bat=s.get("bat"),
                       progression=s.get("progression"))
        out.append((t, post))
    if verbose and out:
        print("%s  (%d muestras, %d derrames)" % (fname.split("/")[-1], len(sm), len(spills)))
        keyframes = [out[0]] + [out[min(len(out) - 1, int(len(out) * f))]
                                for f in (0.25, 0.5, 0.75)] + [out[-1]]
        for t, p in keyframes:
            print("  t=%6.1f  P(tapa abierta)=%.2f  fill=%.2f±%.2f  sens_ok=%.2f  stuck=%.2f"
                  % (t, p["lid_open"], p["fill"], p["fill_std"], p["sens_good"], p["stuck"]))
    return out


if __name__ == "__main__":
    if len(sys.argv) >= 3 and sys.argv[1] == "replay":
        for f in sys.argv[2:]:
            replay(f)
            print()
    else:
        print(__doc__)
