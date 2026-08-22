"""Firma del cristal v9 (patron resuelto) en FRONTAL y OBLICUA, desfase fijo 0."""
import json, math

camp = json.load(open("/home/ros/isaac_ws/campana_cristal.json"))
R0, R1 = (-3.75, -0.55), (-2.65, 0.75)

def firma(nombre, objetivo):
    d = camp[nombre]
    sim = {int(k): v for k, v in d["perfil"].items()}
    px, py = d["pose"]
    esq = [(R0[0], R0[1]), (R0[0], R1[1]), (R1[0], R0[1]), (R1[0], R1[1])]
    bs_ = sorted(math.degrees(math.atan2(y-py, x-px)) % 360 for x, y in esq)
    # ventana continua (ojo al cruce de 0)
    if bs_[-1] - bs_[0] > 180:
        bs_ = [b - 360 if b > 180 else b for b in bs_]
        bs_.sort()
    b_lo, b_hi = int(bs_[0]//2)*2, int(bs_[-1]//2)*2
    total = con = 0
    for b in range(b_lo, b_hi + 2, 2):
        total += 1
        if sim.get(b % 360) is not None and sim[b % 360] <= 3.2:
            con += 1
    sin = total - con
    print("%-8s ventana %d..%d (%d sectores): sin retorno %d = %.0f%%  [real: %d%%]" % (
        nombre, b_lo, b_hi, total, sin, 100*sin/total, objetivo))

firma("FRONTAL", 44)
firma("OBLICUA", 32)
