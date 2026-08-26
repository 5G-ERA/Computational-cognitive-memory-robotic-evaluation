"""Analisis de la campana: calibrar desfase en A y medir la firma del cristal FRONTAL."""
import json, math, statistics

OC = 0.2
DOOR = (-3.90, 1.25); DOOR_R = 0.55
camp = json.load(open("/home/ros/isaac_ws/campana_cristal.json"))

# --- estructura para el ray-march de prediccion (mismas fuentes que el generador) ---
pared = {(round(p[0]/OC), round(p[1]/OC))
         for p in json.load(open("/home/ros/isaac_ws/ref_map_g1.json"))["points"]}
nav = json.load(open("/home/ros/isaac_ws/nav_map.json"))
mueble = {(int(c[0]), int(c[1])) for c in nav.get("cells", [])}
occ = {c for c in (pared | mueble)
       if math.hypot(c[0]*OC - DOOR[0], c[1]*OC - DOOR[1]) >= DOOR_R}

def predice(px, py, maxr=4.6):
    out = {}
    for b in range(0, 360, 2):
        a = math.radians(b + 1.0)
        r = 0.15
        while r <= maxr:
            c = (round((px + r*math.cos(a))/OC), round((py + r*math.sin(a))/OC))
            if c in occ:
                out[b] = r
                break
            r += 0.05
    return out

# --- pose oblicua: ¿esta libre? ---
for cand in ((-1.47, 1.10), (-1.47, -0.90), (-1.30, -0.75)):
    c = (round(cand[0]/OC), round(cand[1]/OC))
    vecinos = sum(1 for dx in (-1,0,1) for dy in (-1,0,1) if (c[0]+dx, c[1]+dy) in occ)
    print("pose %s: celda ocupada=%s, vecinos ocupados=%d" % (cand, c in occ, vecinos))

# --- calibracion del desfase en A ---
A = camp["A"]
simA = {int(k): v for k, v in A["perfil"].items()}
predA = predice(*A["pose"])
mejor = None
for des in range(0, 360, 2):
    errs = [abs(simA[(b+des) % 360] - r) for b, r in predA.items() if (b+des) % 360 in simA]
    if len(errs) >= 40:
        m = statistics.median(errs)
        if mejor is None or m < mejor[1]:
            mejor = (des, m, len(errs))
DES, med, ncom = mejor
print("\nDESFASE CALIBRADO: %d grados (mediana |dr| %.2f m sobre %d sectores)" % (DES, med, ncom))
json.dump({"desfase": DES}, open("/home/ros/isaac_ws/lidar_offset.json", "w"))

# --- firma del cristal FRONTAL con el desfase FIJO ---
F = camp["FRONTAL"]
simF = {int(k): v for k, v in F["perfil"].items()}
px, py = F["pose"]
R0, R1 = (-3.75, -0.55), (-2.65, 0.75)
esq = [(R0[0], R0[1]), (R0[0], R1[1]), (R1[0], R0[1]), (R1[0], R1[1])]
brgs = sorted(math.degrees(math.atan2(y-py, x-px)) % 360 for x, y in esq)
b_lo, b_hi = int(brgs[0]//2)*2, int(brgs[-1]//2)*2
print("\nventana geometrica del cristal desde FRONTAL: %d..%d" % (b_lo, b_hi))
total = con_retorno = a_distancia = 0
for b in range(b_lo, b_hi + 2, 2):
    total += 1
    bs = (b + DES) % 360
    r = simF.get(bs)
    if r is not None:
        con_retorno += 1
        if 1.2 <= r <= 2.9:
            a_distancia += 1
print("=== FIRMA FRONTAL (desfase fijo, sin ajuste por scan) ===")
print("sectores: %d | con retorno: %d | sin retorno: %d (%.0f%%)" % (
    total, con_retorno, total - con_retorno, 100*(total-con_retorno)/total))
print("objetivo real: 44%% sin retorno de frente")
