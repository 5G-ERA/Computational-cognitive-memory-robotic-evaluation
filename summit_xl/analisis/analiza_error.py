#!/usr/bin/env python3
"""Resume el error de AMCL de uno o varios CSV de error_amcl.py, por distancia al vano. 17-sep-2026."""
import csv, math, sys

# Margen por lado MEDIDO con `experimento.py perfil` sobre el mapa corregido: la holgura de la huella real dentro del
# vano es constante y vale 50 mm. (La cuenta de plano, (742−613)/2 = 64,5 mm, sale algo mayor porque el vano
# reconstruido queda en 725 mm y no exactamente centrado.)
MARGEN = 0.050
filas = []
for r in sys.argv[1:]:
    # El CSV puede acabar en NUL si el proceso se cortó a media línea: se filtran los bytes nulos y la última línea.
    with open(r, errors="ignore") as f:
        texto = f.read().replace("\x00", "")
    for d in csv.DictReader(texto.splitlines()):
        try:
            filas.append({k: float(v) for k, v in d.items()})
        except (TypeError, ValueError):
            pass
# Marca los transitorios: tras un teletransporte AMCL tarda en reconverger sobre la pose que se le acaba de dar, y
# ese trozo no dice nada del seguimiento. Un salto de la VERDAD de más de 0,3 m entre muestras es un teletransporte.
SALTO, ESPERA, ARRANQUE = 0.3, 5.0, 10.0
filas = [f for f in filas if f["t"] >= ARRANQUE]     # los primeros segundos son el estado en que quedó la corrida anterior
ult_salto = -99.0
for i, f in enumerate(filas):
    if i and math.hypot(f["vx"] - filas[i-1]["vx"], f["vy"] - filas[i-1]["vy"]) > SALTO:
        ult_salto = f["t"]
    f["transitorio"] = (f["t"] - ult_salto) < ESPERA
est = [f for f in filas if not f["transitorio"]]
print("%d muestras de %d fichero(s) · %d en transitorio tras teletransporte (descartadas)"
      % (len(filas), len(sys.argv) - 1, len(filas) - len(est)))
filas = est

def tramo(nombre, ms):
    if not ms:
        print("  %-20s sin muestras" % nombre); return
    lat = sorted(abs(m["err_lat"]) for m in ms)
    tot = sorted(math.hypot(m["err_lat"], m["err_lon"]) for m in ms)
    yaw = sorted(abs(m["err_yaw"]) for m in ms)
    print("  %-20s n=%5d · lateral mediana %5.1f  p95 %5.1f  máx %5.1f mm · total máx %5.1f mm · rumbo p95 %4.1f°"
          % (nombre, len(ms), 1000 * lat[len(lat) // 2], 1000 * lat[int(.95 * (len(lat) - 1))], 1000 * lat[-1],
             1000 * tot[-1], yaw[int(.95 * (len(yaw) - 1))]))
    return lat

tramo("todo", filas)
tramo("a >3 m del vano", [m for m in filas if m["dist_vano"] > 3])
tramo("2-3 m del vano", [m for m in filas if 2 < m["dist_vano"] <= 3])
tramo("1-2 m del vano", [m for m in filas if 1 < m["dist_vano"] <= 2])
lat = tramo("EN EL VANO (<1 m)", [m for m in filas if m["dist_vano"] <= 1])
print("\n  margen por lado, medido con la huella real dentro del vano: %.1f mm" % (1000 * MARGEN))
if lat:
    peor = lat[-1]
    fuera = sum(1 for v in lat if v >= MARGEN)
    print("  error lateral en el vano: máximo %.1f mm · %d de %d muestras (%.0f %%) por encima del margen → %s"
          % (1000 * peor, fuera, len(lat), 100.0 * fuera / len(lat), "NO CABE" if fuera else "cabe"))
else:
    print("  sin muestras dentro del vano: la travesía no llegó")
