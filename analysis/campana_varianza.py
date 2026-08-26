"""Campana de VARIANZA del gemelo Isaac: N tramos encadenados, sin tocar parametros.

Resuelve la salvedad declarada en el analisis de realismo ("4 runs afinados contra 132 reales:
demuestra que el gemelo esta en la distribucion correcta, no que su varianza este calibrada").
Aqui se acumulan tramos suficientes para comparar DISPERSION, no solo medianas, y se guarda
la lista de runs para que el analisis los identifique sin adivinar por marca de tiempo.

    N=30 python3 analysis/campana_varianza.py
"""
import json, os, subprocess, sys, time

N = int(os.environ.get("N", "20"))
ENV = dict(os.environ, G1_SIM_URL=os.environ.get("G1_SIM_URL", "ws://localhost:8766"),
           G1_SIM_LUZ="116", G1_DOOR_VIS="1", G1_SIM_PERC="1", G1_NOVIS="0",
           G1_PERC="127.0.0.1:8010", G1_SIM_NOISE="1", G1_DOOR_CTR2="1",
           G1_DOOR_CTR_HOLD="0.15", G1_DOOR_YAW2="1", G1_DOOR_EXIT_CTR="1",
           G1_DOOR_CTR_TOL="0.07")
RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SALIDA = os.path.join(RAIZ, "dataset", "campana_isaac.json")

hechos = []
if os.path.exists(SALIDA):
    hechos = json.load(open(SALIDA)).get("runs", [])
    print("continuando campana: %d tramos ya hechos" % len(hechos), flush=True)

destino = "B"
for i in range(len(hechos), N):
    antes = set(os.listdir(os.path.join(RAIZ, "dataset")))
    t0 = time.time()
    r = subprocess.run(["timeout", "420", "python3", "g1_sim_adapter.py", "goto", destino],
                       cwd=RAIZ, env=ENV, capture_output=True, text=True)
    nuevos = [f for f in os.listdir(os.path.join(RAIZ, "dataset"))
              if f not in antes and f.endswith("_ours_%s.json" % destino)]
    ok = [l for l in r.stdout.splitlines() if "LLEGADO" in l or "NO LLEGA" in l]
    est = "reached" if any("LLEGADO" in l for l in ok) else "fallo"
    hechos.append({"i": i, "destino": destino, "fichero": nuevos[0] if nuevos else None,
                   "estado": est, "wall_s": round(time.time()-t0, 1)})
    json.dump({"runs": hechos, "n_objetivo": N}, open(SALIDA, "w"), indent=1)
    print("[%2d/%2d] -> %s : %s (%s, %.0fs de reloj)" % (
        i+1, N, destino, est, nuevos[0] if nuevos else "sin fichero",
        time.time()-t0), flush=True)
    destino = "A" if destino == "B" else "B"
    time.sleep(1.5)
print("campana completa: %d tramos" % len(hechos))
