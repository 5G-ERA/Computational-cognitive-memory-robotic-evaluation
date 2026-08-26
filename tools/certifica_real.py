#!/usr/bin/env python3
"""Certificado de referencia (Omega_t) para runs REALES escenificadas a mano.

En el gemelo, guion.py escenifica y certifica EN EL MISMO ACTO. En el robot real
quien mueve el mundo es el OPERADOR (apaga luces, colocá un objeto), y el acto
queda declarado por sus instantes de pared anotados. Este util convierte esas
anotaciones en el MISMO formato de certificado que escribe guion.py, junto a la
run, para que dcc_omega.carga_referencia lo puntue sin distinguir tier.

REGLA DE ORO (del 21-ago): el instante se anota leyendo EL RELOJ DE LA MAQUINA
QUE CONDUCE (date +%H:%M:%S en esa terminal), no un movil -- `started` de la run
viene de ese reloj y el mapeo es contra el. La gracia declarada del puntuador es
±2 s alrededor de cada frontera.

Uso:
  python3 tools/certifica_real.py --run dataset/20260827_1012xx_ours_B.json \
      --config T3R \
      --seg "inicio|motion|{\"luz\": 85}" \
      --seg "10:13:05|illumination|{\"luz\": 116}"

  --seg "T_PARED|delta|estado_json"   (repetible, en orden). T_PARED es HH:MM:SS
  del reloj de la maquina conductora (el dia se toma de la run), o la palabra
  "inicio" para el arranque de la run.
"""
import argparse
import json
import os
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True)
    ap.add_argument("--config", required=True)
    ap.add_argument("--seg", action="append", required=True,
                    help='"HH:MM:SS|delta|estado_json" o "inicio|delta|estado_json"')
    a = ap.parse_args()

    d = json.load(open(a.run))
    started = d["started"]                      # "YYYY-MM-DD HH:MM:SS", reloj local
    t0 = time.mktime(time.strptime(started, "%Y-%m-%d %H:%M:%S"))
    dia = started.split(" ")[0]

    segs = []
    for s in a.seg:
        t_s, delta, est = s.split("|", 2)
        if t_s.strip().lower() == "inicio":
            t_pared = t0
        else:
            t_pared = time.mktime(time.strptime(dia + " " + t_s.strip(),
                                                "%Y-%m-%d %H:%M:%S"))
        if t_pared < t0 - 2:
            raise SystemExit("segmento ANTES del arranque de la run: %s" % t_s)
        segs.append({"t_pared": t_pared, "delta": delta.strip(),
                     "estado": json.loads(est)})

    if [x["t_pared"] for x in segs] != sorted(x["t_pared"] for x in segs):
        raise SystemExit("segmentos desordenados en el tiempo")

    ref = {"config": a.config, "destino": d.get("label", "?"), "tier": "real",
           "origen": "operador (certifica_real): instantes de pared anotados del "
                     "reloj de la maquina conductora",
           "segmentos": segs}
    dst = a.run.replace(".json", "_omega_ref.json")
    if os.path.exists(dst):
        raise SystemExit("ya existe %s -- no se reescribe (no-rewriting); borra a mano "
                         "si de verdad toca" % dst)
    json.dump(ref, open(dst, "w"))
    print("certificado: %s  (%d segmentos, t0=%s)" % (dst, len(segs), started))
    for s in segs:
        print("  t_run=%+6.1fs  delta=%-14s estado=%s"
              % (s["t_pared"] - t0, s["delta"], json.dumps(s["estado"])))


if __name__ == "__main__":
    main()
