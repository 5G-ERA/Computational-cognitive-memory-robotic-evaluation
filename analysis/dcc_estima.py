#!/usr/bin/env python3
"""Estimacion confirmatoria del nivel de replay: lo que la seccion 8.3.8 del paper exige poblar.

El paper (A Computational Theory of Cognitive Memory, V4.3, supl. 8.3.8) no admite ninguna
afirmacion confirmatoria de la Fase 2 hasta reportar: poblacion de analisis, estimaciones
C1-C4, el contraste prerregistrado con su intervalo de incertidumbre, manejo de datos
faltantes e intervenciones de seguridad. Este script produce exactamente eso desde los
guiones del nivel de replay.

DECISIONES CONGELADAS (antes de mirar resultados confirmatorios; hoy corre sobre material de
DESARROLLO del 20-ago y se declara como tal):

  - UNIDAD: el run/configuracion, no la muestra (decision de Renxi). La estimacion por
    condicion es la media NO ponderada de los aciertos por run.
  - INCERTIDUMBRE: bootstrap percentil sobre runs, 10.000 replicas, semilla 7 congelada --
    el mismo metodo que la Fase 1 del paper (que uso bootstrap agrupado por memoria).
  - DATOS FALTANTES: una muestra sin tramo prescrito no entra; una muestra cuyo campo de
    evidencia es None resuelve con lo que queda (el resolutor ya trata None como ausencia).
    Un run con <5 muestras puntuables se EXCLUYE y se reporta.
  - SEGURIDAD: colisiones y paradas del dataset de cada run, reportadas por separado del
    acierto (la seccion 8.3.7 exige no mezclar estabilidad con resolucion).

SECUNDARIAS de replay (seccion 9.3 del protocolo), solo sobre tramos con transicion prescrita:
  - retardo de cambio: primer instante con salida correcta tras el limite t0 del tramo.
  - persistencia falsa: fraccion del tramo POSTERIOR a una transicion en que se sigue
    resolviendo el rol anterior.
  - retorno: acierto en el tramo que devuelve al titular tras cesar la condicion temporal.

Uso:  python3 analysis/dcc_estima.py [--covmap MAPA.json] guiones/*.json
"""
import json
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dcc_roles as R                                            # noqa: E402
import cov_g1                                                    # noqa: E402
import dcc_score as DS                                           # noqa: E402

CONDS = ("C1", "C2", "C3", "C4")
SEMILLA = 7
REPLICAS = 10000
MIN_MUESTRAS = 5


def puntua_run(guion, covmap):
    d = DS.carga_run(guion["run"])
    ss = cov_g1.aplica(d, covmap, borra=guion.get("cristal")) if covmap is not None \
        else (d.get("samples") or [])
    ac = {c: [0, 0] for c in CONDS}
    salidas = {c: [] for c in CONDS}              # (t, prescrito, salida) para las secundarias
    for m in ss:
        pres, _ = DS.prescrito_en(guion["tramos"], m.get("t", -1))
        if pres is None:
            continue
        for c in CONDS:
            r = R.condicion(m, c)
            ac[c][0] += DS.acierta(c, pres, r["out"])
            ac[c][1] += 1
            salidas[c].append((m.get("t"), pres, r["out"]))
    ev = d.get("events") or []
    seguridad = {"colisiones": sum(1 for e in ev if e.get("kind") == "collision"),
                 "paradas": sum(1 for e in ev if e.get("kind") in ("estop", "operator_stop"))}
    return ac, salidas, seguridad


def secundarias(guion, salidas):
    """Solo tienen sentido si el guion prescribe una transicion (mas de un delta distinto)."""
    tramos = guion["tramos"]
    deltas = [t["delta"] for t in tramos]
    out = {}
    for i, tr in enumerate(tramos):
        if i == 0 or tr["delta"] == tramos[i - 1]["delta"]:
            continue
        for c in CONDS:
            dentro = [(t, p, z) for (t, p, z) in salidas[c] if tr["t0"] <= t < tr["t1"]]
            if not dentro:
                continue
            correcto = [t for (t, p, z) in dentro if DS.acierta(c, tr["delta"], z)]
            clave = "%s@%s" % (tr["delta"], c)
            out.setdefault(clave, {})
            out[clave]["retardo_s"] = round(min(correcto) - tr["t0"], 1) if correcto else None
            prev = tramos[i - 1]["delta"]
            persiste = sum(1 for (t, p, z) in dentro
                           if DS.acierta(c, prev, z) and not DS.acierta(c, tr["delta"], z))
            out[clave]["persistencia_falsa"] = round(persiste / len(dentro), 2)
    return out


def ic_bootstrap(porrun, f, seed=SEMILLA, n=REPLICAS):
    """IC 95% percentil del estadistico f(lista de filas por run), remuestreando RUNS."""
    rng = random.Random(seed)
    vals = []
    m = len(porrun)
    for _ in range(n):
        vals.append(f([porrun[rng.randrange(m)] for _ in range(m)]))
    vals.sort()
    return vals[int(0.025 * n)], vals[int(0.975 * n)]


def main():
    args = sys.argv[1:]
    covmap = None
    if "--covmap" in args:
        i = args.index("--covmap")
        covmap = cov_g1.celdas(args[i + 1])
        del args[i:i + 2]
    if not args:
        raise SystemExit(__doc__)

    porrun = []                                   # una fila por run: {cond: acierto medio}
    excluidos = []
    seg_tot = {"colisiones": 0, "paradas": 0}
    sec_todas = []
    for g in args:
        guion = json.load(open(g))
        ac, salidas, seg = puntua_run(guion, covmap)
        if ac["C4"][1] < MIN_MUESTRAS:
            excluidos.append((guion["run"], ac["C4"][1]))
            continue
        porrun.append({c: ac[c][0] / ac[c][1] for c in CONDS})
        for k in seg_tot:
            seg_tot[k] += seg[k]
        s = secundarias(guion, salidas)
        if s:
            sec_todas.append((guion["run"], s))

    print("POBLACION: %d runs puntuados, %d excluidos (<%d muestras): %s"
          % (len(porrun), len(excluidos), MIN_MUESTRAS, excluidos or "-"))
    print("Unidad = run (media no ponderada). Bootstrap percentil %d replicas, semilla %d.\n"
          % (REPLICAS, SEMILLA))

    est = {}
    for c in CONDS:
        vals = [r[c] for r in porrun]
        est[c] = sum(vals) / len(vals)
        lo, hi = ic_bootstrap(porrun, lambda rs, c=c: sum(r[c] for r in rs) / len(rs))
        print("  %s: %5.1f%%   IC95 [%5.1f, %5.1f]" % (c, 100 * est[c], 100 * lo, 100 * hi))

    print("\nCONTRASTES (por run, IC95 bootstrap):")
    for a, b, nombre in (("C4", "C3", "DCC con la informacion revisada (PRINCIPAL)"),
                         ("C4", "C2", "la interfaz revisada, con resolucion (PRINCIPAL)"),
                         ("C3", "C1", "la interfaz, para el verificador"),
                         ("C4", "C1", "el cambio de paradigma completo")):
        d = 100 * (est[a] - est[b])
        lo, hi = ic_bootstrap(porrun, lambda rs, a=a, b=b:
                              sum(r[a] - r[b] for r in rs) / len(rs))
        print("  %s-%s: %+6.1f pp  IC95 [%+6.1f, %+6.1f]   %s"
              % (a, b, d, 100 * lo, 100 * hi, nombre))

    print("\nSEGURIDAD (aparte del acierto): colisiones %d, paradas %d en los runs puntuados."
          % (seg_tot["colisiones"], seg_tot["paradas"]))

    if sec_todas:
        print("\nSECUNDARIAS en tramos con transicion prescrita:")
        for run, s in sec_todas:
            for clave, v in sorted(s.items()):
                print("  %s  %s: retardo %s s, persistencia falsa %s"
                      % (run[9:15], clave, v.get("retardo_s"), v.get("persistencia_falsa")))


if __name__ == "__main__":
    main()
