#!/usr/bin/env python3
"""Tablas de resultados LEGIBLES POR MAQUINA del paquete Experimento 2 (Note 8 §8.16).

Recalcula desde las runs grabadas + certificados (deterministico, sin robot) y escribe:
  reproducibility/resultados_stage2_dev.json   (todo: por run, agregados, contrastes)
  reproducibility/resultados_stage2_dev.csv    (una fila por run x condicion)

Etiquetado de condiciones y nombres de contrastes: V5.8 §8.6-8.7 (C2 = interfaz
original + resolucion distribuida; C3 = interfaz revisada + incumbente temporal;
efectos de interfaz C3-C1 y C4-C2; efectos de resolucion C2-C1 y C4-C3).
Tier: DESARROLLO (gemelo). El confirmatorio no existe y no se exporta.
"""
import csv
import json
import os
import statistics
import subprocess
import sys

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)
from dcc_conditions import evalua_todas, usa_pose_para, CONDICIONES, PROCESO, INTERFAZ
from dcc_omega import carga_referencia, puntua_run

MAN = os.path.join(RAIZ, "tasks", "manifiestos", "campana_dcc_v2.txt")


def runs_del_manifiesto():
    out = []
    for ln in open(MAN):
        if "|" not in ln or "COMPLETA" in ln:
            continue
        cfg, dst, f = ln.strip().split("|")
        if not f:
            continue
        fp = os.path.join(RAIZ, f)
        ref = fp.replace(".json", "_omega_ref.json")
        if not os.path.exists(ref):
            continue
        d = json.load(open(fp))
        if len(d.get("samples") or []) < 30:
            continue
        out.append((cfg, os.path.basename(f), d, ref))
    return out


def q(v, p):
    s = sorted(v)
    return s[min(len(s) - 1, int(p * len(s)))]


def main():
    filas = []
    for cfg, nombre, d, ref in runs_del_manifiesto():
        segs = carga_referencia(ref, d)
        r = puntua_run(d, segs, evalua_todas, usa_pose_para(d))
        fila = {"config": cfg, "run": nombre, "git_sha": (d.get("git") or {}).get("sha")}
        for c in CONDICIONES:
            a = r.get(c)
            fila[c + "_A_meta"] = round(a["meta"] / a["n"], 4) if a and a["n"] else None
            fila[c + "_A_omega"] = round(a["omega"] / a["n"], 4) if a and a["n"] else None
            fila[c + "_n"] = a["n"] if a else 0
        filas.append(fila)

    contrastes = {}
    NOMBRES = {  # V5.8 §8.7
        "C3-C1": "interface_effect_under_temporal",
        "C4-C2": "interface_effect_under_distributed",
        "C2-C1": "resolution_effect_under_original",
        "C4-C3": "resolution_effect_under_revised",
        "C4-C1": "diagonal_full_vs_baseline",
    }
    for par, nombre in NOMBRES.items():
        a, b = par.split("-")
        difs = [f[a + "_A_meta"] - f[b + "_A_meta"] for f in filas
                if f.get(a + "_A_meta") is not None and f.get(b + "_A_meta") is not None]
        difs.sort()
        n = len(difs)
        contrastes[par] = {
            "name": nombre, "n_runs": n,
            "median_pp": round(100 * statistics.median(difs), 1),
            "iqr_pp": [round(100 * q(difs, .25), 1), round(100 * q(difs, .75), 1)],
            "wins": sum(1 for x in difs if x > 0),
            "per_run_pp": [round(100 * x, 1) for x in difs],
        }

    agregado = {}
    for c in CONDICIONES:
        vals = [f[c + "_A_meta"] for f in filas if f.get(c + "_A_meta") is not None]
        agregado[c] = {
            "interface": "revised" if INTERFAZ[c] == "I1" else "original",
            "resolution": "distributed" if PROCESO[c] == "distribuida" else "temporal",
            "A_meta_median": round(statistics.median(vals), 4),
            "A_meta_iqr": [round(q(vals, .25), 4), round(q(vals, .75), 4)],
            "n_runs": len(vals),
        }

    sha = subprocess.run(["git", "-C", RAIZ, "rev-parse", "--short", "HEAD"],
                         capture_output=True, text=True).stdout.strip()
    out = {
        "package": "Experiment 2 (robotic development-to-deployment) - development tier",
        "paper": "A Computational Theory of Cognitive Memory V5.8, Supplementary Note 8",
        "tier": "development",
        "manifest": os.path.relpath(MAN, RAIZ),
        "repo_head": sha,
        "conditions": agregado,
        "prespecified_contrasts": contrastes,
        "per_run": filas,
        "caveats": [
            "development tier only: the frozen confirmatory C1-C4 deployment evaluation has not been run",
            "unit of analysis is the run; contrasts are within-run paired differences",
            "T10 excluded (not stageable in the kinematic twin, ledger D10); T12 verified as a record property",
        ],
    }
    dst = os.path.join(RAIZ, "reproducibility", "resultados_stage2_dev.json")
    json.dump(out, open(dst, "w"), indent=1)

    dst_csv = os.path.join(RAIZ, "reproducibility", "resultados_stage2_dev.csv")
    with open(dst_csv, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(filas[0].keys()))
        w.writeheader()
        w.writerows(filas)
    print("escrito %s (%d runs) y %s" % (os.path.relpath(dst, RAIZ), len(filas),
                                         os.path.relpath(dst_csv, RAIZ)))
    for par, cdat in contrastes.items():
        print("  %s (%s): %+.1f pp %s, %d/%d > 0" % (par, cdat["name"], cdat["median_pp"],
                                                     cdat["iqr_pp"], cdat["wins"], cdat["n_runs"]))


if __name__ == "__main__":
    main()
