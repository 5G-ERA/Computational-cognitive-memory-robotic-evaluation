#!/usr/bin/env python3
"""La foto completa de una sesion en un comando, para verla ANTES de salir del laboratorio.

    python3 analysis/resumen_sesion.py            # hoy
    python3 analysis/resumen_sesion.py 20260820   # otra fecha

Secciones: runs reales del dia (llegadas, colisiones, cruces) - cov_missing y K_online
sugerido - autoridad emitida (phase_sent) - muestreo de vision por etiqueta (el par W2) -
tabla de noisecheck (bloque del cristal) - pendientes de subir.
Solo LEE: no toca dataset ni estado.
"""
import collections
import glob
import json
import math
import os
import statistics
import subprocess
import sys
import time

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)
import dcc_roles as R                                            # noqa: E402

DX, DY = -3.90, 1.25
AX = math.radians(135.0)
UX, UY = math.cos(AX), math.sin(AX)


def med(v):
    v = sorted(x for x in v if isinstance(x, (int, float)))
    return v[len(v) // 2] if v else None


def main():
    dia = sys.argv[1] if len(sys.argv) > 1 else time.strftime("%Y%m%d")
    print("=" * 72)
    print("RESUMEN DE SESION %s" % dia)
    print("=" * 72)

    # --- runs reales del dia ---
    fs = sorted(glob.glob(os.path.join(RAIZ, "dataset", dia + "_*_ours_[AB].json")))
    reales = []
    for f in fs:
        try:
            d = json.load(open(f))
        except Exception:
            continue
        if d.get("sim_id"):
            continue
        reales.append((f, d))
    print("\n-- RUNS REALES: %d --" % len(reales))
    lleg = ncol = ncru = 0
    cm_normal = []
    marcas_tot = collections.Counter()
    aut_tot = collections.Counter()
    for f, d in reales:
        ss = d.get("samples") or []
        ev = d.get("events") or []
        s = d.get("summary") or {}
        g = d.get("goal") or {}
        dfin = math.hypot(ss[-1]["x"] - g.get("x", 0), ss[-1]["y"] - g.get("y", 0)) if ss else 9
        ok = dfin < 0.45
        col = sum(1 for e in ev if e.get("kind") == "collision")
        cru = sum(1 for e in ev if e.get("kind") == "door_crossed")
        lleg += ok; ncol += col; ncru += cru
        cms = [m.get("cov_missing") for m in ss if isinstance(m.get("cov_missing"), (int, float))]
        cm_normal += cms
        for m in ss:
            aut_tot[R.authority_of(R.vista(m, "I1"))[0]] += 1
            for mk in ("!H", "!D", "!S", "!C", "!c", "!M"):
                if mk in str(m.get("phase_sent") or ""):
                    marcas_tot[mk] += 1
        print("  %s -> %s  %-6s t=%ss col=%d cruces=%d  arm=%s  cov_missing max=%s" % (
            os.path.basename(f)[9:15], os.path.basename(f)[-6], "LLEGO" if ok else "FALLO",
            s.get("time_s", "?"), col, cru, (d.get("env_g1") or {}).get("G1_ARM", "-"),
            max(cms) if cms else "-"))
    if reales:
        print("  TOTAL: %d/%d llegadas, %d colisiones, %d cruces" % (lleg, len(reales), ncol, ncru))

    # --- cov_missing / K_online ---
    if cm_normal:
        c = collections.Counter(cm_normal)
        mx = max(cm_normal)
        print("\n-- cov_missing (todas las muestras): %s --" % dict(sorted(c.items())))
        print("  K_online sugerido = max_observado + 1 = %d  (provisional 3; congelar con Renxi)"
              % max(2, min(4, mx + 1)))
    else:
        print("\n-- cov_missing: SIN datos (G1_COVREF no exportado?) --")

    # --- autoridad ---
    if aut_tot:
        print("\n-- AUTORIDAD (phase_sent, primera emision real): %s --" % dict(aut_tot.most_common()))
        print("  marcas: %s" % dict(marcas_tot.most_common()))

    # --- muestreo de vision ---
    dfe = time.strftime("%Y-%m-%d", time.strptime(dia, "%Y%m%d"))
    reg = os.path.join(RAIZ, "calib_luz", dfe, "muestras.jsonl")
    if os.path.exists(reg):
        filas = [json.loads(l) for l in open(reg) if l.strip()]
        print("\n-- MUESTREO DE VISION (%d muestras) --" % len(filas))
        por = collections.defaultdict(list)
        for f in filas:
            por[f.get("etiqueta", "?")].append(f)
        for et, fl in por.items():
            sillas = []
            for f in fl:
                for dd in ((f.get("percepcion") or {}).get("detecciones") or []):
                    if dd[0] == "chair":
                        sillas.append(dd[1])
            hqs = [f["hq"]["imagen"].get("grano") for f in fl
                   if isinstance(f.get("hq"), dict) and isinstance(f["hq"].get("imagen"), dict)
                   and f["hq"]["imagen"].get("grano") is not None]
            print("  %-42s n=%-3d grano med %s  contraste med %s  silla %d/%d conf med %s  grano HQ med %s" % (
                et[:42], len(fl),
                med(f["imagen"]["grano"] for f in fl),
                med(f["imagen"]["contraste"] for f in fl),
                len(sillas), len(fl), round(med(sillas), 2) if sillas else "-",
                med(hqs) if hqs else "-"))
    else:
        print("\n-- MUESTREO DE VISION: sin muestras hoy --")

    # --- noisecheck (bloque cristal) ---
    nf = sorted(glob.glob(os.path.join(RAIZ, "dataset", dia + "_*_noise.json")))
    if nf:
        print("\n-- NOISECHECK (cristal real / pared) --")
        for f in nf:
            rows = (json.load(open(f)).get("rows")) or []
            cds = [r.get("cov_def") for r in rows if isinstance(r.get("cov_def"), (int, float))]
            if cds:
                sc = sorted(cds)
                print("  %s  cov_def mediana %.3f  p90 %.3f  (filas %d)" % (
                    os.path.basename(f), statistics.median(cds),
                    sc[int(0.9 * (len(sc) - 1))], len(rows)))
    else:
        print("\n-- NOISECHECK: ninguno hoy --")

    # --- pendiente de subir ---
    try:
        st = subprocess.run(["git", "-C", RAIZ, "status", "--short"],
                            capture_output=True, text=True, timeout=10).stdout.strip()
        n = len([l for l in st.splitlines() if l.strip()])
        print("\n-- GIT: %d ficheros sin commitear%s --" % (n, "" if n == 0 else "  (bloque 5.2!)"))
    except Exception:
        pass
    print()


if __name__ == "__main__":
    main()
