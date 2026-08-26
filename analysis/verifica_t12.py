#!/usr/bin/env python3
"""T12 - el mapeo sucesor sustituye al incumbente SIN reescribirlo (non-rewrite).

El protocolo lo define como propiedad del REGISTRO, no del mundo: "Pass is verified in
the record, not in behaviour: the original mapping must be recoverable after the
successor exists. kappa_t records both, with provenance and timestamps."

El evento real que lo instancia (25-ago): la referencia de visibilidad de SESION del
gemelo (dataset/visibilidad_gemelo_sesion.json) sustituyo como referencia operativa a
sus incumbentes -- la provisional del mismo dia (visibilidad_gemelo_sesion_prov.json)
y el mapa historico del Summit (summit/ref_map_g1.json) -- sin tocar ninguno de los
dos. Este verificador comprueba las tres condiciones del pass EN EL REGISTRO:

  1. SUCESOR con procedencia: existe, es legible, declara su base de evidencia (src)
     y esta versionado (commit conocido en git).
  2. INCUMBENTES recuperables: los ficheros originales siguen legibles tal cual
     (no reescritos: el sucesor es un fichero NUEVO) y su historia esta en git.
  3. kappa_t registra ambos: este certificado deja constancia de sucesor e
     incumbentes con procedencia y timestamps, y se versiona junto al codigo.

Escribe dataset/certificado_T12.json y sale 0 si PASS, 1 si FAIL.
"""
import json
import os
import subprocess
import sys
import time

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SUCESOR = os.path.join(RAIZ, "dataset", "visibilidad_gemelo_sesion.json")
INCUMBENTES = [
    os.path.join(RAIZ, "dataset", "visibilidad_gemelo_sesion_prov.json"),
    os.path.join(RAIZ, "summit", "ref_map_g1.json"),
]


def git(*args):
    p = subprocess.run(["git", "-C", RAIZ] + list(args),
                       capture_output=True, text=True, timeout=30)
    return p.stdout.strip()


def ficha(path):
    """Legibilidad + procedencia + historia git de un fichero del registro."""
    rel = os.path.relpath(path, RAIZ)
    out = {"fichero": rel, "legible": False, "src": None, "npts": None,
           "commit": None, "fecha_commit": None}
    try:
        d = json.load(open(path))
        out["legible"] = True
        out["src"] = d.get("src")
        out["npts"] = d.get("npts", len(d.get("points", []) or []) or None)
    except Exception as e:
        out["error"] = repr(e)
    log = git("log", "-1", "--format=%H|%cI", "--", rel)
    if log:
        out["commit"], out["fecha_commit"] = log.split("|")
    return out


def main():
    suc = ficha(SUCESOR)
    incs = [ficha(p) for p in INCUMBENTES]

    fallos = []
    if not (suc["legible"] and suc["src"] and suc["commit"]):
        fallos.append("sucesor sin legibilidad/procedencia/version: %s" % suc)
    for i in incs:
        if not i["legible"]:
            fallos.append("incumbente NO recuperable: %s" % i)
        if not i["commit"]:
            fallos.append("incumbente sin historia git: %s" % i["fichero"])
    # non-rewrite explicito: sucesor e incumbentes son ficheros DISTINTOS
    if any(os.path.samefile(SUCESOR, p) for p in INCUMBENTES if os.path.exists(p)):
        fallos.append("el sucesor reescribio a un incumbente (mismo fichero)")

    cert = {
        "config": "T12",
        "definicion": "successor mapping supersedes incumbent (non-rewrite); "
                      "pass verificado en el registro, no en el comportamiento",
        "verificado": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "sucesor": suc,
        "incumbentes": incs,
        "pass": not fallos,
        "fallos": fallos,
    }
    dst = os.path.join(RAIZ, "dataset", "certificado_T12.json")
    json.dump(cert, open(dst, "w"), indent=1)
    print(json.dumps(cert, indent=1))
    print("\nT12:", "PASS" if not fallos else "FAIL", "->", os.path.relpath(dst, RAIZ))
    sys.exit(0 if not fallos else 1)


if __name__ == "__main__":
    main()
