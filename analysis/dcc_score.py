#!/usr/bin/env python3
"""Puntuador del nivel de REPLAY: una grabacion, cuatro condiciones, los contrastes de Renxi.

Es el analisis primario del protocolo. Toma un episodio grabado y un GUION que declara lo que
debia resolverse en cada tramo, pasa la MISMA evidencia por C1..C4 y devuelve aciertos y
contrastes. La situacion fisica no se equilibra entre brazos: es identica, porque es la misma
grabacion. Eso es lo que pedia Renxi -- "the same physical trajectory and evidence represented in
different cognitive forms".

EL GUION ES LA REFERENCIA INDEPENDIENTE (Omega_t). Sale del diseno del experimento, no de los
sensores del robot: nosotros decidimos cuando se apaga la luz, si el paso esta acristalado o
abierto, que representante se ha desactivado. Por eso lo que DEBIA resolverse se sabe por
construccion y ningun arbitro lo infiere de la telemetria del sistema bajo prueba.

Formato del guion (JSON), tramos por tiempo de run:
    {"episodio": "W1-cristal", "run": "20260820_161500_ours_A",
     "tramos": [{"t0": 0,  "t1": 25, "delta": "motion",         "nota": "aproximacion normal"},
                {"t0": 25, "t1": 48, "delta": "lidar_coverage", "nota": "encara el cristal"},
                {"t0": 48, "t1": 999,"delta": "motion",         "nota": "ya paso"}]}

'delta' es lo prescrito: un rol, o una salida gobernada (review/defer/no_use). Para los CONTROLES
NO RESUELTOS se pone "review" o "defer": ahi responder con confianza es FALLO, no casi-acierto.

Uso:  python3 analysis/dcc_score.py guion.json [guion2.json ...]
"""
import json
import os
import sys
import collections

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import dcc_roles as R                                            # noqa: E402

CONDS = ("C1", "C2", "C3", "C4")
# PRIMARIO: A_meta = 1[Z = delta], identidad estricta, tal como lo define el protocolo.
# C1 y C3 verifican al titular y su espacio es retain/reject/unresolved, asi que cuando lo
# prescrito es OTRO rol NO pueden acertar. Eso no es un apano del puntuador: ES EL HALLAZGO --
# la linea base temporal no reconstruye roles alternativos, y el contraste C4-C3 lo mide.
# Un primer intento dio credito a C3 por responder 'unresolved' ante un rol prescrito, y eso
# igualaba C3 y C4 por construccion: C4-C3 salia -0.4 pp, borrando el contraste principal.
def acierta(cond, prescrito, salida):
    if prescrito == "motion" and cond in ("C1", "C3"):
        return salida == "retain"          # el titular sigue gobernando
    return salida == prescrito


# SECUNDARIO, y va aparte porque el protocolo lo pide aparte: si el verificador del titular
# emitio un juicio RESPONSABLE en su propio espacio. Ante un rol alternativo prescrito, lo
# responsable es 'unresolved' -- no verificar en falso. Esto NO entra en A_meta.
def responsable(cond, prescrito, salida):
    if cond not in ("C1", "C3"):
        return None
    if prescrito == "motion":
        return salida == "retain"
    return salida == "unresolved"


def carga_run(nombre):
    for f in (os.path.join("dataset", nombre + ".json"), nombre):
        if os.path.exists(f):
            return json.load(open(f))
    raise SystemExit("no encuentro la grabacion: %s" % nombre)


def prescrito_en(tramos, t):
    for tr in tramos:
        if tr["t0"] <= t < tr["t1"]:
            return tr["delta"], tr.get("nota", "")
    return None, ""


def puntua(guion):
    d = carga_run(guion["run"])
    ss = d.get("samples") or []
    ac = {c: [0, 0] for c in CONDS}                  # [aciertos, total]
    sec = {c: [0, 0] for c in ("C1", "C3")}          # secundario: juicio responsable
    detalle = collections.defaultdict(lambda: collections.Counter())
    for m in ss:
        pres, _ = prescrito_en(guion["tramos"], m.get("t", -1))
        if pres is None:
            continue
        for c in CONDS:
            r = R.condicion(m, c)
            ok = acierta(c, pres, r["out"])
            ac[c][0] += ok
            ac[c][1] += 1
            if not ok:
                detalle[c]["%s->%s" % (pres, r["out"])] += 1
            rp = responsable(c, pres, r["out"])
            if rp is not None:
                sec[c][0] += rp
                sec[c][1] += 1
    return ac, detalle, len(ss), sec


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    tot = {c: [0, 0] for c in CONDS}
    for g in sys.argv[1:]:
        guion = json.load(open(g))
        ac, det, n, sec = puntua(guion)
        print("\n=== %s  (%s, %d muestras) ===" % (guion.get("episodio", "?"), guion["run"], n))
        for c in CONDS:
            a, t = ac[c]
            tot[c][0] += a
            tot[c][1] += t
            iface = "I0" if c in ("C1", "C2") else "I1"
            modo = "titular" if c in ("C1", "C3") else "distribuida"
            print("  %s (%s, %-11s) %4d/%-4d = %5.1f%%   %s" % (
                c, iface, modo, a, t, 100.0 * a / t if t else 0,
                ", ".join("%s x%d" % (k, v) for k, v in det[c].most_common(2))))
        for c in ("C1", "C3"):
            a, t = sec[c]
            if t:
                print("     secundario %s -- juicio responsable del verificador: %5.1f%% (%d/%d)"
                      % (c, 100.0 * a / t, a, t))
    print("\n=== CONJUNTO ===")
    pct = {}
    for c in CONDS:
        a, t = tot[c]
        pct[c] = 100.0 * a / t if t else 0.0
        print("  %s: %5.1f%%  (%d/%d)" % (c, pct[c], a, t))
    print("\n=== CONTRASTES (los de Renxi) ===")
    print("  C4-C3 (DCC con la misma informacion revisada) : %+6.1f pp" % (pct["C4"] - pct["C3"]))
    print("  C4-C2 (la interfaz revisada, con resolucion)  : %+6.1f pp" % (pct["C4"] - pct["C2"]))
    print("  C3-C1 (la interfaz, para el verificador)      : %+6.1f pp" % (pct["C3"] - pct["C1"]))
    print("  C4-C1 (el cambio de paradigma completo)       : %+6.1f pp" % (pct["C4"] - pct["C1"]))
    print("\n  OJO: son proporciones por MUESTRA. La unidad independiente del protocolo es la")
    print("  configuracion o el run, asi que para inferencia hay que agregar por run y agrupar")
    print("  por trazado -- esto describe, no infiere.")


if __name__ == "__main__":
    main()
