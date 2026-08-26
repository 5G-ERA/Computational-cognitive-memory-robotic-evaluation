# -*- coding: utf-8 -*-
"""Certificados Omega_t y el segundo desenlace primario, A_Omega.

    Omega_t(y_t) = <delta_t(y_t), kappa_t(y_t)>
    A_Omega = 1[ la arquitectura reconstruye la resolucion gobernada CON SUS FUNDAMENTOS ]

LA DISTINCION QUE ESTE MODULO ENCARNA: A_meta pregunta si Z_t coincide con delta_t.
A_Omega pregunta mas -- si la resolucion se sostiene EN EL FUNDAMENTO CORRECTO. Acertar
"illumination" citando la bateria es A_meta=1, A_Omega=0: la resolucion es correcta por
accidente, no reconstruida desde los fundamentos que la interfaz preserva. Por eso A_Omega
es el desenlace que mide la utilidad de la INTERFAZ (C4-C2), y por eso no puede computarse
sin certificados.

DE DONDE SALE CADA MITAD:
  - El certificado de REFERENCIA lo escribe guion.py EN EL ACTO de escenificar (§3: la
    referencia viene del guion, no de la telemetria del sistema bajo prueba). Cada segmento
    declara delta_t, el fundamento requerido y el estado del mundo que lo justifica.
  - La RECONSTRUCCION sale de la run grabada: cada condicion C1-C4 se evalua sobre la
    muestra y su `razon` lleva el fundamento que cito (los prefijos de dcc_roles:
    contrato:/cobertura:/bat=/objeto:/conjunta:/defecto:).

El reloj: los certificados van en tiempo de pared (epoch); la run lleva `started` con
resolucion de 1 s y muestras en tiempo relativo. El mapeo t_run = t_pared - epoch(started)
hereda esa resolucion de +-1 s, que la ventana de gracia de +-2 s en cada frontera absorbe.
Esta declarado aqui para que nadie lo descubra como sorpresa.

GRUESO DECLARADO DE LA v1: para review/defer del verificador temporal se acepta el
fundamento "no verificable" sin exigir que cite la senal de calidad concreta. Refinarlo
exigiria etiquetar que subtipo de insuficiencia declara cada guion; anotado como mejora,
no como hecho.
"""
import json, math, os, time, calendar

# envolvente de identificabilidad del objeto: la MISMA que el canal de percepcion real
# (camara fx=600, W=640 -> medio campo 28.07 grados; alcance de la curva calibrada 4.5 m).
# delta_t = object solo donde el objeto declarado es identificable; fuera de la envolvente
# la evidencia correcta es motion aunque el objeto SIGA en la sala. Es la clausula
# "becomes identifiable" de la tabla, hecha computable a priori desde la posicion declarada
# y la pose -- el mismo movimiento que la pre-registracion hace con la region ciega (§4.2).
# La pose viene del propio robot: limitacion ya declarada en §3, no nueva.
ENV_HFOV = 28.07
ENV_RMAX = 4.5

# que fundamento exige cada delta_t (prefijo de `razon` antes de los dos puntos)
FUNDAMENTO = {
    "motion":        ("defecto", "verificable"),
    "illumination":  ("contrato",),
    "lidar_quality": ("cobertura",),
    "energy":        ("bat",),
    "object":        ("objeto",),
    "review":        ("conjunta", "no verificable"),
    "defer":         ("conjunta", "no verificable"),
    "no_use":        ("assist", "fault"),
}


def epoch_de(started):
    """'2026-08-24 12:00:15' (hora local del equipo) -> epoch. Resolucion 1 s, declarada."""
    return time.mktime(time.strptime(started, "%Y-%m-%d %H:%M:%S"))


def carga_referencia(fichero_ref, run):
    """Certificado de referencia mapeado al tiempo de LA run: [(t_desde, t_hasta, delta, estado)]."""
    ref = json.load(open(fichero_ref))
    t0 = epoch_de(run["started"])
    fin = (run.get("samples") or [{"t": 0}])[-1]["t"]
    segs = []
    for i, s in enumerate(ref["segmentos"]):
        t_a = max(0.0, s["t_pared"] - t0)
        t_b = (ref["segmentos"][i + 1]["t_pared"] - t0) if i + 1 < len(ref["segmentos"]) else fin
        segs.append({"desde": t_a, "hasta": t_b, "delta": s["delta"], "estado": s["estado"]})
    return segs


def _cristal_en_ventana(est, m, d_max=2.5):   # d_max = COV_R declarado del instrumento
    """El cristal declarado esta en la VENTANA DEL INSTRUMENTO de la muestra: distancia
    <= d_max al rect Y rumbo al punto mas cercano dentro del sector de cobertura declarado
    (+-40 deg, el COV_SECT del robot). Anadido 25-ago: la zona por distancia sola sobre-exigia
    -- en zona pero sin encarar la pared, la decepcion es INDETECTABLE por diseno del
    instrumento, y el resolutor con evidencia completa contestaba (con razon) el rol simple;
    medido en T8: C4 respondia illumination en 152/176 muestras delta=defer y las abladas
    C2/C3 puntuaban MAS que C4 solo porque su ignorancia les fuerza el rol conjunto.
    Computable a priori (estado declarado + pose + geometria declarada): no depende de la
    condicion, asi que no viola la simetria de la referencia (SS4)."""
    rects = [r for r in str(est.get("cristal") or "").split(";") if r.strip()]
    mx, my = m.get("x", 0), m.get("y", 0)
    for r_ in rects:
        try:
            x0, y0, x1, y1 = (float(t_) for t_ in r_.split(","))
        except ValueError:
            continue
        cx = min(max(mx, min(x0, x1)), max(x0, x1))
        cy = min(max(my, min(y0, y1)), max(y0, y1))
        if math.hypot(mx - cx, my - cy) > d_max:
            continue
        brg = (math.degrees(math.atan2(cy - my, cx - mx)) - m.get("yaw", 0) + 540.0) % 360.0 - 180.0
        if abs(brg) <= 40.0:
            return True
    return False


def delta_muestra(seg, m):
    """delta_t derivada para UNA muestra: tupla de roles aceptables.

    Es la funcion central del certificado: delta no es la etiqueta del segmento sino
    f(estado declarado, pose). La usan el puntuador primario (A_meta/A_Omega) y el de
    desenlaces secundarios (retardo de conmutacion, retorno, persistencia falsa) -- una
    sola definicion para que no puedan divergir.
    """
    delta = seg["delta"]
    est = seg.get("estado") or {}
    if delta in ("defer", "review"):
        luz_mala = float(est.get("luz", 0)) > 99.0
        en_zona = _cristal_en_ventana(est, m)
        if en_zona and luz_mala:
            return ("defer", "review")
        if luz_mala:
            return ("illumination",)
        if en_zona:
            return ("lidar_quality",)
        return ("motion",)
    if delta == "lidar_quality":
        # mismo patron que defer: exigible solo donde el cristal declarado afecta a la
        # cobertura del robot (T1 sufria lo de T8: certificado global, fenomeno local),
        # incluida la condicion de ENCARAMIENTO (ver _cristal_en_ventana)
        if _cristal_en_ventana(est, m):
            return ("lidar_quality",)
        return ("motion",)
    if delta == "no_use":
        # T10: el bloqueo esta declarado desde el arranque, pero no_use solo es exigible
        # tras el DESCUBRIMIENTO -- computable a priori: el robot ha pasado >=8 s dentro de
        # la zona de observacion del vano (2.0 m). Antes de descubrir: la silla identificable
        # pide object; si no, motion. El estado de descubrimiento se acumula en la propia
        # muestra via _t10_visto (el puntuador recorre en orden temporal).
        bq = est.get("bloqueo")
        if bq:
            d_v = math.hypot(m.get("x", 0) - float(bq[0]), m.get("y", 0) - float(bq[1]))
            seg.setdefault("_t10_dentro", 0.0)
            if d_v <= 2.0:
                if seg.get("_t10_t0") is None:
                    seg["_t10_t0"] = m.get("t", 0)
                seg["_t10_dentro"] = m.get("t", 0) - seg["_t10_t0"]
            if seg["_t10_dentro"] >= 8.0:
                return ("no_use", "defer")     # abstencion gobernada tambien admisible tras descubrir
        objs = est.get("objetos") or []
        for o in objs:
            dx, dy = float(o[1]) - m.get("x", 0), float(o[2]) - m.get("y", 0)
            r = math.hypot(dx, dy)
            brg = (math.degrees(math.atan2(dy, dx)) - m.get("yaw", 0) + 540) % 360 - 180
            if 0.2 < r <= ENV_RMAX and abs(brg) <= ENV_HFOV:
                return ("object",)
        return ("motion",)
    if delta == "object":
        objs = est.get("objetos") or []
        for o in objs:
            dx, dy = float(o[1]) - m.get("x", 0), float(o[2]) - m.get("y", 0)
            r = math.hypot(dx, dy)
            brg = (math.degrees(math.atan2(dy, dx)) - m.get("yaw", 0) + 540) % 360 - 180
            if 0.2 < r <= ENV_RMAX and abs(brg) <= ENV_HFOV:
                return ("object",)
        return ("motion",)
    return (delta,)


def fundamento_de(razon):
    r = (razon or "").strip()
    if r.startswith("no verificable"):
        return "no verificable"
    if r.startswith("verificable"):
        return "verificable"
    return r.split(":", 1)[0].split("=", 1)[0].strip()


def puntua_run(run, segs, evalua_todas, usa_pose, ventana=2.0):
    """A_meta y A_Omega por condicion sobre una run, contra su certificado de referencia."""
    out = {}
    for m in run.get("samples") or []:
        t = m.get("t")
        if t is None:
            continue
        seg = next((s for s in segs if s["desde"] <= t < s["hasta"]), None)
        if seg is None:
            continue
        # ventana de gracia alrededor de CADA frontera (retardo instrumental declarado)
        if any(abs(t - s["desde"]) < ventana for s in segs[1:]):
            continue
        deltas_ok = delta_muestra(seg, m)
        acepta = tuple(f for d_ in deltas_ok for f in FUNDAMENTO.get(d_, ()))
        r = evalua_todas(m, usa_pose=usa_pose)
        for c, res in r.items():
            a = out.setdefault(c, {"meta": 0, "omega": 0, "n": 0})
            a["n"] += 1
            ok_meta = res["Z"] in deltas_ok
            if ok_meta:
                a["meta"] += 1
                if fundamento_de(res["razon"]) in acepta:
                    a["omega"] += 1
    return out
