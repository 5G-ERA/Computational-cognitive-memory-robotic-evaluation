#!/usr/bin/env python3
"""summit_run_core.py — el registro de una run del Summit XL con EL MISMO esquema que el G1.

Sin ROS a proposito: todo lo que decide que se escribe y como se cuenta vive aqui y se
prueba en frio (prueba_core.py). El nodo de ROS (summit_run_logger.py) solo le da de comer.

Que se copia del G1 (g1_goto.py, clase de registro + _append_stats) y que no:

  IGUAL    el JSON por run  {schema, mode, label, env, goal, started, samples, events, summary}
           la muestra        {t, x, y, yaw, d, spd, c0, nobs, phase, cmd} + extras
           runs_stats.csv    mismas columnas, mismo orden, mismas formulas (COLS_G1)
           clearance / progression / reliability / laser_noise / loc_conf: g1_metrics.py VERBATIM
           time_s, path_m, straight_m, efficiency = straight/path, c0 = cono de +-25 grados desde el centro
           reloc_jump = >0.5 m entre dos ticks; atasco = >8 s sin acercarse 0.10 m

  DISTINTO, y dicho en la cabecera de cada JSON (campo "defs"):
           la COLISION. En el G1 es "mandaba avanzar y la POSE no se movio 5 cm en 0.9 s".
           Eso en el Summit no puede funcionar: su odometria es open_loop (sale de los comandos,
           no de los encoders), asi que la pose avanza aunque el robot este clavado contra algo.
           Y a 0.05 m/s -la velocidad de cruce- 0.9 s son 4.5 cm: el umbral fijo del G1 contaria
           cada cruce lento como un choque. Aqui se mide contra las RUEDAS (joint_states, que si
           es hardware) y relativo a lo mandado. Ver ContactDetector.

  NO HAY   bateria (el BMS no esta cableado: lee 0/nan), percepcion por camara, spills.
           Esas columnas quedan en blanco, que es lo que hace el G1 con lo que no tiene.

Igual que en el G1: collisions=0 NO significa limpio. Un roce que no frena las ruedas no se ve.
"""
from __future__ import annotations
import csv, hashlib, json, math, os, statistics, time
from collections import deque

# Columnas de dataset/runs_stats.csv del G1, en su orden. prueba_core.py las contrasta contra
# el fuente de g1_goto.py: si el G1 anade una columna, la prueba falla y hay que traerla aqui.
COLS_G1 = [
    "run", "started", "mode", "goal", "result", "time_s", "path_m", "efficiency", "collisions",
    "stuck_episodes", "stuck_time_s", "astar_fails", "aggressive_on", "reloc_jumps",
    "pct_dwa", "pct_door", "pct_brk", "pct_seek", "pct_recovery", "pct_stop",
    "spd_mean", "c0_mean", "c0_min", "c0_hard_mean", "c0_hard_min", "near_wall_ticks",
    "obs_mean", "obs_max", "obs_per_m2", "n_hard_mean", "colmap_cells",
    "perc_n_mean", "color_pts_mean", "carpet_pct_mean", "color_near_mean", "dets_by_label",
    "laser_noise_mean", "filt_rej_mean", "scan_hz", "stale_pct", "gated_pct", "safer_inserts",
    "map_adds", "map_dels", "tick_ms_p95",
    "clearance_mean", "clearance_min", "clearance_max",
    "progression_mean", "progression_min", "progression_max",
    "reliability_mean", "reliability_min", "reliability_max",
    "loc_conf_mean", "loc_conf_min", "loc_conf_max", "laser_noise_min", "laser_noise_max",
    "spd_max", "c0_max", "c0_hard_max", "perc_n_min", "perc_n_max", "color_pts_max",
    "carpet_pct_min", "carpet_pct_max", "bat_start", "bat_end", "bat_used", "bat_min",
]
# Lo que el Summit sabe y el G1 no. Van DESPUES, para que las columnas comunes casen por posicion.
COLS_SUMMIT = [
    "robot", "arm", "straight_m", "spd_cmd_mean", "spd_cmd_max", "c0_body_min",
    "collision_ahead", "teb_infeasible", "patience_exceeded", "recoveries", "scan_drops",
    "drive_stalls", "amcl_jumps", "amcl_jump_max_m", "collisions_human", "valid", "notes",
]

MEDIO_LARGO = 0.361           # del centro al frente de la huella: c0_body = c0 - MEDIO_LARGO
OCELL = 0.2                   # la misma celda de obstaculos que el G1, para que nobs y loc_match casen

DEFS = {
    "c0": "m del CENTRO del robot al punto del laser mas cercano en un cono de +-25 grados, tope 2.5 (= clear_dir del G1)",
    "c0_body": "c0 - 0.361: lo que queda hasta el frente de la huella. El G1 no lo tiene; su cuerpo es ~0.2 m",
    "collision": "NO es la del G1. wheel: mandado >=0.04 m en la ventana y las ruedas <25% de eso, con esfuerzo; "
                 "effort: ruedas <50% de lo mandado y esfuerzo >1.7x la mediana libre; human: marcada a mano. "
                 "Refractario 4 s y nunca durante una recuperacion de Nav2 (igual que el G1); ademas un contacto = una colision: "
                 "no se rearma hasta que las ruedas giran 0.5 s seguidos",
    "drive_stall": "ruedas paradas con mando y SIN esfuerzo: driver no habilitado, no un choque. No cuenta como colision",
    "collision_ahead": "rechazo del comprobador de colision de Nav2. Es una PREDICCION, no un contacto: no cuenta como colision",
    "reloc_jump": ">0.5 m entre dos ticks (= G1)",
    "amcl_jump": "discontinuidad de map->odom >= 0.04 m entre dos ticks. El G1 no lo tiene",
    "spd": "velocidad lineal medida en las ruedas (joint_states x radio); si no hay, derivada de la pose",
    "loc_conf": "fraccion de celdas del laser en vivo que caen sobre o junto a una celda ocupada del mapa (= match_score del G1)",
    "progression": "g1_metrics.SEIMetrics con prog_ref=0.30 m/s, que es la marcha del G1. El Summit cruza a 0.05: "
                   "compara progress_rate (m/s) si quieres unidades, progression si quieres la escala del G1",
}


def sha256_de(ruta):
    try:
        return hashlib.sha256(open(ruta, "rb").read()).hexdigest()[:16]
    except OSError:
        return None


def clear_dir(pts_base, off_deg=0.0, maxd=2.5, cone=25.0):
    """c0: igual que clear_dir() del G1, con los puntos ya en el marco del robot (x adelante)."""
    best = maxd
    for (px, py) in pts_base:
        d = math.hypot(px, py)
        if d < 0.05 or d >= best:
            continue
        ang = abs((math.degrees(math.atan2(py, px)) - off_deg + 180) % 360 - 180)
        if ang < cone:
            best = d
    return best


def match_score(live_cells, ref_cells):
    """= match_score() del G1."""
    if not ref_cells or not live_cells:
        return None
    hit = 0
    for (cx, cy) in live_cells:
        if any((cx + dx, cy + dy) in ref_cells for dx in (-1, 0, 1) for dy in (-1, 0, 1)):
            hit += 1
    return round(hit / len(live_cells), 3)


def celdas(pts_mapa, cell=OCELL):
    return {(round(x / cell), round(y / cell)) for (x, y) in pts_mapa}


class ContactDetector:
    """Colision del Summit. Ver el docstring del modulo para el porque de cada diferencia con el G1."""

    def __init__(self, win=2.0, refractario=4.0):
        self.win = win; self.refractario = refractario
        self.h = deque()                     # (t, v_cmd, v_rueda, esfuerzo)
        self.base = deque(maxlen=40)         # esfuerzo rodando libre, como lt_base del G1
        self.last_t = -99.0
        self.armado = True                   # un CONTACTO = una colision: no se vuelve a contar hasta que las ruedas giren otra vez
        self.libre_desde = None

    def update(self, now, v_cmd, v_rueda, esfuerzo, en_recuperacion=False):
        """Devuelve None, o (kind, src, extra) con kind en {"collision", "drive_stall"}."""
        if v_rueda is None:
            return None                      # sin ruedas no hay detector: se dice en la cabecera, no se inventa
        self.h.append((now, abs(v_cmd), abs(v_rueda), esfuerzo))
        while self.h and now - self.h[0][0] > self.win:
            self.h.popleft()
        if abs(v_cmd) > 0.02 and abs(v_rueda) > 0.5 * abs(v_cmd) and esfuerzo is not None:
            self.base.append(esfuerzo)
        # rearme: medio segundo seguido girando de verdad. Sin esto un robot clavado 16 s contaba 4 choques
        # (banco, 19-sep); el G1 no lo necesita porque tras chocar retrocede, y este registro es pasivo.
        if abs(v_cmd) > 0.02 and abs(v_rueda) > 0.5 * abs(v_cmd):
            self.libre_desde = self.libre_desde if self.libre_desde is not None else now
            if now - self.libre_desde >= 0.5:
                self.armado = True
        else:
            self.libre_desde = None
        if not self.armado or en_recuperacion or now - self.last_t <= self.refractario or len(self.h) < 5:
            return None

        def integra(desde, k):
            tot = 0.0; prev = None
            for r in self.h:
                if r[0] < desde:
                    prev = r; continue
                if prev is not None:
                    tot += 0.5 * (r[k] + prev[k]) * (r[0] - prev[0])
                prev = r
            return tot

        bl = statistics.median(self.base) if len(self.base) >= 5 else None
        span = now - self.h[0][0]
        extra = {"v_cmd": round(v_cmd, 3), "v_rueda": round(v_rueda, 3),
                 "esfuerzo": None if esfuerzo is None else round(esfuerzo, 1),
                 "esfuerzo_base": None if bl is None else round(bl, 1)}
        # --- "wheel": el analogo del "odom" del G1 (ventana >=0.9 s, >=8 muestras) ---
        if span >= 0.9 and len(self.h) >= 8:
            esperado = integra(now - span, 1); medido = integra(now - span, 2)
            if esperado >= 0.04 and medido < 0.25 * esperado:
                self.last_t = now; self.h.clear(); self.armado = False; self.libre_desde = None
                extra.update({"esperado_m": round(esperado, 3), "medido_m": round(medido, 3)})
                if bl is not None and esfuerzo is not None and esfuerzo < 0.2 * bl:
                    return ("drive_stall", "wheel", extra)      # paradas y sin empujar: el driver, no un choque
                return ("collision", "wheel", extra)
        # --- "effort": el analogo del "imu" del G1 (ventana >=0.5 s, pico sobre la mediana libre) ---
        if span >= 0.5 and bl is not None and esfuerzo is not None:
            esperado = integra(now - 0.5, 1); medido = integra(now - 0.5, 2)
            if esperado >= 0.02 and medido < 0.5 * esperado and esfuerzo > 1.7 * bl:
                self.last_t = now; self.h.clear(); self.armado = False; self.libre_desde = None
                extra.update({"esperado_m": round(esperado, 3), "medido_m": round(medido, 3)})
                return ("collision", "effort", extra)
        return None


# Literales de Nav2 que el 19-sep resultaron ser el diagnostico entero. patron -> kind del evento.
ROSOUT = [
    ("Collision Ahead", "collision_ahead"),
    ("trajectory is not feasible", "teb_infeasible"),
    ("Controller patience exceeded", "patience_exceeded"),
    ("Running backup", "recovery"), ("Running spin", "recovery"), ("Running wait", "recovery"),
    ("Running drive_on_heading", "drive_on_heading"),
    ("Message Filter dropping message", "scan_drop"),
    ("Begin navigating", "nav_begin"), ("Goal succeeded", "nav_ok"),
    ("Aborting handle", "nav_abort"),
]


def clasifica_rosout(texto):
    for pat, kind in ROSOUT:
        if pat in texto:
            return kind
    return None


class RunRecorder:
    """= la clase de registro de g1_goto.py: sample(), event(), finish()."""

    def __init__(self, dataset_dir, mode, label, goal, env="real", arm=None, cabecera=None):
        os.makedirs(dataset_dir, exist_ok=True)
        self.dir = dataset_dir; self.mode = mode; self.t0 = time.time()
        self.fname = os.path.join(dataset_dir, time.strftime("%Y%m%d_%H%M%S") + f"_{mode}_{label}.json")
        self.rec = {"schema": "g1_goto_run/v1", "robot": "summit_xl", "mode": mode, "label": label,
                    "env": env, "sim_id": os.environ.get("SUMMIT_SIM_ID") or None,
                    "goal": {"x": goal[0], "y": goal[1]}, "OCELL": OCELL,
                    "started": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "arm": arm or os.environ.get("SUMMIT_ARM") or None,
                    "env_summit": {k: v for k, v in sorted(os.environ.items()) if k.startswith("SUMMIT_")},
                    "defs": DEFS, "samples": [], "events": [], "summary": {}}
        if cabecera:
            self.rec.update(cabecera)

    def sample(self, t, x, y, yaw, d, spd, c0, nobs, cmd=None, phase="", extra=None):
        rec = {"t": round(t, 2), "x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 1),
               "d": round(d, 3), "spd": round(spd, 3), "c0": round(c0, 2), "nobs": nobs, "phase": phase,
               "cmd": [round(float(v), 3) for v in cmd] if cmd else None}
        if extra:
            rec.update({k: v for k, v in extra.items() if v is not None})
        self.rec["samples"].append(rec)

    def event(self, kind, t, x, y, extra=None):
        ev = {"kind": kind, "t": round(t, 2), "x": round(x, 3), "y": round(y, 3)}
        if extra:
            ev.update(extra)
        self.rec["events"].append(ev)

    def finish(self, result, summary):
        self.rec["result"] = result
        self.rec["duration_s"] = round(time.time() - self.t0, 2)
        self.rec["summary"] = dict(summary or {})
        tmp = self.fname + ".tmp"
        json.dump(self.rec, open(tmp, "w")); os.replace(tmp, self.fname)
        fila = self.fila_stats()
        path = os.path.join(self.dir, "runs_stats.csv")
        nuevo = not os.path.exists(path)
        with open(path, "a", newline="") as fo:
            w = csv.DictWriter(fo, fieldnames=COLS_G1 + COLS_SUMMIT)
            if nuevo:
                w.writeheader()
            w.writerow(fila)
        return self.fname, path, fila

    def fila_stats(self):
        """= _append_stats() del G1, formula por formula."""
        s = self.rec["samples"]; ev = self.rec["events"]; sm = self.rec["summary"]

        def num(k):
            return [x.get(k) for x in s if isinstance(x.get(k), (int, float))]

        def _mean(k):
            v = num(k); return round(sum(v) / len(v), 3) if v else ""

        def _minv(k):
            v = num(k); return round(min(v), 3) if v else ""

        def _maxv(k):
            v = num(k); return round(max(v), 3) if v else ""

        def n_ev(kind):
            return sum(1 for e in ev if e.get("kind") == kind)

        # atasco: >8 s sin acercarse >=0.10 m al objetivo (identico al G1)
        stuck_n = 0; stuck_s = 0.0; best_d = None; last_imp = None; in_stuck = False; prev_t = None
        for x in s:
            t, dd = x.get("t", 0.0), x.get("d")
            if dd is None:
                continue
            if best_d is None or dd < best_d - 0.10:
                best_d = dd; last_imp = t; in_stuck = False
            elif last_imp is not None and (t - last_imp) > 8.0:
                if not in_stuck:
                    stuck_n += 1; in_stuck = True
            if in_stuck and prev_t is not None:
                stuck_s += max(0.0, t - prev_t)
            prev_t = t
        fam = {"DWA": 0, "DOOR": 0, "BRK": 0, "SEEK": 0, "R": 0, "STOP": 0, "OTRO": 0}
        for x in s:
            p = (x.get("phase") or "")
            k = ("DWA" if p.startswith("DWA") else "DOOR" if p.startswith("DOOR") else
                 "BRK" if p.startswith("BRK") else "SEEK" if p.startswith("SEEK") else
                 "R" if p.startswith("R-") else "STOP" if p.startswith("STOP") else "OTRO")
            fam[k] += 1
        nt = max(1, len(s))
        xs = [x["x"] for x in s]; ys = [x["y"] for x in s]
        area = max(1.0, (max(xs) - min(xs) + 1.0) * (max(ys) - min(ys) + 1.0)) if s else 1.0
        saltos = [e.get("dist", 0.0) for e in ev if e.get("kind") == "amcl_jump"]
        c0min = sm.get("c0min", _minv("c0"))
        fila = {k: "" for k in COLS_G1 + COLS_SUMMIT}
        fila.update({
            "run": os.path.basename(self.fname)[:-5], "started": self.rec.get("started", ""),
            "mode": self.mode, "goal": self.rec.get("label", ""), "result": self.rec.get("result", ""),
            "time_s": sm.get("time_s", self.rec.get("duration_s", "")),
            "path_m": sm.get("path_m", ""), "efficiency": sm.get("efficiency", ""),
            "collisions": n_ev("collision"),
            "stuck_episodes": stuck_n, "stuck_time_s": round(stuck_s, 1),
            "reloc_jumps": sm.get("reloc_jumps", n_ev("reloc_jump")),
            "pct_dwa": round(100.0 * fam["DWA"] / nt, 1), "pct_door": round(100.0 * fam["DOOR"] / nt, 1),
            "pct_brk": round(100.0 * fam["BRK"] / nt, 1), "pct_seek": round(100.0 * fam["SEEK"] / nt, 1),
            "pct_recovery": round(100.0 * fam["R"] / nt, 1), "pct_stop": round(100.0 * fam["STOP"] / nt, 1),
            "spd_mean": _mean("spd"), "c0_mean": _mean("c0"), "c0_min": c0min,
            "obs_mean": _mean("nobs"), "obs_max": sm.get("obs_max", _maxv("nobs")),
            "obs_per_m2": round((_mean("nobs") or 0) / area, 2) if s else "",
            "laser_noise_mean": sm.get("laser_noise_mean", _mean("laser_noise")),
            "scan_hz": sm.get("scan_hz", ""), "stale_pct": sm.get("stale_pct", ""),
            "tick_ms_p95": sm.get("tick_ms_p95", ""),
            "clearance_mean": _mean("clearance"), "clearance_min": _minv("clearance"),
            "clearance_max": _maxv("clearance"),
            "progression_mean": _mean("progression"), "progression_min": _minv("progression"),
            "progression_max": _maxv("progression"),
            "reliability_mean": _mean("reliability"), "reliability_min": _minv("reliability"),
            "reliability_max": _maxv("reliability"),
            "loc_conf_mean": _mean("loc_conf"), "loc_conf_min": _minv("loc_conf"), "loc_conf_max": _maxv("loc_conf"),
            "laser_noise_min": _minv("laser_noise"),
            "laser_noise_max": sm.get("laser_noise_max", _maxv("laser_noise")),
            "spd_max": _maxv("spd"), "c0_max": _maxv("c0"),
            # --- solo Summit ---
            "robot": "summit_xl", "arm": self.rec.get("arm") or "",
            "straight_m": sm.get("straight_m", ""),
            "spd_cmd_mean": _mean("spd_cmd"), "spd_cmd_max": _maxv("spd_cmd"),
            "c0_body_min": round(c0min - MEDIO_LARGO, 3) if isinstance(c0min, (int, float)) else "",
            "collision_ahead": n_ev("collision_ahead"), "teb_infeasible": n_ev("teb_infeasible"),
            "patience_exceeded": n_ev("patience_exceeded"), "recoveries": n_ev("recovery"),
            "scan_drops": n_ev("scan_drop"), "drive_stalls": n_ev("drive_stall"),
            "amcl_jumps": len(saltos), "amcl_jump_max_m": round(max(saltos), 3) if saltos else "",
            "collisions_human": sum(1 for e in ev if e.get("kind") == "collision" and e.get("src") == "human"),
            "valid": sm.get("valid", ""), "notes": sm.get("notes", ""),
        })
        return fila


def path_len(trail):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(trail, trail[1:]))
