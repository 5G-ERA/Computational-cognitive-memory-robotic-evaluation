#!/usr/bin/env python3
"""summit_run_logger.py — registra una run del Summit XL con el esquema del G1. PASIVO.

No publica nada que mueva el robot: solo escucha. Se lanza antes de la travesia y se para
con Ctrl+C (o SIGTERM, o solo al llegar con --auto); al parar escribe

    <dataset>/<fecha>_<mode>_<label>.json     schema g1_goto_run/v1, el mismo del G1
    <dataset>/runs_stats.csv                  una fila mas, mismas columnas que el G1

Uso, en el robot:
    python3 summit_run_logger.py --label B --goal 0.362 -6.076 --arm C1-LASER --auto
    python3 summit_run_logger.py --label vano --marca vano_antes        # lee ~/ab/marcas.txt

Marcar a mano desde otra terminal (Natan ve un roce que las ruedas no notan):
    ros2 topic pub --once /robot/run/colision std_msgs/msg/Empty
    ros2 topic pub --once /robot/run/fase std_msgs/msg/String "{data: DOOR-LASER}"
    ros2 topic pub --once /robot/run/nota std_msgs/msg/String "{data: 'la puerta estaba entornada'}"
"""
import argparse, math, os, signal, statistics, subprocess, sys, threading, time

import numpy as np
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import Log
from sensor_msgs.msg import LaserScan, JointState
from std_msgs.msg import Empty, String
import tf2_ros

AQUI = os.path.dirname(os.path.abspath(__file__))
# g1_metrics.py NO se duplica en el repo: es src/g1_metrics.py, el mismo fichero que usa el G1.
# En el robot se despliega una copia junto a este script (despliega.sh); gana la que este al lado.
for _d in (AQUI, os.path.join(AQUI, "..", "..", "src")):
    if os.path.exists(os.path.join(_d, "g1_metrics.py")):
        sys.path.insert(0, os.path.abspath(_d)); break
sys.path.insert(0, AQUI)
import summit_run_core as core
from g1_metrics import SEIMetrics, SensingMonitor

LASER_X = -0.01924            # el mismo que cruza_vano.py y el banco 2D
RADIO_RUEDA = 0.1114


def c0_np(pts, maxd=2.5, cone=25.0):
    """= core.clear_dir(), vectorizado: el robot ya pierde su ritmo de control por carga y esto corre a 10 Hz."""
    d = np.hypot(pts[:, 0], pts[:, 1])
    ok = (d >= 0.05) & (np.abs(np.degrees(np.arctan2(pts[:, 1], pts[:, 0]))) < cone)
    return float(min(maxd, d[ok].min())) if ok.any() else maxd


def yaw_de(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Registro(Node):
    def __init__(self, a):
        super().__init__("summit_run_logger")
        self.a = a; ns = a.ns.rstrip("/")
        be = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        lat = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.buf = tf2_ros.Buffer(); self.tfl = tf2_ros.TransformListener(self.buf, self)
        self.pts = None; self.t_scan = 0.0; self.n_scan = 0
        self.cmd = {}                          # topic -> (t, vx, wz): QUIEN manda dice en que fase estamos
        self.v_rueda = None; self.esfuerzo = None; self.t_js = 0.0
        self.amcl = None; self.ref = None; self.fase_ext = None; self.t_recov = -99.0; self.t_doh = -99.0
        self.pend = []                         # eventos que llegan entre ticks: (kind, extra)
        self.oido = set()
        self.create_subscription(LaserScan, ns + a.scan, self.on_scan, be)
        self.create_subscription(JointState, ns + a.joints, self.on_js, be)
        self.create_subscription(PoseWithCovarianceStamped, ns + "/amcl_pose", self.on_amcl, 10)
        self.create_subscription(OccupancyGrid, ns + a.mapa, self.on_mapa, lat)
        for tp in a.cmd + [a.cmd_efectivo]:
            self.create_subscription(Twist, ns + tp, lambda m, tp=tp: self.on_cmd(tp, m), 10)
        self.create_subscription(Log, "/rosout", self.on_rosout, 50)
        self.create_subscription(Empty, ns + "/run/colision", lambda m: self.pend.append(("collision", {"src": "human"})), 10)
        self.create_subscription(String, ns + "/run/fase", lambda m: setattr(self, "fase_ext", m.data.strip() or None), 10)
        self.create_subscription(String, ns + "/run/nota", lambda m: self.notas.append(m.data), 10)
        self.notas = []

        aqui = os.path.dirname(os.path.abspath(__file__))
        self.rd = core.RunRecorder(a.dataset, a.mode, a.label, tuple(a.goal), env=a.env, arm=a.arm, cabecera={
            "frames": {"map": a.frame_map, "odom": a.frame_odom, "base": a.frame_base},
            "topics": {"scan": ns + a.scan, "joints": ns + a.joints, "map": ns + a.mapa, "cmd_efectivo": ns + a.cmd_efectivo, "cmd_fuentes": [ns + t for t in a.cmd]},
            "metrics": {"clear_full": 1.5, "prog_ref": 0.30, "g1_metrics_sha": core.sha256_de(sys.modules["g1_metrics"].__file__)},
            "logger_sha": {f: core.sha256_de(os.path.join(aqui, f)) for f in ("summit_run_logger.py", "summit_run_core.py")},
            "goal_tol": a.tol, "params": {}})
        if not a.sin_volcado:
            threading.Thread(target=self.volcado, daemon=True).start()
        self.sei = SEIMetrics(); self.sens = SensingMonitor(); self.det = core.ContactDetector()
        self.t0 = time.time(); self.trail = []; self.last = None; self.last_mo = None
        self.minc0 = 9.0; self.njumps = 0; self.obs_max = 0; self.ticks = []; self.stale = 0; self.nt = 0
        self.t_quieto = None; self.fin = None; self.t_tick = None; self.sin_pose = 0
        self.create_timer(0.1, self.tick)
        self.get_logger().info("registrando -> %s   (Ctrl+C para cerrar la run)" % self.rd.fname)

    # ---------------------------------------------------------------- entradas
    def on_scan(self, m):
        r = np.asarray(m.ranges, dtype=np.float64); th = m.angle_min + m.angle_increment * np.arange(len(r))
        ok = np.isfinite(r) & (r > m.range_min) & (r < min(m.range_max, 10.0))
        self.pts = np.stack([r[ok] * np.cos(th[ok]) + LASER_X, r[ok] * np.sin(th[ok])], axis=1)
        self.t_scan = time.time(); self.n_scan += 1; self.oido.add("scan")

    def on_js(self, m):
        idx = [i for i, n in enumerate(m.name) if "wheel" in n]
        if not idx:
            return
        if len(m.velocity) >= len(m.name):
            self.v_rueda = RADIO_RUEDA * sum(abs(m.velocity[i]) for i in idx) / len(idx)
        if len(m.effort) >= len(m.name):
            self.esfuerzo = sum(abs(m.effort[i]) for i in idx) / len(idx)
        self.t_js = time.time(); self.oido.add("joints")

    def on_amcl(self, m):
        p = m.pose.pose; self.amcl = (p.position.x, p.position.y, yaw_de(p.orientation)); self.oido.add("amcl")

    def on_mapa(self, m):
        d = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        ys, xs = np.nonzero(d >= 65); res = m.info.resolution; o = m.info.origin.position
        self.ref = core.celdas(zip(o.x + (xs + 0.5) * res, o.y + (ys + 0.5) * res)); self.oido.add("map")

    def on_cmd(self, tp, m):
        self.cmd[tp] = (time.time(), m.linear.x, m.angular.z); self.oido.add("cmd:" + tp)

    def on_rosout(self, m):
        kind = core.clasifica_rosout(m.msg)
        if not kind:
            return
        if kind == "recovery":
            self.t_recov = time.time()
        if kind == "drive_on_heading":
            self.t_doh = time.time()
        self.pend.append((kind, {"nodo": m.name, "msg": m.msg[:120]}))

    def volcado(self):
        """Parametros EFECTIVOS en la cabecera: 'la tabla del paper no puede depender de memoria humana'."""
        ns = self.a.ns.rstrip("/")
        for nodo in ("behavior_server", "local_costmap/local_costmap", "controller_server", "amcl"):
            try:
                r = subprocess.run(["ros2", "param", "dump", ns + "/" + nodo], capture_output=True, text=True, timeout=12)
                if r.returncode == 0 and r.stdout.strip():
                    self.rd.rec["params"][nodo] = r.stdout
            except Exception as e:
                self.rd.rec["params"][nodo] = "no se pudo: %s" % e

    # ---------------------------------------------------------------- el tick
    def pose(self):
        try:
            t = self.buf.lookup_transform(self.a.frame_map, self.a.frame_base, Time()).transform
            mo = self.buf.lookup_transform(self.a.frame_map, self.a.frame_odom, Time()).transform.translation
            return (t.translation.x, t.translation.y, yaw_de(t.rotation)), (mo.x, mo.y), "tf"
        except Exception:
            return (self.amcl, None, "amcl") if self.amcl else (None, None, None)

    def fase(self, now, vx, wz, quien):
        if now - self.t_recov < 1.5:
            return "R-nav2"
        if self.fase_ext:
            return self.fase_ext
        if abs(vx) < 0.005 and abs(wz) < 0.01:
            return "STOP"
        if now - self.t_doh < 60.0 and quien != self.a.cmd_externo:
            return "DOOR-DOH"
        return "DOOR-LASER" if quien == self.a.cmd_externo else "DWA-TEB"

    def tick(self):
        now = time.time(); t = now - self.t0
        if self.t_tick is not None:
            self.ticks.append(now - self.t_tick)
        self.t_tick = now
        p, mo, src = self.pose()
        if p is None:
            self.sin_pose += 1
            if self.sin_pose % 50 == 1:
                self.get_logger().warn("sin pose todavia (ni TF %s->%s ni amcl_pose)" % (self.a.frame_map, self.a.frame_base))
            return
        x, y, yaw = p; self.rd.rec.setdefault("pose_src", src)
        # quien manda AHORA: el comando mas reciente y no caducado de entre los topics escuchados
        vivos = {tp: v for tp, v in self.cmd.items() if now - v[0] < 0.5}
        # QUIEN manda: la fuente viva mas reciente que pide moverse. CUANTO: la orden efectiva si se oye.
        fuentes = {tp: v for tp, v in vivos.items() if tp != self.a.cmd_efectivo and (abs(v[1]) > 0.005 or abs(v[2]) > 0.01)}
        quien = max(fuentes, key=lambda k: fuentes[k][0]) if fuentes else None
        ef = vivos.get(self.a.cmd_efectivo)
        vx, wz = (ef[1], ef[2]) if ef else ((fuentes[quien][1], fuentes[quien][2]) if quien else (0.0, 0.0))
        self.rd.rec.setdefault("cmd_src", "efectivo" if ef else "fuente")
        # saltos: el del G1 (>0.5 m por tick) y el que al Summit le importa (map->odom, cm)
        jump = False
        if self.last is not None:
            jd = math.hypot(x - self.last[0], y - self.last[1])
            if jd > 0.5:
                jump = True; self.njumps += 1
                self.rd.event("reloc_jump", t, x, y, {"dist": round(jd, 2), "from": [round(self.last[0], 2), round(self.last[1], 2)]})
        if mo is not None and self.last_mo is not None:
            md = math.hypot(mo[0] - self.last_mo[0], mo[1] - self.last_mo[1])
            if md >= 0.04:
                self.rd.event("amcl_jump", t, x, y, {"dist": round(md, 3)})
        self.last = (x, y); self.last_mo = mo if mo is not None else self.last_mo
        if not self.trail or math.hypot(x - self.trail[-1][0], y - self.trail[-1][1]) >= 0.01:
            self.trail.append((x, y))
        # laser -> c0 (marco del robot) y celdas en vivo (marco del mapa, OCELL del G1)
        fresco = self.pts is not None and now - self.t_scan < 0.5
        self.nt += 1; self.stale += 0 if fresco else 1
        c0 = c0_np(self.pts) if self.pts is not None else 2.5
        live = set()
        if self.pts is not None:
            c, s = math.cos(yaw), math.sin(yaw)
            live = core.celdas(zip(x + c * self.pts[:, 0] - s * self.pts[:, 1], y + s * self.pts[:, 0] + c * self.pts[:, 1]))
        self.minc0 = min(self.minc0, c0); self.obs_max = max(self.obs_max, len(live))
        d = math.hypot(self.a.goal[0] - x, self.a.goal[1] - y)
        extra = dict(self.sei.update(t, d, c0))
        if fresco:                             # igual que el G1: en ticks con el laser viejo no se actualiza el ruido
            extra.update(self.sens.update(t, live, c0, core.match_score(live, self.ref), jump))
        js_ok = now - self.t_js < 0.5
        vr = self.v_rueda if js_ok else None
        spd = vr if vr is not None else (math.hypot(x - self._px, y - self._py) / 0.1 if hasattr(self, "_px") else 0.0)
        self._px, self._py = x, y
        extra.update({"spd_cmd": round(abs(vx), 3), "c0_body": round(c0 - core.MEDIO_LARGO, 3),
                      "esfuerzo": None if not js_ok or self.esfuerzo is None else round(self.esfuerzo, 1),
                      "quien": quien, "scan_age": round(now - self.t_scan, 2) if self.pts is not None else None})
        self.rd.sample(t, x, y, math.degrees(yaw), d, spd, c0, len(live), cmd=(vx, wz),
                       phase=self.fase(now, vx, wz, quien), extra=extra)
        r = self.det.update(t, vx, vr, self.esfuerzo if js_ok else None, en_recuperacion=now - self.t_recov < 3.0)
        if r:
            self.pend.append((r[0], dict(r[2], src=r[1], c0=round(c0, 2))))
        while self.pend:
            kind, ex = self.pend.pop(0)
            self.rd.event(kind, t, x, y, ex)
            if kind in ("collision", "drive_stall"):
                self.get_logger().warn("%s [%s] en (%.2f, %.2f)" % (kind.upper(), ex.get("src"), x, y))
        # llegada
        if self.a.auto:
            if d <= self.a.tol and abs(vx) < 0.005 and abs(wz) < 0.01:
                self.t_quieto = self.t_quieto or now
                if now - self.t_quieto > 2.0:
                    self.fin = "reached"
            else:
                self.t_quieto = None
        if self.a.max_s and t > self.a.max_s:
            self.fin = "timeout"

    # ---------------------------------------------------------------- cierre
    def cierra(self, result=None):
        T = time.time() - self.t0; s = self.rd.rec["samples"]
        if result is None:
            result = self.fin or ("reached" if s and s[-1]["d"] <= self.a.tol else "aborted")
        if self.a.result_file and os.path.exists(self.a.result_file):
            result = open(self.a.result_file).read().strip() or result
        plen = core.path_len(self.trail)
        recta = math.hypot(self.trail[-1][0] - self.trail[0][0], self.trail[-1][1] - self.trail[0][1]) if self.trail else 0.0
        ln = [x["laser_noise"] for x in s if isinstance(x.get("laser_noise"), (int, float))]
        tk = sorted(self.ticks)
        summary = {"time_s": round(T, 2), "path_m": round(plen, 2), "straight_m": round(recta, 2),
                   "efficiency": round(recta / plen, 2) if plen > 0 else 0.0,
                   "collisions": sum(1 for e in self.rd.rec["events"] if e["kind"] == "collision"),
                   "c0min": round(self.minc0, 2), "reloc_jumps": self.njumps, "obs_max": self.obs_max,
                   "laser_noise_mean": round(statistics.mean(ln), 3) if ln else "",
                   "laser_noise_max": round(max(ln), 3) if ln else "",
                   "scan_hz": round(self.n_scan / T, 2) if T > 0 else "",
                   "stale_pct": round(100.0 * self.stale / max(1, self.nt), 1),
                   "tick_ms_p95": round(1000 * tk[int(0.95 * (len(tk) - 1))], 1) if tk else "",
                   "oido": sorted(self.oido), "notes": " | ".join(self.notas),
                   # una run sin ruedas no tiene detector de colision: se dice, no se deja un 0 que parezca limpio
                   "valid": False if "joints" not in self.oido or not s else True,
                   "invalid_reason": "" if "joints" in self.oido and s else ("sin joint_states: collisions no es fiable" if s else "sin pose")}
        fj, fc, fila = self.rd.finish(result, summary)
        print("\n  run     %s\n  stats   %s" % (fj, fc))
        print("  %s · %.1f s · %.2f m · ef %.2f · col %s · collision_ahead %s · c0min %.2f · oido %s"
              % (result, T, plen, summary["efficiency"], fila["collisions"], fila["collision_ahead"], self.minc0, sorted(self.oido)))
        if not summary["valid"]:
            print("  *** RUN MARCADA NO VALIDA: %s ***" % summary["invalid_reason"])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--label", required=True); ap.add_argument("--goal", type=float, nargs=2)
    ap.add_argument("--marca", help="nombre en ~/ab/marcas.txt, en vez de --goal")
    ap.add_argument("--mode", default="ours"); ap.add_argument("--arm", default=None)
    ap.add_argument("--env", default=os.environ.get("SUMMIT_ENV", "real"), choices=["real", "sim"])
    ap.add_argument("--dataset", default=os.path.expanduser("~/dataset"))
    ap.add_argument("--tol", type=float, default=0.25); ap.add_argument("--auto", action="store_true")
    ap.add_argument("--max-s", type=float, default=0.0); ap.add_argument("--result-file", default=None)
    ap.add_argument("--ns", default="/robot"); ap.add_argument("--scan", default="/top_laser/scan")
    ap.add_argument("--joints", default="/joint_states"); ap.add_argument("--mapa", default="/map")
    ap.add_argument("--cmd-efectivo", default="/robotnik_base_control/cmd_vel_limited",
                    help="la orden que LLEGA al controlador, tras el mux y los limites: contra esta se miden las ruedas")
    ap.add_argument("--cmd", nargs="+", default=["/cmd_vel_nav", "/move_base/cmd_vel"],
                    help="las fuentes: solo sirven para saber QUIEN manda (Nav2 o el cruce por laser)")
    ap.add_argument("--cmd-externo", default="/move_base/cmd_vel", help="por donde manda cruza_vano.py")
    ap.add_argument("--frame-map", default="robot_map"); ap.add_argument("--frame-odom", default="robot_odom")
    ap.add_argument("--frame-base", default="robot_base_footprint"); ap.add_argument("--sin-volcado", action="store_true")
    a = ap.parse_args()
    if a.goal is None:
        if not a.marca:
            ap.error("hace falta --goal X Y o --marca NOMBRE")
        for ln in open(os.path.expanduser("~/ab/marcas.txt")):
            f = ln.split()
            if f and f[0] == a.marca:
                a.goal = [float(f[1]), float(f[2])]
        if a.goal is None:
            ap.error("la marca %r no esta en ~/ab/marcas.txt" % a.marca)
    rclpy.init(); n = Registro(a)
    signal.signal(signal.SIGTERM, lambda *_: setattr(n, "fin", n.fin or "sigterm"))
    try:
        while rclpy.ok() and n.fin is None:
            rclpy.spin_once(n, timeout_sec=0.2)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass                                  # en Humble el Ctrl+C llega como ExternalShutdownException, no como KeyboardInterrupt
    finally:
        n.cierra(None if n.fin in (None, "sigterm") else n.fin)
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
