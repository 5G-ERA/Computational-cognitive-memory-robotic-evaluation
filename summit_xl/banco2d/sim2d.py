#!/usr/bin/env python3
"""Simulador 2D del Summit XL sobre el mapa real. 17-sep-2026, noche.

Un robot cinemático (integra cmd_vel con retardo de primer orden) con la huella real, un láser por trazado de rayos
sobre el mapa de hoy (rbk_2026_09_17_16_23_20, remuestreado a 1 cm) con los mismos parámetros que el scan del robot
(723 rayos, ±180°, 0,45–20 m, 3 Hz), odometría y TF con los nombres del robot, y —lo que el gemelo Isaac no tenía—
detección de choque contra la verdad: si la huella toca una celda ocupada, el robot se para y lo dice.

Topics (los del robot):  /robot/cmd_vel (entra) · /robot/top_laser/scan · /robot/robotnik_base_control/odom · /tf ·
/tf_static.  Propios: /sim/pose (PoseStamped, teletransporta) · /sim/choque (Bool) · /sim/holgura (Float32, m, distancia
mínima de la huella al obstáculo más cercano, la magnitud de la interfaz DCA) · /sim/pose_verdad (PoseStamped).

  python3 sim2d.py --mapa mapa.yaml --x -0.019 --y 0.018 --yaw -1.5 [--tf-verdad] [--ruido 0.01] [--puerta X,Y,YAW,ANCHO]

--tf-verdad publica map→odom exacto (sin AMCL). --puerta redibuja las jambas a la anchura dada (m) centradas en X,Y con
el eje del vano a YAW grados: para medir a qué hueco pasa el behavior server con cada huella.
"""
import argparse, math, os, sys, time
import numpy as np, yaml
from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, JointState
from std_msgs.msg import Bool, Float32
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

FINO = 0.01                      # resolución interna del trazado de rayos y del choque
HUELLA = [(0.361, -0.3065), (0.361, 0.3065), (-0.361, 0.3065), (-0.361, -0.3065)]   # p_local_costmap.yaml, sin padding
LASER_X = -0.01924               # robot_top_3d_laser_link respecto a base_footprint (URDF)
N_RAYOS, ANG_MIN, INC = 723, -math.pi, math.radians(0.498)   # scan real (bolsa 17-sep)
R_MIN, R_MAX, R_MARCHA = 0.45, 20.0, 12.0
HZ_SCAN, HZ_ODOM, HZ_SIM, TIMEOUT_CMD = 3.0, 25.0, 50.0, 0.5
# Límites de aceleración del robot (p_controller_server.yaml: acc_lim_x 0.2 m/s², acc_lim_theta 0.5 rad/s²). Antes
# esto era un retardo de primer orden de 0,2 s y el robot seguía girando tras el comando: el Spin parecía pasarse 3,5×
# cuando en el robot se pasa ~2×. Un límite de aceleración sí frena como el de verdad.
ACC_V, ACC_W = 0.2, 0.5


def quat(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))


class Mapa:
    def __init__(self, ruta_yaml, puerta=None):
        y = yaml.safe_load(open(ruta_yaml))
        img = np.array(Image.open(os.path.join(os.path.dirname(ruta_yaml), y["image"])).convert("L"), dtype=np.float32)
        occ = (255.0 - img) / 255.0
        if y.get("negate", 0):
            occ = 1.0 - occ
        ocupada = occ > y.get("occupied_thresh", 0.65)
        libre = occ < y.get("free_thresh", 0.25)
        desconocida = ~ocupada & ~libre
        self.res0 = float(y["resolution"]); self.ox, self.oy = float(y["origin"][0]), float(y["origin"][1])
        # Las rejillas se quedan en la resolución del mapa y los pasos del trazado son más finos (FINO): remuestrear
        # con un factor no entero —5 cm → 2,5 cm daba 2— desalineaba la geometría y el robot «chocaba» quieto en A.
        # el PNG va de arriba abajo; la fila 0 del array interno es y = origen
        ocupada = np.flipud(ocupada); desconocida = np.flipud(desconocida)
        self.solido = ocupada                      # para el choque: sólo lo ocupado
        self.pared = ocupada | desconocida         # para el láser: lo desconocido también bloquea
        self.res = self.res0; self.alto, self.ancho = self.pared.shape
        if puerta:
            self.redibuja_puerta(*puerta)
        self.dist = None
        try:
            from scipy import ndimage
            self.dist = ndimage.distance_transform_edt(~self.solido) * self.res
        except Exception:
            pass

    def redibuja_puerta(self, px, py, yaw_deg, ancho):
        """Limpia una ventana de 1,2 × 0,6 m centrada en (px,py) y pinta dos jambas de 10 cm dejando `ancho` libre."""
        th = math.radians(yaw_deg); ux, uy = math.cos(th), math.sin(th); vx, vy = -uy, ux   # u = eje del vano, v = transversal
        for a in np.arange(-0.3, 0.3, self.res / 3):
            for b in np.arange(-0.6, 0.6, self.res / 3):
                i, j = self.celda(px + a * ux + b * vx, py + a * uy + b * vy)
                if 0 <= i < self.alto and 0 <= j < self.ancho:
                    jamba = abs(b) >= ancho / 2 and abs(a) <= 0.05
                    self.solido[i, j] = jamba; self.pared[i, j] = jamba

    def celda(self, x, y):
        return int((y - self.oy) / self.res), int((x - self.ox) / self.res)

    def ocupado(self, ii, jj, capa):
        dentro = (ii >= 0) & (ii < self.alto) & (jj >= 0) & (jj < self.ancho)
        r = np.ones_like(ii, dtype=bool)
        r[dentro] = capa[ii[dentro], jj[dentro]]
        return r


class Sim(Node):
    def __init__(self, a):
        super().__init__("sim2d_summit")
        self.m = Mapa(a.mapa, a.puerta)
        self.x, self.y, self.yaw = a.x, a.y, math.radians(a.yaw)
        self.ox0 = (self.x, self.y, self.yaw)       # odom: la verdad, sin deriva, arrancando en el origen de odom
        self.v = self.w = 0.0; self.v_cmd = self.w_cmd = 0.0; self.t_cmd = 0.0
        self.choque = False; self.ruido = a.ruido; self.tf_verdad = a.tf_verdad
        self.angs = ANG_MIN + INC * np.arange(N_RAYOS)
        be = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub_scan = self.create_publisher(LaserScan, a.topic_scan, be)
        self.atipicos = a.atipicos
        self.pub_odom = self.create_publisher(Odometry, "/robot/robotnik_base_control/odom", 10)
        self.pub_choque = self.create_publisher(Bool, "/sim/choque", 10)
        self.pub_holg = self.create_publisher(Float32, "/sim/holgura", 10)
        self.pub_verdad = self.create_publisher(PoseStamped, "/sim/pose_verdad", 10)
        # en modo verdad no hay AMCL: se publica /robot/amcl_pose exacto para que summit_ab.sh y summit_cruce.sh funcionen igual
        self.pub_amcl = self.create_publisher(PoseWithCovarianceStamped, "/robot/amcl_pose", 10) if a.tf_verdad else None
        # Lo que el robot real da y el banco no daba: ruedas (hardware) y la orden efectiva tras el mux.
        # Sin esto summit_run_logger.py no tiene con que detectar un atasco en el banco.
        self.pub_js = self.create_publisher(JointState, "/robot/joint_states", be)
        self.pub_lim = self.create_publisher(Twist, "/robot/robotnik_base_control/cmd_vel_limited", 10)
        self.create_subscription(Twist, "/robot/cmd_vel", self.cb_cmd, 10)
        self.create_subscription(PoseStamped, "/sim/pose", self.cb_pose, 10)
        self.tf = TransformBroadcaster(self); self.tfs = StaticTransformBroadcaster(self)
        self.estaticas()
        self.create_timer(1 / HZ_SIM, self.paso); self.create_timer(1 / HZ_ODOM, self.odom); self.create_timer(1 / HZ_SCAN, self.scan)
        self.get_logger().info("sim2d: mapa %dx%d a %.3f m · pose (%.2f, %.2f, %.1f°) · tf_verdad=%s · ruido %.3f"
                               % (self.m.ancho, self.m.alto, self.m.res0,
                                  self.x, self.y, math.degrees(self.yaw), self.tf_verdad, self.ruido))

    def estaticas(self):
        ts = []
        for padre, hijo, x, z in (("robot_base_footprint", "robot_base_link", 0.0, 0.11),
                                  ("robot_base_link", "robot_top_3d_laser_base_link", LASER_X, 0.35642),
                                  ("robot_top_3d_laser_base_link", "robot_top_3d_laser_link", 0.0, 0.0635)):
            t = TransformStamped(); t.header.stamp = self.get_clock().now().to_msg(); t.header.frame_id = padre; t.child_frame_id = hijo
            t.transform.translation.x = x; t.transform.translation.z = z; t.transform.rotation.w = 1.0; ts.append(t)
        self.tfs.sendTransform(ts)

    def cb_cmd(self, m):
        self.v_cmd, self.w_cmd, self.t_cmd = m.linear.x, m.angular.z, time.time()

    def cb_pose(self, m):
        # Teletransporte SIN salto de odometría: se mueve también el origen del marco odom, de modo que la odometría
        # publicada sea continua. Si no, AMCL ve un salto de metros y diverge, y lo que se mide ya no es AMCL.
        ox, oy, oth = self.odom_xy()
        q = m.pose.orientation; self.x, self.y = m.pose.position.x, m.pose.position.y
        self.yaw = 2 * math.atan2(q.z, q.w); self.v = self.w = self.v_cmd = self.w_cmd = 0.0; self.choque = False
        th0 = self.yaw - oth; c, s = math.cos(th0), math.sin(th0)
        self.ox0 = (self.x - (ox * c - oy * s), self.y - (ox * s + oy * c), th0)
        self.get_logger().info("teletransportado a (%.2f, %.2f, %.1f°)" % (self.x, self.y, math.degrees(self.yaw)))

    def huella_toca(self):
        pts = []
        for k in range(4):
            (ax, ay), (bx, by) = HUELLA[k], HUELLA[(k + 1) % 4]
            for s in np.linspace(0, 1, 40):
                pts.append((ax + s * (bx - ax), ay + s * (by - ay)))
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        pts = np.array(pts); wx = self.x + pts[:, 0] * c - pts[:, 1] * s; wy = self.y + pts[:, 0] * s + pts[:, 1] * c
        ii = ((wy - self.m.oy) / self.m.res).astype(int); jj = ((wx - self.m.ox) / self.m.res).astype(int)
        toca = self.m.ocupado(ii, jj, self.m.solido).any()
        holg = float("nan")
        if self.m.dist is not None:
            d = np.full(len(ii), np.nan); dentro = (ii >= 0) & (ii < self.m.alto) & (jj >= 0) & (jj < self.m.ancho)
            d[dentro] = self.m.dist[ii[dentro], jj[dentro]]; holg = float(np.nanmin(d)) if dentro.any() else float("nan")
        return toca, holg

    def paso(self):
        self._paso_base(); self.pub_ruedas()

    def pub_ruedas(self):
        """Ruedas del Summit (radio 0,1114, via 0,534). Clavado contra algo: giran 0 y el esfuerzo sube."""
        R, T = 0.1114, 0.534
        vl, vr = (self.v - self.w * T / 2) / R, (self.v + self.w * T / 2) / R
        empuja = self.choque and (abs(self.v_cmd) > 0.01 or abs(self.w_cmd) > 0.01)
        ef = 95.0 if empuja else (5.0 if abs(self.v) < 1e-3 and abs(self.w) < 1e-3 else 40.0 + 60.0 * abs(self.v))
        js = JointState(); js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["robot_front_left_wheel_joint", "robot_back_left_wheel_joint",
                   "robot_front_right_wheel_joint", "robot_back_right_wheel_joint"]
        js.velocity = [vl, vl, vr, vr]; js.effort = [ef, ef, ef, ef]; js.position = [0.0] * 4
        self.pub_js.publish(js)
        lim = Twist(); lim.linear.x = float(self.v_cmd); lim.angular.z = float(self.w_cmd); self.pub_lim.publish(lim)

    def _paso_base(self):
        dt = 1 / HZ_SIM
        if time.time() - self.t_cmd > TIMEOUT_CMD:
            self.v_cmd = self.w_cmd = 0.0
        if self.choque:
            self.v = self.w = 0.0; return
        self.v += max(-ACC_V * dt, min(ACC_V * dt, self.v_cmd - self.v))
        self.w += max(-ACC_W * dt, min(ACC_W * dt, self.w_cmd - self.w))
        self.x += self.v * math.cos(self.yaw) * dt; self.y += self.v * math.sin(self.yaw) * dt; self.yaw += self.w * dt
        toca, holg = self.huella_toca()
        self.pub_holg.publish(Float32(data=holg))
        self.pub_choque.publish(Bool(data=False))   # se publica SIEMPRE: quien escucha necesita ver el fin del choque
        if toca:
            self.choque = True; self.v = self.w = 0.0
            self.pub_choque.publish(Bool(data=True))
            self.get_logger().error("¡CHOQUE! huella contra pared en (%.2f, %.2f, %.1f°)" % (self.x, self.y, math.degrees(self.yaw)))

    def odom_xy(self):
        """Pose en el marco odom: la verdad referida a la pose en que se ancló el marco."""
        x0, y0, th0 = self.ox0; c, s = math.cos(-th0), math.sin(-th0)
        dx, dy = self.x - x0, self.y - y0
        return dx * c - dy * s, dx * s + dy * c, self.yaw - th0

    def odom(self):
        now = self.get_clock().now().to_msg()
        # odom = verdad expresada en el marco odom, cuyo origen es la pose inicial (map→odom = pose inicial)
        x0, y0, th0 = self.ox0; ox, oy, oth = self.odom_xy()
        o = Odometry(); o.header.stamp = now; o.header.frame_id = "robot_odom"; o.child_frame_id = "robot_base_footprint"
        o.pose.pose.position.x = ox; o.pose.pose.position.y = oy; o.pose.pose.orientation = quat(oth)
        o.twist.twist.linear.x = self.v; o.twist.twist.angular.z = self.w; self.pub_odom.publish(o)
        t = TransformStamped(); t.header.stamp = now; t.header.frame_id = "robot_odom"; t.child_frame_id = "robot_base_footprint"
        t.transform.translation.x = ox; t.transform.translation.y = oy; t.transform.rotation = quat(oth)
        ts = [t]
        if self.tf_verdad:
            g = TransformStamped(); g.header.stamp = now; g.header.frame_id = "robot_map"; g.child_frame_id = "robot_odom"
            g.transform.translation.x = x0; g.transform.translation.y = y0; g.transform.rotation = quat(th0); ts.append(g)
        self.tf.sendTransform(ts)
        p = PoseStamped(); p.header.stamp = now; p.header.frame_id = "robot_map"
        p.pose.position.x = self.x; p.pose.position.y = self.y; p.pose.orientation = quat(self.yaw); self.pub_verdad.publish(p)
        if self.pub_amcl is not None:
            q = PoseWithCovarianceStamped(); q.header = p.header; q.pose.pose = p.pose; self.pub_amcl.publish(q)

    def scan(self):
        lx = self.x + LASER_X * math.cos(self.yaw); ly = self.y + LASER_X * math.sin(self.yaw)
        dirs = self.angs + self.yaw; cx, sy = np.cos(dirs), np.sin(dirs)
        rangos = np.full(N_RAYOS, np.inf); vivos = np.ones(N_RAYOS, dtype=bool)
        for k in range(1, int(R_MARCHA / FINO) + 1):
            r = k * FINO
            ii = ((ly + r * sy[vivos] - self.m.oy) / self.m.res).astype(int); jj = ((lx + r * cx[vivos] - self.m.ox) / self.m.res).astype(int)
            hit = self.m.ocupado(ii, jj, self.m.pared)
            idx = np.flatnonzero(vivos)[hit]; rangos[idx] = r; vivos[idx] = False
            if not vivos.any():
                break
        if self.ruido > 0:
            fin = np.isfinite(rangos); rangos[fin] += np.random.normal(0, self.ruido, fin.sum())
        if self.atipicos > 0:
            # Cola pesada como la del láser real (bolsa del 17-sep: p99 de la desviación 112 mm): una fracción de los
            # rayos con retorno se desplaza ±12 cm. Un solo atípico dentro del vano marca una celda letal.
            fin = np.flatnonzero(np.isfinite(rangos)); k = np.random.random(len(fin)) < self.atipicos
            rangos[fin[k]] += np.random.uniform(-0.12, 0.12, int(k.sum()))
        rangos[rangos < R_MIN] = np.inf
        m = LaserScan(); m.header.stamp = self.get_clock().now().to_msg(); m.header.frame_id = "robot_top_3d_laser_link"
        m.angle_min = ANG_MIN; m.angle_max = ANG_MIN + INC * (N_RAYOS - 1); m.angle_increment = INC
        m.time_increment = 0.0; m.scan_time = 1 / HZ_SCAN; m.range_min = R_MIN; m.range_max = R_MAX
        m.ranges = [float(v) for v in rangos]; self.pub_scan.publish(m)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mapa", required=True); ap.add_argument("--x", type=float, default=0.0); ap.add_argument("--y", type=float, default=0.0)
    ap.add_argument("--yaw", type=float, default=0.0, help="grados"); ap.add_argument("--tf-verdad", action="store_true")
    ap.add_argument("--ruido", type=float, default=0.01, help="sigma del láser, m")
    ap.add_argument("--atipicos", type=float, default=0.0, help="fracción de rayos con un atípico de ±12 cm")
    ap.add_argument("--topic-scan", default="/robot/top_laser/scan")
    ap.add_argument("--puerta", default=None, help="X,Y,YAW,ANCHO: redibuja las jambas a esa anchura")
    a = ap.parse_args()
    if a.puerta:
        a.puerta = [float(v) for v in a.puerta.split(",")]
    rclpy.init(); n = Sim(a)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
