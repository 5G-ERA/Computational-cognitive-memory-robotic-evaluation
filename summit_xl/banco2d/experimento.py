#!/usr/bin/env python3
"""Banco de la puerta en el simulador 2D. 17-sep-2026, noche.

Hace, sin tocar el robot, las preguntas que hoy costaron una caja:

  plan DEST [DEST…]        ¿planifica NavFn de la pose actual a cada marca? (ComputePathToPose)
  barrido PARAM V1 V2 …    repite `plan B` cambiando un parámetro en caliente (footprint_padding, inflation_radius,
                           resolution no: esa exige relanzar) y dice con cuál pasa
  va DEST                  NavigateToPose de verdad, siguiendo holgura y choque hasta que acabe
  recto METROS             DriveOnHeading, lo mismo
  cruza [EJE] [DEST]       la maniobra del robot: Nav2 a vano_antes → alinear al eje → recto → Nav2 al destino
  pasa [N] [EJE]           N pasadas por el vano guiadas por la verdad (cmd_vel), para medir el error de AMCL dentro
  rectos N [METROS]        N cruces rectos (DriveOnHeading) desde vano_antes alineado, sin limpiar el costmap entre uno y otro
  perfil [DESDE HASTA]     holgura de la huella real a lo largo del eje del vano, paso a paso (la magnitud DCA)
  holgura                  imprime la holgura actual (m de la huella al obstáculo más cercano, verdad del mapa)

Usa los topics del robot bajo /robot y el dominio 40.
"""
import math, sys, time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose, DriveOnHeading, Spin
from rcl_interfaces.srv import SetParameters, GetParameters
from std_msgs.msg import Bool, Float32

MARCAS = "/home/ros/ab/marcas.txt"


def frange(a, b, p):
    v = a
    while v <= b + 1e-9:
        yield v; v += p


def marcas():
    d = {}
    for ln in open(MARCAS):
        c = ln.split()
        if len(c) >= 4:
            d[c[0]] = (float(c[1]), float(c[2]), float(c[3]))
    return d


def pose(x, y, yaw_deg, stamp):
    p = PoseStamped(); p.header.frame_id = "robot_map"; p.header.stamp = stamp
    p.pose.position.x = x; p.pose.position.y = y
    p.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2); p.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2)
    return p


class Banco(Node):
    def __init__(self):
        super().__init__("banco_puerta")
        self.holg = float("nan"); self.choque = False; self.verdad = None
        self.create_subscription(Float32, "/sim/holgura", lambda m: setattr(self, "holg", m.data), 10)
        self.create_subscription(Bool, "/sim/choque", lambda m: setattr(self, "choque", m.data), 10)
        self.create_subscription(PoseStamped, "/sim/pose_verdad", self.cb_verdad, 10)
        self.m = marcas()
        self.cmd = self.create_publisher(Twist, "/robot/cmd_vel", 10)

    def cb_verdad(self, m):
        q = m.pose.orientation
        self.verdad = (m.pose.position.x, m.pose.position.y, math.degrees(2 * math.atan2(q.z, q.w)))

    def espera_verdad(self, t=5.0):
        """Espera a una pose NUEVA. Antes devolvía la cacheada si ya había una, así que el bucle de alineación
        leía rumbos viejos y parecía que el giro se pasaba 10x: era el banco, no el robot."""
        self.verdad = None; t0 = time.time()
        while self.verdad is None and time.time() - t0 < t:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.verdad

    def destino(self, nombre):
        if nombre in self.m:
            return self.m[nombre]
        return tuple(float(v) for v in nombre.split(","))

    # --- acciones ------------------------------------------------------------------------------------------
    def accion(self, tipo, nombre, meta, seguir=False, tope=180):
        cli = ActionClient(self, tipo, nombre)
        if not cli.wait_for_server(timeout_sec=10):
            return "SIN_SERVIDOR", None
        fut = cli.send_goal_async(meta)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return "RECHAZADO", None
        res = gh.get_result_async(); t0 = time.time(); peor = float("inf")
        while not res.done() and time.time() - t0 < tope:
            rclpy.spin_once(self, timeout_sec=0.2)
            if not math.isnan(self.holg):
                peor = min(peor, self.holg)
            if self.choque:
                gh.cancel_goal_async(); time.sleep(0.5)
                return "CHOQUE", peor
        if not res.done():
            gh.cancel_goal_async()
            return "TIEMPO", peor
        st = {1: "ACEPTADO", 2: "EJECUTANDO", 3: "CANCELANDO", 4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}.get(res.result().status, str(res.result().status))
        return st, (peor if seguir else res.result().result)

    def plan(self, nombre):
        x, y, yaw = self.destino(nombre)
        g = ComputePathToPose.Goal(); g.goal = pose(x, y, yaw, self.get_clock().now().to_msg()); g.use_start = False
        st, r = self.accion(ComputePathToPose, "/robot/compute_path_to_pose", g, tope=30)
        n = len(r.path.poses) if st == "SUCCEEDED" and hasattr(r, "path") else 0
        largo = 0.0
        if n > 1:
            ps = [(p.pose.position.x, p.pose.position.y) for p in r.path.poses]
            largo = sum(math.dist(ps[i], ps[i + 1]) for i in range(len(ps) - 1))
        return st, n, largo

    def tele(self, x, y, yaw, avisa_amcl=True):
        """Teletransporta el simulador y, por defecto, le da a AMCL la misma pose inicial.

        Sin el aviso, AMCL no puede enterarse: la odometría del banco es continua a través del salto a propósito
        (para no hacerle divergir), así que el robot «aparece» en otro sitio sin que nada se lo diga y el error medido
        es el del salto, no el de AMCL. Avisarle es lo que se hace en el laboratorio cuando Natan mueve el robot.
        """
        p = PoseStamped(); p.header.frame_id = "robot_map"; p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = x; p.pose.position.y = y
        p.pose.orientation.z = math.sin(math.radians(yaw) / 2); p.pose.orientation.w = math.cos(math.radians(yaw) / 2)
        if not hasattr(self, "_pub_tele"):
            self._pub_tele = self.create_publisher(PoseStamped, "/sim/pose", 10)
            self._pub_ini = self.create_publisher(PoseWithCovarianceStamped, "/robot/initialpose", 10)
        for _ in range(5):
            self._pub_tele.publish(p); rclpy.spin_once(self, timeout_sec=0.1)
        if avisa_amcl:
            q = PoseWithCovarianceStamped(); q.header = p.header; q.pose.pose = p.pose
            q.pose.covariance[0] = q.pose.covariance[7] = 0.25; q.pose.covariance[35] = 0.068
            for _ in range(3):
                self._pub_ini.publish(q); rclpy.spin_once(self, timeout_sec=0.2)
            time.sleep(2)
        self.choque = False     # el teletransporte limpia el choque también en el banco, no sólo en el simulador

    def alinea(self, eje, tol=1.0, tope=40):
        """Alineación en lazo cerrado por cmd_vel, en vez del Spin de Nav2.

        Medido en el banco: el Spin del behavior_server arranca a `min_rotational_vel` 0,4 rad/s y la base frena a
        0,5 rad/s², así que **cualquier** Spin se pasa ~9° (0,4²/2/0,5 = 0,16 rad) y el bucle de media ganancia
        oscila entre ±5° sin converger nunca. Con un mando propio a 0,03–0,15 rad/s la frenada cabe en el error.
        """
        t0 = time.time()
        while time.time() - t0 < tope:
            self.espera_verdad(2)
            err = (eje - self.verdad[2] + 180) % 360 - 180
            if abs(err) <= tol:
                self.cmd.publish(Twist()); return err
            w = math.copysign(min(0.15, max(0.03, abs(math.radians(err)) * 0.6)), err)
            m = Twist(); m.angular.z = w; self.cmd.publish(m)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.cmd.publish(Twist()); return err

    # --- parámetros en caliente ----------------------------------------------------------------------------
    def pon(self, nodo, nombre, valor):
        cli = self.create_client(SetParameters, "%s/set_parameters" % nodo)
        if not cli.wait_for_service(timeout_sec=5):
            return False
        req = SetParameters.Request(); req.parameters = [Parameter(nombre, value=valor).to_parameter_msg()]
        fut = cli.call_async(req); rclpy.spin_until_future_complete(self, fut, timeout_sec=8)
        return bool(fut.result() and fut.result().results[0].successful)


def main():
    rclpy.init(); b = Banco(); a = sys.argv[1:]
    if not a:
        print(__doc__); return
    cmd = a[0]
    b.espera_verdad()
    print("pose verdad: (%.2f, %.2f) yaw %.1f°" % b.verdad if b.verdad else "sin pose")
    if cmd == "plan":
        for d in a[1:] or ["B"]:
            st, n, largo = b.plan(d)
            print("  plan → %-14s %-10s %3d puntos %6.2f m" % (d, st, n, largo))
    elif cmd == "barrido":
        param, vals = a[1], [float(v) for v in a[2:]]
        for v in vals:
            ok1 = b.pon("/robot/global_costmap/global_costmap", param, v)
            ok2 = b.pon("/robot/local_costmap/local_costmap", param, v)
            time.sleep(2.0)
            st, n, largo = b.plan("B")
            print("  %s=%.3f (global %s, local %s) → %-10s %3d puntos %6.2f m" % (param, v, ok1, ok2, st, n, largo))
    elif cmd == "va":
        x, y, yaw = b.destino(a[1])
        g = NavigateToPose.Goal(); g.pose = pose(x, y, yaw, b.get_clock().now().to_msg())
        st, peor = b.accion(NavigateToPose, "/robot/navigate_to_pose", g, seguir=True, tope=int(a[2]) if len(a) > 2 else 180)
        b.espera_verdad(1)
        print("  navegar → %s · holgura mínima %.3f m · fin (%.2f, %.2f) yaw %.1f°" % ((st, peor) + b.verdad))
    elif cmd == "recto":
        g = DriveOnHeading.Goal(); g.target.x = float(a[1]); g.speed = 0.05
        g.time_allowance.sec = int(a[2]) if len(a) > 2 else 90
        st, peor = b.accion(DriveOnHeading, "/robot/drive_on_heading", g, seguir=True, tope=120)
        b.espera_verdad(1)
        print("  recto %.2f m → %s · holgura mínima %.3f m · fin (%.2f, %.2f) yaw %.1f°" % ((float(a[1]), st, peor) + b.verdad))
    elif cmd == "cruza":
        eje = float(a[1]) if len(a) > 1 else -96.6; dest = a[2] if len(a) > 2 else "B"
        ax, ay, _ = b.destino("vano_antes"); dx, dy, _ = b.destino("vano_despues")
        largo = math.dist((ax, ay), (dx, dy)) + 0.15
        g = NavigateToPose.Goal(); g.pose = pose(ax, ay, eje, b.get_clock().now().to_msg())
        st, peor = b.accion(NavigateToPose, "/robot/navigate_to_pose", g, seguir=True, tope=180)
        b.espera_verdad(1); print("  1) a vano_antes → %s · holgura mín %.3f · en (%.2f, %.2f) yaw %.1f°" % ((st, peor) + b.verdad))
        if st != "SUCCEEDED":
            rclpy.shutdown(); return
        err = b.alinea(eje, tol=1.0)
        b.espera_verdad(1); print("     alineado en lazo cerrado: rumbo %.1f° · error %+.1f°" % (b.verdad[2], err))
        dg = DriveOnHeading.Goal(); dg.target.x = largo; dg.speed = 0.05; dg.time_allowance.sec = 90
        st, peor = b.accion(DriveOnHeading, "/robot/drive_on_heading", dg, seguir=True, tope=120)
        b.espera_verdad(1); print("  2) recto %.2f m → %s · holgura mín %.3f · en (%.2f, %.2f) yaw %.1f°" % ((largo, st, peor) + b.verdad))
        if st == "CHOQUE":
            rclpy.shutdown(); return
        x, y, yaw = b.destino(dest)
        g = NavigateToPose.Goal(); g.pose = pose(x, y, yaw, b.get_clock().now().to_msg())
        st, peor = b.accion(NavigateToPose, "/robot/navigate_to_pose", g, seguir=True, tope=180)
        b.espera_verdad(1)
        print("  3) a %s → %s · holgura mín %.3f · en (%.2f, %.2f) yaw %.1f° · a %.2f m de la marca"
              % (dest, st, peor, b.verdad[0], b.verdad[1], b.verdad[2], math.dist((b.verdad[0], b.verdad[1]), (x, y))))
    elif cmd == "pasa":
        # Pasadas por el vano guiadas por la VERDAD, no por AMCL: el objetivo no es que Nav2 lo consiga, es meter al
        # robot en el vano tantas veces como haga falta para medir ahí el error de AMCL.
        n = int(a[1]) if len(a) > 1 else 3; eje = float(a[2]) if len(a) > 2 else -96.6
        ax, ay, _ = b.destino("vano_antes"); dx, dy, _ = b.destino("vano_despues")
        th = math.radians(eje); ux, uy = math.cos(th), math.sin(th)
        # Arranca 0,30 m antes de vano_antes y para en s=+0,55 del centro del vano: medido con `perfil`, a s=+0,75 la
        # holgura es CERO (hay mobiliario justo detrás de la puerta). Seguir recto más allá es el choque del 17-sep.
        largo = 0.30 + math.dist((ax, ay), (dx, dy)) / 2 + 0.55
        for k in range(n):
            b.tele(ax - 0.30 * ux, ay - 0.30 * uy, eje); time.sleep(3)
            b.alinea(eje, tol=0.5)
            t0 = time.time(); rec = 0.0; prev = None
            while rec < largo and time.time() - t0 < 90:
                b.espera_verdad(2)
                if b.choque:
                    break
                px, py, yaw = b.verdad
                if prev: rec += math.dist((px, py), prev)
                prev = (px, py)
                err = (eje - yaw + 180) % 360 - 180
                m = Twist(); m.linear.x = 0.08; m.angular.z = max(-0.1, min(0.1, math.radians(err) * 0.8)); b.cmd.publish(m)
                rclpy.spin_once(b, timeout_sec=0.05)
            b.cmd.publish(Twist()); b.espera_verdad(2)
            print("  pasada %d: %.2f m · %s · fin (%.2f, %.2f) yaw %.1f°" % (k + 1, rec, "CHOQUE" if b.choque else "ok", ) + b.verdad[:0] or None) if False else print(
                "  pasada %d: recorrido %.2f m · %s · fin (%.2f, %.2f) yaw %.1f°" % (k + 1, rec, "CHOQUE" if b.choque else "ok", b.verdad[0], b.verdad[1], b.verdad[2]))
    elif cmd == "rectos":
        # El recto del robot, repetido: teletransporte a vano_antes con el rumbo del eje y DriveOnHeading. El costmap
        # NO se limpia entre repeticiones, a propósito: lo que se quiere ver es si las marcas del láser se acumulan.
        n = int(a[1]); metros = float(a[2]) if len(a) > 2 else 1.35; eje = -96.6
        ax, ay, _ = b.destino("vano_antes"); ok = 0
        for k in range(n):
            b.tele(ax, ay, eje, avisa_amcl=False); time.sleep(4)
            g = DriveOnHeading.Goal(); g.target.x = metros; g.speed = 0.05; g.time_allowance.sec = 60
            st, peor = b.accion(DriveOnHeading, "/robot/drive_on_heading", g, seguir=True, tope=90)
            b.espera_verdad(2); ux, uy = math.cos(math.radians(eje)), math.sin(math.radians(eje))
            rec = (b.verdad[0] - ax) * ux + (b.verdad[1] - ay) * uy
            ok += st == "SUCCEEDED"
            print("  recto %d: %-9s recorrido %.2f m de %.2f · holgura mín %.3f m" % (k + 1, st, rec, metros, peor))
        print("  → %d de %d" % (ok, n))
    elif cmd == "perfil":
        # Recorre el eje del vano teletransportando la huella y leyendo la holgura contra la verdad del mapa. Es el
        # perfil de holgura del paso: dónde está el estrechamiento y cuánto margen queda de verdad.
        d0 = float(a[1]) if len(a) > 1 else -1.2; d1 = float(a[2]) if len(a) > 2 else 1.2
        eje = float(a[3]) if len(a) > 3 else -96.6
        ax, ay, _ = b.destino("vano_antes"); dx, dy, _ = b.destino("vano_despues")
        mx, my = (ax + dx) / 2, (ay + dy) / 2
        th = math.radians(eje); ux, uy = math.cos(th), math.sin(th)
        print("  s(m)   holgura(mm)   choque")
        peor = (9.9, 0.0)
        for sm in [round(v, 2) for v in list(frange(d0, d1, 0.05))]:
            b.tele(mx + sm * ux, my + sm * uy, eje)
            b.holg = float("nan")
            t0 = time.time()
            while math.isnan(b.holg) and time.time() - t0 < 2:
                rclpy.spin_once(b, timeout_sec=0.05)
            print("  %+5.2f   %8.1f      %s" % (sm, 1000 * b.holg, "SÍ" if b.choque else ""))
            if b.holg < peor[0]:
                peor = (b.holg, sm)
        print("\n  holgura mínima %.1f mm a s=%+.2f m del centro del vano (margen geométrico teórico 64,5 mm)"
              % (1000 * peor[0], peor[1]))
    elif cmd == "holgura":
        for _ in range(20):
            rclpy.spin_once(b, timeout_sec=0.2)
        print("  holgura %.3f m · choque %s" % (b.holg, b.choque))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
