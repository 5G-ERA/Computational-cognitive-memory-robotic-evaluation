#!/usr/bin/env python3
"""Mide el error de AMCL contra la verdad del banco, con el vano como referencia. 17-sep-2026.

Compara la pose que **usa Nav2** —la transformada map→base_footprint, no el topic `/robot/amcl_pose`, que sólo se
publica cuando hay actualización— con `/sim/pose_verdad`. Descompone el error en la dirección que importa para una
puerta: **lateral al eje del vano** (el que te mete en la jamba) y a lo largo del eje.

  python3 error_amcl.py [segundos] [--csv FICHERO]

Saca una tabla por tramos de la travesía y el resumen; el error lateral en el vano es el número que decide si
`vano − robot` es la holgura real o sólo la de un plano.
"""
import math, sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

EJE = -96.6                                   # rumbo del vano, grados
VANO_A = (1.830, -3.418); VANO_D = (1.646, -5.006)
MEDIO = ((VANO_A[0] + VANO_D[0]) / 2, (VANO_A[1] + VANO_D[1]) / 2)


class Error(Node):
    def __init__(self):
        super().__init__("error_amcl")
        self.buf = Buffer(); TransformListener(self.buf, self)
        self.verdad = None; self.sello = None; self.muestras = []
        self.create_subscription(PoseStamped, "/sim/pose_verdad", self.cb, 20)

    def cb(self, m):
        q = m.pose.orientation
        self.verdad = (m.pose.position.x, m.pose.position.y, math.degrees(2 * math.atan2(q.z, q.w)))
        self.sello = m.header.stamp

    def estima(self):
        # A la marca de tiempo de la verdad, NO en «lo último disponible»: con Time(0) tf2 usa el instante común más
        # reciente, que lo marca el eslabón lento (map→odom de AMCL), y se acaba comparando la pose de ahora con la
        # estimación de hace un segundo. Eso daba «errores» de 750 mm y 85° que eran latencia, no AMCL.
        try:
            t = self.buf.lookup_transform("robot_map", "robot_base_footprint", Time.from_msg(self.sello))
        except Exception:
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y, math.degrees(2 * math.atan2(q.z, q.w)))


def main():
    seg = float(sys.argv[1]) if len(sys.argv) > 1 and not sys.argv[1].startswith("--") else 120.0
    csv = sys.argv[sys.argv.index("--csv") + 1] if "--csv" in sys.argv else None
    rclpy.init(); n = Error()
    th = math.radians(EJE); ux, uy = math.cos(th), math.sin(th); vx, vy = -uy, ux
    t0 = time.time(); f = open(csv, "w") if csv else None
    if f:
        f.write("t,vx,vy,vyaw,ex,ey,eyaw,err_lat,err_lon,err_yaw,dist_vano\n")
    while time.time() - t0 < seg:
        rclpy.spin_once(n, timeout_sec=0.1)
        e = n.estima()
        if e is None or n.verdad is None:
            continue
        dx, dy = e[0] - n.verdad[0], e[1] - n.verdad[1]
        lat = -dx * uy + dy * ux          # componente perpendicular al eje del vano: la que mete en la jamba
        lon = dx * ux + dy * uy
        dyaw = (e[2] - n.verdad[2] + 180) % 360 - 180
        dvano = math.dist((n.verdad[0], n.verdad[1]), MEDIO)
        n.muestras.append((time.time() - t0, n.verdad, e, lat, lon, dyaw, dvano))
        if f:
            f.write("%.2f,%.3f,%.3f,%.1f,%.3f,%.3f,%.1f,%.4f,%.4f,%.2f,%.3f\n"
                    % ((n.muestras[-1][0],) + n.verdad + e + (lat, lon, dyaw, dvano)))
        time.sleep(0.05)
    if f:
        f.close()

    def resumen(nombre, ms):
        if not ms:
            print("  %-22s sin muestras" % nombre); return
        lat = [abs(m[3]) for m in ms]; lon = [abs(m[4]) for m in ms]; yaw = [abs(m[5]) for m in ms]
        tot = [math.hypot(m[3], m[4]) for m in ms]
        lat.sort(); tot.sort()
        print("  %-22s n=%4d · lateral med %5.1f mm p95 %5.1f mm máx %5.1f mm · total máx %5.1f mm · rumbo máx %4.1f°"
              % (nombre, len(ms), 1000 * lat[len(lat) // 2], 1000 * lat[int(0.95 * (len(lat) - 1))], 1000 * max(lat),
                 1000 * max(tot), max(yaw)))

    print("muestras: %d en %.0f s" % (len(n.muestras), seg))
    resumen("todo el recorrido", n.muestras)
    resumen("a >2 m del vano", [m for m in n.muestras if m[6] > 2.0])
    resumen("a 1–2 m del vano", [m for m in n.muestras if 1.0 < m[6] <= 2.0])
    resumen("EN EL VANO (<1 m)", [m for m in n.muestras if m[6] <= 1.0])
    lat_vano = [abs(m[3]) for m in n.muestras if m[6] <= 1.0]
    if lat_vano:
        # holgura geométrica por lado con el vano a la medida de la cinta y la huella real
        margen = (0.742 - 0.613) / 2
        print("\n  margen geométrico por lado: %.1f mm · error lateral máximo de AMCL en el vano: %.1f mm → %s"
              % (1000 * margen, 1000 * max(lat_vano), "cabe" if max(lat_vano) < margen else "NO CABE"))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
