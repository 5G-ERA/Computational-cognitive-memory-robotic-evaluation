"""Estructura vertical de la nube 3D del Summit, ya en frame G1."""
import json, math
import numpy as np

T = json.load(open("/home/ros/isaac_ws/g1_summit_transform.json"))["summit_to_g1"]
s, th, t = T["scale"], math.radians(T["rot_deg"]), T["t"]
c, sn = math.cos(th), math.sin(th)

xs, ys, zs = [], [], []
with open("/home/ros/isaac_ws/summit_map.pcd") as f:
    datos = False
    for line in f:
        if not datos:
            if line.startswith("DATA"):
                datos = True
            continue
        p = line.split()
        if len(p) < 3:
            continue
        x, y, z = float(p[0]), float(p[1]), float(p[2])
        xs.append(s * (c * x - sn * y) + t[0])
        ys.append(s * (sn * x + c * y) + t[1])
        zs.append(z)
X, Y, Z = np.array(xs), np.array(ys), np.array(zs)
print("puntos:", len(X))
print("extension X: %.1f .. %.1f   Y: %.1f .. %.1f   Z: %.2f .. %.2f" % (
    X.min(), X.max(), Y.min(), Y.max(), Z.min(), Z.max()))
print("\nhistograma de ALTURA (z):")
h, edges = np.histogram(Z, bins=24, range=(Z.min(), min(Z.max(), 4.0)))
for i, n in enumerate(h):
    if n:
        print("  %5.2f-%5.2f m : %6d %s" % (edges[i], edges[i+1], n, "#" * int(40 * n / h.max())))
np.save("/home/ros/isaac_ws/nube_g1.npy", np.stack([X, Y, Z], axis=1).astype(np.float32))
print("\nguardado nube_g1.npy (frame G1)")
