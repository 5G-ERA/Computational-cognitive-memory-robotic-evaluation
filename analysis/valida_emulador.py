"""¿Reproduce el emulador la curva real que lo alimenta? (auto-consistencia)"""
import sys, math, statistics
sys.path.insert(0, "sim")
from emulador_deteccion import EmuladorDeteccion

# escenario de la tanda: silla a distancia d, de frente, sin nada en medio
print("%-8s %-6s %8s %8s   %s" % ("dist", "luz", "P(sim)", "conf sim", "real (P / conf)"))
REAL = {(0.3, "luz"): (1.00, 0.94), (0.3, "poca"): (1.00, 0.94),
        (1.5, "luz"): (0.67, 0.93), (1.5, "poca"): (0.78, 0.91),
        (1.8, "luz"): (0.95, 0.82), (1.8, "poca"): (1.00, 0.54)}
malo = 0
for (d, luz), (pr, cr) in sorted(REAL.items()):
    e = EmuladorDeteccion([("chair", d, 0.0)], ocupacion=set(), semilla=11)
    brillo = 116 if luz == "luz" else 85
    n, confs = 400, []
    hits = 0
    for _ in range(n):
        r = e.detecta(0.0, 0.0, 0.0, brillo)
        if r:
            hits += 1
            confs.append(r[0][1])
    p = hits / n
    c = statistics.median(confs) if confs else 0
    dif = abs(p - pr) + abs(c - cr)
    if dif > 0.12:
        malo += 1
    print("%-8.1f %-6s %8.2f %8.2f   %.2f / %.2f%s" % (
        d, luz, p, c, pr, cr, "   <-- desvia" if dif > 0.12 else ""))
print("\n%s" % ("TODAS dentro de tolerancia" if malo == 0 else "%d condiciones desvian" % malo))

# y el par W2: ¿aparece el contraste de luz a 1.8 m?
e = EmuladorDeteccion([("chair", 1.8, 0.0)], ocupacion=set(), semilla=3)
for luz, brillo in (("luz", 116), ("poca", 85)):
    cs = [r[0][1] for _ in range(300) for r in [e.detecta(0, 0, 0, brillo)] if r]
    print("W2 a 1.8m %-5s: conf mediana %.2f" % (luz, statistics.median(cs)))
