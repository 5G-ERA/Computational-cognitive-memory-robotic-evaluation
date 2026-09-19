#!/bin/bash
# Summit XL · A → puerta → B, cruzando por HOLGURA medida en el láser. 18-sep-2026.
#
#   bash ~/summit_ab_holgura.sh [destino]        # destino: B por defecto
#
# Es summit_cruce.sh con el paso 3 cambiado: donde aquél usaba el `DriveOnHeading` de Nav2, éste usa cruza_vano.py.
# Motivo, medido hoy con tres travesías y una cuarta: el recto de Nav2 comprueba la colisión contra un costmap
# construido sobre la pose de AMCL, y AMCL SALTA ~5,7 cm al entrar en el marco (medido: el lateral pasa de −2,2 a
# +3,5 cm recorriendo 0,39 m con 0,7° de error de rumbo, cuando la geometría sólo permite 5 mm). Con 5 cm de margen
# por lado, ese salto basta para que el costmap vea colisión donde hay hueco: las travesías 2 y 3 se clavaron en el
# mismo punto, 0,39 m. cruza_vano.py no usa AMCL ni el costmap —mide las dos jambas en el láser y se centra entre
# ellas—, y cruzó a la primera con 89 y 113 mm de holgura real y 2,6 mm de error lateral.
#
# Requisitos: robot en A con su pose dada, Natan a un lado del marco CON LA SETA.
set -u
set +u; source /opt/ros/humble/setup.bash; set -u
export ROS_DOMAIN_ID=39 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
DEST=${1:-B}; EJE=${2:--89.0}; TS=$(date +%Y%m%d_%H%M%S); LOG=~/ab/ab_holgura_$TS.txt; CSV=~/ab/holgura_$TS.csv
M=~/ab/marcas.txt
read AX AY _ <<<"$(awk '$1=="vano_antes"{print $2,$3,$4}' $M)"
read DX DY DYAW _ <<<"$(awk -v n="$DEST" '$1==n{print $2,$3,$4}' $M)"
say() { echo "$(date +%H:%M:%S) $*" | tee -a "$LOG"; }
pos() { timeout 6 ros2 topic echo --once /robot/amcl_pose --field pose.pose.position 2>/dev/null | grep -E '^[[:space:]]*(x|y):' | awk '{printf "%.2f ", $2}'; }
yaw() { timeout 6 ros2 topic echo --once /robot/amcl_pose --field pose.pose.orientation 2>/dev/null | grep -E '^[[:space:]]*(z|w):' | awk '{print $2}' | tr '\n' ' ' | python3 -c 'import sys,math; v=sys.stdin.read().split(); print("%.1f" % math.degrees(2*math.atan2(float(v[0]),float(v[1])))) if len(v)==2 else print("nan")'; }
edad() { python3 -c "import json,time; d=json.load(open('/tmp/latido.json')); print('%.1f' % (time.time()-d['$1']))" 2>/dev/null || echo 999; }
scan_ok() { python3 -c "import sys; sys.exit(0 if $(edad scan) < 3.0 else 1)"; }
T0=$(date +%s); dt() { echo "+$(( $(date +%s) - T0 ))s"; }

say "=== A → PUERTA → $DEST · cruce por holgura · desde $(pos)yaw $(yaw)° ==="
HU=$(timeout 6 ros2 param get /robot/local_costmap/local_costmap footprint_padding 2>/dev/null | sed 's/.*: //')
SA=$(timeout 6 ros2 param get /robot/behavior_server simulate_ahead_time 2>/dev/null | sed 's/.*: //')
say "salvaguardas: padding $HU · simulate_ahead $SA s"
pgrep -f "scan_latido.py$" >/dev/null || { say "arranco el latido del láser"; nohup python3 ~/scan_latido.py >/dev/null 2>&1 </dev/null & sleep 5; }
scan_ok || { say "SIN LÁSER: paro"; exit 1; }
python3 -c "import sys; sys.exit(0 if $(edad map_odom) < 5.0 else 1)" || { say "AMCL sin localizar: da la pose primero"; exit 1; }

say "--- 1) Nav2 hasta vano_antes"
bash ~/summit_ab.sh ir "$AX" "$AY" "$EJE" 120 > /tmp/abh_ir1.txt 2>&1
ST=$(grep -oE 'status: [A-Z]+' /tmp/abh_ir1.txt | tail -1)
say "    $(dt) $ST · en $(pos)yaw $(yaw)°"
[ "$ST" = "status: SUCCEEDED" ] || { say "no llegó a vano_antes: paro"; exit 1; }

# No hace falta alinear: cruza_vano.py detecta el vano por delante y se orienta solo con la pared, que es una recta
# ajustada sobre decenas de puntos y no depende de la pose del mapa.
say "--- 2) cruce por holgura (láser, sin AMCL ni costmap) · CSV $CSV"
python3 ~/cruza_vano.py --metros 1.35 --csv "$CSV" 2>&1 | tee -a "$LOG"
CR=${PIPESTATUS[0]}
say "    $(dt) en $(pos)yaw $(yaw)°"
[ "$CR" = 0 ] || { say "el cruce no llegó al final: Natan lo saca con el mando"; exit 1; }

say "--- 3) Nav2 hasta $DEST"
bash ~/summit_ab.sh ir "$DEST" 120 2>&1 | tee /tmp/abh_ir2.txt | grep -E "quedan=|resultado" | sed 's/^/    /' | tee -a "$LOG"
ST=$(grep -oE 'status: [A-Z]+' /tmp/abh_ir2.txt | tail -1)
read PX PY <<<"$(pos)"
DB=$(python3 -c "import math; print('%.2f' % math.hypot($DX-$PX, $DY-$PY))")
say "=== RESULTADO: $ST · $(dt) total · llegada ($PX, $PY) yaw $(yaw)° · a $DB m de $DEST ==="
say "    holgura mínima del cruce: $(python3 -c "
import csv
f=[r for r in csv.DictReader(open('$CSV'))]
i=[float(r['holg_izq']) for r in f if r['holg_izq'] not in ('','nan')]
d=[float(r['holg_der']) for r in f if r['holg_der'] not in ('','nan')]
print('izq %.0f mm · der %.0f mm · %d ciclos' % (1000*min(i), 1000*min(d), len(f))) if i and d else print('sin datos')" 2>/dev/null)"
