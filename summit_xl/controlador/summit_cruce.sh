#!/bin/bash
# Summit XL · travesía completa en un comando: A → alinear en vano_antes → cruzar recto → Nav2 hasta el destino.
# 17-sep-2026. Escrito con los cruces del día: Nav2 deja ~5° de error de rumbo y el vano de 742 mm sólo perdona
# ~2°; el cruce recto se para en el marco y hay que relanzarlo desde dentro.
#
# 17-sep, 18:20: el robot chocó con una caja tras cruzar. Causa: el láser llevaba muerto todo el tramo recto (2,0 m),
# y además yo había bajado la huella y simulate_ahead. Desde entonces:
#   · GUARDA DE LÁSER: todo movimiento (Nav2 y recto) se vigila; si el scan calla ~5 s se cancela el objetivo y se para.
#   · el tramo recto acaba 15 cm después de vano_despues, no 40.
#   · el script se niega a correr con las salvaguardas bajadas (huella < 0,30 m o simulate_ahead < 1 s), salvo SIN_RED=1.
#
#   bash ~/summit_cruce.sh [destino] [eje°]      # destino: B por defecto (el mismo que el G1), o vano_despues, o "x y yaw"
#   Requisitos: robot localizado (pose A dada), Natan con la seta a un lado del marco.
set -u
set +u; source /opt/ros/humble/setup.bash; set -u
export ROS_DOMAIN_ID=39 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# 18-sep: -96,6° era el eje de las MARCAS (dos poses del robot), ~8° girado respecto a la normal de la puerta.
DEST=${1:-B}; EJE=${2:--89.0}; TS=$(date +%Y%m%d_%H%M%S); LOG=~/ab/cruce_$TS.txt
M=~/ab/marcas.txt; read AX AY _ <<<"$(awk '$1=="vano_antes"{print $2,$3,$4}' $M)"; read PDX PDY _ <<<"$(awk '$1=="vano_despues"{print $2,$3,$4}' $M)"; read DX DY DYAW _ <<<"$(awk -v n="$DEST" '$1==n{print $2,$3,$4}' $M)"
[ -n "${DX:-}" ] || { DX=$1; DY=$2; DYAW=$3; EJE=${4:--89.0}; }
say() { echo "$(date +%H:%M:%S) $*" | tee -a "$LOG"; }
yaw() { timeout 6 ros2 topic echo --once /robot/amcl_pose --field pose.pose.orientation 2>/dev/null | grep -E '^\s*(z|w):' | awk '{print $2}' | tr '\n' ' ' | python3 -c 'import sys,math; z,w=[float(x) for x in sys.stdin.read().split()]; print("%.1f" % math.degrees(2*math.atan2(z,w)))'; }
pos() { timeout 6 ros2 topic echo --once /robot/amcl_pose --field pose.pose.position 2>/dev/null | grep -E '^\s*(x|y):' | awk '{printf "%.2f ", $2}'; }
lateral() { # cm del centro del robot respecto a la recta vano_antes→vano_despues (+ = izquierda del sentido de cruce)
  read PX PY <<<"$(pos)"; python3 -c "
import math; ax,ay,bx,by,px,py=$AX,$AY,$PDX,$PDY,$PX,$PY
L=math.hypot(bx-ax,by-ay); ux,uy=(bx-ax)/L,(by-ay)/L
print('%+.1f cm lateral · %.2f m a lo largo del eje' % (100*(-(px-ax)*uy+(py-ay)*ux), (px-ax)*ux+(py-ay)*uy))"; }
T0=$(date +%s); dt() { echo "+$(( $(date +%s) - T0 ))s"; }
# El scan se lee del latido de ~/scan_latido.py (nodo persistente): una sonda `ros2 topic echo` nueva cada vez daba 0 con el
# scan fluyendo (medido 18:50). Edad del último scan < 3 s = vivo.
edad() { python3 -c "import json,time; d=json.load(open('/tmp/latido.json')); print('%.1f' % (time.time()-d['$1']))" 2>/dev/null || echo 999; }
scan_ok() { python3 -c "import sys; sys.exit(0 if $(edad scan) < 3.0 else 1)"; }
param() { timeout 6 ros2 param get "$1" "$2" 2>/dev/null | sed 's/^.*value is: //'; }

# --- Guarda de láser -------------------------------------------------------------------------------------------
# vigilado CMD… : corre CMD en su propio grupo de procesos y sondea el scan cada ~2 s. Dos sondas mudas seguidas
# (≈5 s, 25 cm a 5 cm/s) → cancela TODOS los objetivos de Nav2 (navigate_to_pose, drive_on_heading, spin) y mata
# el grupo. Deja LASER_MUDO=1 para quien lo llame.
LASER_MUDO=0
cancela_todo() {
  for a in navigate_to_pose drive_on_heading spin; do   # back_up no expone cancel_goal en este Nav2
    timeout 5 ros2 service call /robot/$a/_action/cancel_goal action_msgs/srv/CancelGoal '{}' >/dev/null 2>&1
  done
}
vigilado() {
  local out=$1; shift; LASER_MUDO=0
  setsid "$@" > "$out" 2>&1 &
  local pid=$! miss=0
  while kill -0 $pid 2>/dev/null; do
    if scan_ok; then miss=0; else miss=$((miss+1)); fi
    if [ $miss -ge 2 ]; then
      say "    !!! LÁSER MUDO ~5 s en movimiento: cancelo objetivos y paro"
      cancela_todo; kill -INT -- -$pid 2>/dev/null; sleep 2; kill -- -$pid 2>/dev/null; cancela_todo
      LASER_MUDO=1; break
    fi
  done
  wait $pid 2>/dev/null; return 0
}
recto() { # $1 metros → imprime metros recorridos y estado (LASER_MUDO si lo paró la guarda)
  local out=/tmp/cruce_recto_$$.txt
  vigilado "$out" timeout 90 ros2 action send_goal --feedback /robot/drive_on_heading nav2_msgs/action/DriveOnHeading "{target: {x: $1, y: 0.0, z: 0.0}, speed: 0.05, time_allowance: {sec: 60, nanosec: 0}}"
  awk -v m=$LASER_MUDO '/distance_traveled/{d=$2} /status:/{s=$NF} END{if(m==1)s="LASER_MUDO"; printf "%.2f %s\n", d+0, s}' "$out"
}

say "=== CRUCE COMPLETO → $DEST ($DX, $DY, $DYAW°) · eje $EJE° · desde $(pos)yaw $(yaw)° ==="
# --- Salvaguardas: se comprueban, no se bajan ------------------------------------------------------------------
HUELLA=$(param /robot/local_costmap/local_costmap footprint | python3 -c "import sys,ast; print('%.4f' % max(abs(p[0]) for p in ast.literal_eval(sys.stdin.read().strip() or '[[0,0]]')))" 2>/dev/null || echo 0)
SIMA=$(param /robot/behavior_server simulate_ahead_time); PADL=$(param /robot/local_costmap/local_costmap footprint_padding)
say "salvaguardas: huella ±$HUELLA m · padding local $PADL · simulate_ahead $SIMA s"
python3 -c "import sys; sys.exit(0 if float('${HUELLA:-0}')>=0.30 and float('${SIMA:-0}')>=1.0 else 1)" \
  || { [ "${SIN_RED:-0}" = 1 ] && say "    AVISO: salvaguardas bajadas y SIN_RED=1: sigo bajo tu responsabilidad" || { say "salvaguardas bajadas: NO corro (SIN_RED=1 para forzar)"; exit 1; }; }
pgrep -f "scan_latido.py$" >/dev/null || { say "sin latido: arranco ~/scan_latido.py"; nohup python3 ~/scan_latido.py >/dev/null 2>&1 </dev/null & sleep 4; }
say "latido: $(python3 ~/scan_latido.py edad)"
python3 -c "import sys; sys.exit(0 if $(edad map_odom) < 5.0 else 1)" || { say "AMCL sin localizar (map→odom con más de 5 s): da la pose primero (bash ~/summit_ab.sh pose A)"; exit 1; }
scan_ok || { docker restart data-process-pointcloud-2-scan-1 >/dev/null; say "láser mudo: conversor reiniciado"; sleep 10; }
scan_ok || { say "SIN LÁSER: paro"; exit 1; }

say "--- 1) Nav2 hasta vano_antes (vigilado)"
vigilado /tmp/cruce_ir1.txt bash ~/summit_ab.sh ir "$AX" "$AY" "$EJE" 120
ST=$(grep -oE 'status: [A-Z]+' /tmp/cruce_ir1.txt | tail -1); [ $LASER_MUDO = 1 ] && ST="status: LASER_MUDO"
say "    $(dt) $ST · en $(pos)yaw $(yaw)° · $(lateral) · recuperaciones $(grep -oE 'recuperaciones=[0-9]+' /tmp/cruce_ir1.txt | tail -1 | cut -d= -f2)"
[ "$ST" = "status: SUCCEEDED" ] || { say "no llegó a vano_antes: paro"; exit 1; }

# Alineación: el Spin de Nav2 se pasa ~2x en moqueta (medido: pedí 5,6° y giró 10,7°), así que se pide la MITAD
# del error y se comprueba de verdad tras cada giro. alinear <eje> <tolerancia°> <intentos>
# 18-sep: el Spin del behavior_server NO puede alinear. Arranca a min_rotational_vel 0,4 rad/s y la base frena a
# 0,5 rad/s²: la frenada sola son 0,16 rad ≈ 9°, así que cualquier giro se pasa de ahí y el bucle de media ganancia
# oscilaba entre ±5° sin converger. Con cmd_vel propio a 0,03–0,15 rad/s converge a ~1° a la primera (banco).
# Alineación en lazo cerrado sobre la ODOMETRÍA, no sobre AMCL.
#
# Medido el 18-sep en la primera travesía buena: las tres alineaciones agotaron su tope de 40 s sin converger y se
# comieron 119 s de los 245 de la travesía. La causa es que AMCL sólo publica cuando el robot se mueve más de
# `update_min_a` = 0,1 rad ≈ 5,7°: pidiéndole que corrija 1,4° el lazo no puede VER lo que hace, corrige a ciegas y
# se pasa al otro lado (se vio saltar de −87,6° a −95,6°). La odometría va a 50 Hz y ve cualquier giro; su deriva en
# los pocos segundos que dura esto es despreciable. Se toma el desfase con AMCL una vez, al empezar, y se controla
# sobre odometría. Y tras cada ráfaga va un CERO explícito: sin él la base sigue con el último mando.
yaw_odom() { timeout 3 ros2 topic echo --once /robot/robotnik_base_control/odom --field pose.pose.orientation 2>/dev/null | grep -E '^[[:space:]]*(z|w):' | awk '{print $2}' | tr '\n' ' ' | python3 -c 'import sys,math; v=sys.stdin.read().split(); print("%.2f" % math.degrees(2*math.atan2(float(v[0]),float(v[1])))) if len(v)==2 else print("nan")'; }
para_base() { timeout 3 ros2 topic pub --once -w 0 /robot/move_base/cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1; }
alinear() {
  local eje=$1 tol=$2 t0=$(date +%s) YA YO OFF Y E W n=0
  YA=$(yaw); YO=$(yaw_odom)
  case "$YA$YO" in *nan*) say "    sin rumbo fiable: no alineo"; return 1;; esac
  OFF=$(python3 -c "print('%.2f' % (($YA)-($YO)))")          # desfase AMCL - odometría, tomado una vez
  while [ $(( $(date +%s) - t0 )) -lt 25 ]; do
    YO=$(yaw_odom); Y=$(python3 -c "print('%.2f' % ((($YO)+($OFF)+180)%360-180))")
    E=$(python3 -c "print('%.2f' % ((($eje)-($Y)+180)%360-180))")
    n=$((n+1))
    python3 -c "import sys; sys.exit(0 if abs($E) <= $tol else 1)" && { para_base; say "    rumbo $Y° · error $E° · alineado en ${n} pasos, $(( $(date +%s) - t0 ))s"; return 0; }
    scan_ok || { para_base; say "    láser mudo al alinear: paro"; return 1; }
    W=$(python3 -c "import math; e=math.radians($E); print('%.3f' % (math.copysign(min(0.20, max(0.05, abs(e)*1.2)), e)))")
    timeout 2 ros2 topic pub -r 20 -t 6 /robot/move_base/cmd_vel geometry_msgs/msg/Twist "{angular: {z: $W}}" >/dev/null 2>&1
    para_base
  done
  para_base; YO=$(yaw_odom); Y=$(python3 -c "print('%.2f' % ((($YO)+($OFF)+180)%360-180))")
  E=$(python3 -c "print('%.2f' % ((($eje)-($Y)+180)%360-180))"); say "    rumbo $Y° · error $E° (se agotó el tiempo, $n pasos)"
  python3 -c "import sys; sys.exit(0 if abs($E) <= $tol else 1)"
}
say "--- 2) alinear al eje por cmd_vel (tolerancia 1,0°)"
alinear "$EJE" 1.0 || say "    AVISO: no alineado del todo; sigo"
say "    $(dt) alineado a $(yaw)° · $(lateral)"

# El tramo recto va de vano_antes a vano_despues MÁS 15 cm: lo justo para salir del marco (desde dentro Nav2 no
# planifica y AMCL va retrasado). Antes eran 40 cm y con el láser muerto el robot siguió hasta una caja.
VANO=$(python3 -c "import math; print('%.2f' % math.hypot($PDX-$AX, $PDY-$AY))")
# 18-sep, perfil de holgura medido en el banco: a 0,75 m PASADO el centro del vano la holgura es CERO (mobiliario).
# El recto acaba en +0,55 del centro, no en +0,96 como antes: aquello terminaba 20 cm dentro del mueble.
LARGO=$(python3 -c "print('%.2f' % ($VANO/2 + 0.55))")
say "--- 3) cruce recto $LARGO m a 5 cm/s, vigilado (relanza desde dentro si se para)"
# Cuenta como cruzado en cuanto el recorrido acumulado pasa de vano_antes a vano_despues (menos 15 cm).
PUERTA=$(python3 -c "print('%.2f' % ($VANO/2 + 0.20))")
REST=$LARGO; INT=0; ACUM=0
for i in 1 2 3; do
  read D S <<<"$(recto $REST)"; ACUM=$(python3 -c "print('%.2f' % ($ACUM + $D))"); say "    $(dt) intento $i: $D m · $S · acumulado $ACUM m · $(lateral) · rumbo $(yaw)°"
  [ "$S" = LASER_MUDO ] && { say "cruce parado por la guarda de láser: Natan lo saca con el mando"; exit 1; }
  [ "$S" = SUCCEEDED ] && break
  python3 -c "import sys; sys.exit(0 if $ACUM >= $PUERTA else 1)" && { S=SUCCEEDED; say "    ya pasó vano_despues ($PUERTA m): cruce hecho aunque el recto abortara"; break; }
  REST=$(python3 -c "print(max(0.0, round($REST - $D, 2)))"); INT=$((INT+1))
  python3 -c "import sys; sys.exit(0 if $REST > 0.05 else 1)" || break
  say "    corrijo el rumbo dentro del marco antes de relanzar"; alinear "$EJE" 1.0 || true
  sleep 1
done
say "    cruce: $S · relances $INT · en $(pos)"
[ "$S" = SUCCEEDED ] || { say "no cruzó del todo: Natan lo saca con el mando y se relanza el paso 4 a mano (bash ~/summit_ab.sh ir $DEST)"; exit 1; }
sleep 3   # que AMCL se ponga al día fuera del marco antes de pedir un plan

# Tras el cruce el robot está en el marco y AMCL suele ir retrasado: Nav2 no puede planificar desde la banda
# inscrita. Si el destino está a menos de 0,6 m, se remata RECTO (vigilado); si no, Nav2 (vigilado).
read PX PY <<<"$(pos)"; DIST=$(python3 -c "import math; print('%.2f' % math.hypot($DX-$PX, $DY-$PY))")
if python3 -c "import sys; sys.exit(0 if $DIST < 0.6 else 1)"; then
  say "--- 4) destino a $DIST m: remato recto"
  read D S <<<"$(recto $DIST)"; ST="status: $S"; say "    $D m · $S"
else
  say "--- 4) Nav2 hasta $DEST ($DIST m), vigilado"
  vigilado /tmp/cruce_ir2.txt bash ~/summit_ab.sh ir "$DEST" 120
  grep -E "quedan=|resultado" /tmp/cruce_ir2.txt | sed "s/^/    /" | tee -a "$LOG"
  ST=$(grep -oE 'status: [A-Z]+' /tmp/cruce_ir2.txt | tail -1); [ $LASER_MUDO = 1 ] && ST="status: LASER_MUDO"
fi
QS=$(cat /tmp/cruce_ir1.txt /tmp/cruce_ir2.txt 2>/dev/null | grep -oE 'quickstop=[0-9]+' | sort -u | tail -1)
read PX PY <<<"$(pos)"; DB=$(python3 -c "import math; print('%.2f' % math.hypot($DX-$PX, $DY-$PY))")
DOB=$(cat /tmp/cruce_ir1.txt /tmp/cruce_ir2.txt 2>/dev/null | grep -oE 'dobles [0-9]+' | awk '{s+=$2} END{print s+0}')
say "=== RESULTADO: $ST · $(dt) total · llegada $(pos)yaw $(yaw)° · a $DB m de $DEST · relances del cruce $INT · SYNC dobles $DOB · $QS ==="
say "    ¿está físicamente sobre la marca $DEST? (AMCL puede ir retrasado tras la puerta) · log $LOG"
