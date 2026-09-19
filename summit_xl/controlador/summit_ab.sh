#!/bin/bash
# Summit XL · A→B con Nav2 desde la terminal del robot, sin RViz. 11-sep-2026.
#
#   bash ~/summit_ab.sh pre                 # prechequeo: reloj, maestro+FIFO, SYNC, drives, vigilante, salud, láser, TF
#   bash ~/summit_ab.sh pose A|B|x y yaw°   # pose inicial de AMCL (robot colocado en A mirando yaw) y comprobación
#   bash ~/summit_ab.sh 1m                  # goal a 1 m al frente: la prueba mínima (test_1m.sh)
#   bash ~/summit_ab.sh marca NOMBRE        # guarda la pose actual del robot como marca (A, B, vano_…) en ~/ab/marcas.txt
#   bash ~/summit_ab.sh asigna-marcas MAPA  # tras guardar el SLAM: las marcas leídas durante el mapeo pasan a ser de MAPA
#   bash ~/summit_ab.sh plan                # sólo cálculo: hay camino a cada marca de ~/ab/marcas.txt
#   bash ~/summit_ab.sh ir puerta|B|A|x y yaw° [seg]   # NAVEGA y vigila: estado, pose, cmd_vel, Quick Stop, SYNC dobles
#
# Reglas: seta en la mano de quien esté delante; `ir` se corta con Ctrl-C (cancela el goal). Todo queda en ~/ab/.
set -u
set +u; source /opt/ros/humble/setup.bash 2>/dev/null; set -u
export ROS_DOMAIN_ID=40 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
mkdir -p ~/ab; TS=$(date +%Y%m%d_%H%M%S)
# Puntos con nombre. Desde el 17-sep salen de ~/ab/marcas.txt (medidos con `summit_ab.sh marca NOMBRE` sobre las
# marcas físicas del suelo, en el mapa que esté activo). Los valores de junio sólo valen con el mapa
# rbk_2026_06_26_16_22_47: si existe marcas.txt, NO se usan nunca.
MARCAS=~/ab/marcas.txt
punto() {
  if [ -f "$MARCAS" ] && [[ "$1" =~ ^[A-Za-z_]+[A-Za-z0-9_\']*$ ]]; then
    L=$(awk -v n="$1" '$1==n {print $2, $3, $4, $6}' "$MARCAS" | tail -1)
    if [ -n "$L" ]; then
      set -- $L; ACT="mapa=$(grep -m1 ^MAP_NAME ~/deploy/config/environment/ros_config.env 2>/dev/null | cut -d= -f2)"
      if [ "${4:-}" = "mapa=SLAM_EN_CURSO" ]; then
        echo "ERR ERR ERR"; echo "ERROR: marca medida con el SLAM sin nombre de mapa: tras guardar, bash ~/summit_ab.sh asigna-marcas NOMBRE_DEL_MAPA" >&2; return
      fi
      if [ -n "${4:-}" ] && [ "$4" != "$ACT" ]; then
        echo "ERR ERR ERR"; echo "ERROR: la marca se midió en $4 y el mapa activo es $ACT — no vale en este mapa" >&2; return
      fi
      echo "$1 $2 $3"; return
    fi
    echo "ERR ERR ERR"; echo "ERROR: '$1' no está en $MARCAS (marcas: $(awk '{printf "%s ", $1}' $MARCAS))" >&2; return
  fi
  case "$1" in
    A) echo "-2.897 0.102 -92.04"; echo "AVISO: A del MAPA DE JUNIO (no hay $MARCAS)" >&2;;
    B) echo "-8.832 -1.220 162.96"; echo "AVISO: B del MAPA DE JUNIO (no hay $MARCAS)" >&2;;
    puerta) echo "-7.577 -1.605 162.96"; echo "AVISO: puerta del MAPA DE JUNIO, nunca medida (no hay $MARCAS)" >&2;;
    *) echo "$1 $2 $3";;
  esac; }
quat() { python3 -c "import math,sys; y=math.radians(float(sys.argv[1])); print('%.6f %.6f'%(math.sin(y/2),math.cos(y/2)))" "$1"; }
pose_amcl() { timeout 6 ros2 topic echo --once /robot/amcl_pose 2>/dev/null | python3 -c '
import sys,re,math; t=sys.stdin.read()
m=re.search(r"position:\s*\n\s*x: (-?[\d.e-]+)\s*\n\s*y: (-?[\d.e-]+)",t); q=re.search(r"orientation:\s*\n\s*x: (-?[\d.e-]+)\s*\n\s*y: (-?[\d.e-]+)\s*\n\s*z: (-?[\d.e-]+)\s*\n\s*w: (-?[\d.e-]+)",t)
c=re.search(r"covariance:\s*\[([^\]]*)\]",t,re.S)
if not m: print("sin amcl_pose"); sys.exit()
x,y=float(m.group(1)),float(m.group(2)); yaw=math.degrees(2*math.atan2(float(q.group(3)),float(q.group(4)))) if q else float("nan")
cv=[float(v) for v in c.group(1).replace("\n","").split(",")] if c else []
sx=math.sqrt(cv[0]) if cv else float("nan"); sy=math.sqrt(cv[7]) if cv else float("nan"); syaw=math.degrees(math.sqrt(cv[35])) if cv else float("nan")
print("(%.2f, %.2f) yaw %.0f°  σ x %.2f m  y %.2f m  yaw %.0f°" % (x,y,yaw,sx,sy,syaw))'; }

case "${1:-}" in
pre)
  echo "=== prechequeo A→B · $(date '+%F %H:%M:%S') ===" | tee ~/ab/pre_$TS.txt
  {
  Y=$(date +%Y); [ "$Y" -ge 2026 ] && echo "reloj:       OK ($(date '+%F %H:%M'))" || echo "reloj:       MAL ($(date '+%F')) → ponerlo desde la estación ANTES de nada"
  PID=$(pgrep -f '[/ ]ros2_control_node( |$)' | head -1)
  if [ -n "$PID" ]; then FF=$(ps -L -o cls -p "$PID" | grep -c FF); echo "maestro:     PID $PID · hilos FIFO: $FF $( [ "$FF" = 1 ] && echo OK || echo '← revisar summit-prioriza-sync')"
  else echo "maestro:     NO HAY ros2_control_node → CAN mudo: segfault; docker logs + docker restart mobile-base-base-hw-1, esperar >60 s"; fi
  timeout 5 candump -L can0,080:7FF 2>/dev/null | awk '{split($1,t,/[()]/); if(p&&t[2]-p<0.005)d++; n++; p=t[2]} END{printf "SYNC 5 s:    %d tramas (250 = 50 Hz) · dobles %d %s\n", n, d, (d==0?"OK":"← MAL")}'
  # El statusword se ESCUCHA del TPDO1 de cada drive, no se pide por SDO: el maestro sondea a ~150 Hz por nodo
  # y una petición nuestra por el mismo canal SDO se pierde entre las suyas (medido el 11-sep).
  # COB-ID del TPDO1: nodo 1 = 0x4A1 (tipo 10, uno de cada diez SYNC); nodos 2,3,4 = 0x362/363/364 (tipo 1).
  timeout 3 candump -L can0,4A1:7FF,362:7FF,363:7FF,364:7FF > /tmp/ab_sw.txt 2>/dev/null
  for n in 1 2 3 4; do case $n in 1) id=4A1; w=front_left;; 2) id=362; w=back_left;; 3) id=363; w=front_right;; 4) id=364; w=back_right;; esac
    V=$(grep -m1 "$id#" /tmp/ab_sw.txt | sed -E "s/.*$id#//" | cut -c1-4)
    SW=$( [ -n "$V" ] && echo "0x${V:2:2}${V:0:2}" || echo "SIN TPDO" )
    case $SW in 0x0637) q="Operation Enabled";; 0x0628) q="seta pulsada (Switch On Disabled)";; 0x0617) q="QUICK STOP";; 0x0631) q="Ready to switch on";; *) q="";; esac
    printf "drive n%s %-12s statusword %s %s\n" $n "$w" "$SW" "$q"; done
  rm -f /tmp/ab_sw.txt
  B=$(timeout 8 ros2 topic echo --once /robot/battery_monitor/data 2>/dev/null | grep -E "^\s*(voltage|percentage|current|is_charging|charging):" | tr -d " " | tr "\n" " ")
  echo "batería:     ${B:-sin lectura de /robot/battery_monitor/data}"
  echo "vigilante:   $(XDG_RUNTIME_DIR=/run/user/$(id -u) systemctl --user is-active vigia-amc.service 2>/dev/null || pgrep -f summit_vigia_amc >/dev/null && echo "activo (proceso)") · prioridad: $(systemctl is-active summit-prioriza-sync.service)"
  U=$(docker ps --format '{{.Names}} {{.Status}}' | grep -iE 'unhealthy|restarting|exited' | tr '\n' ';'); echo "contenedores: $(docker ps -q | wc -l) arriba · ${U:-todos healthy} (unhealthy intermitente = healthcheck de 3 s, no es fallo)"
  timeout 15 ros2 topic list >/dev/null 2>&1     # calienta el descubrimiento DDS
  # `ros2 topic hz` es poco fiable con este DDS (da falsos "sin datos", medido el 11-sep): se cuentan mensajes.
  cuenta() { local seg=$1 topic=$2 extra=${3:-}; local n
    n=$(timeout $seg ros2 topic echo $extra --field header.stamp.sec "$topic" 2>/dev/null | grep -c "^[0-9]")   # sin los separadores "---"
    if [ "${n:-0}" -gt 0 ]; then python3 -c "print('%.1f Hz' % ($n/$seg))"; else echo "SIN DATOS"; fi; }
  echo "láser:       $(cuenta 5 /robot/top_laser/scan '--qos-reliability best_effort')"
  echo "odom:        $(cuenta 3 /robot/robotnik_base_control/odom)"
  # La TF se lee de /tf directamente: tf2_echo necesita llenar su buffer y daba falsos negativos.
  TFS=$(timeout 4 ros2 topic echo /tf --field transforms 2>/dev/null | grep -oE "child_frame_id='[a-z_]+'" | sed "s/child_frame_id=//;s/'//g" | sort -u | tr '\n' ' ')
  echo "TF en vivo:  ${TFS:-NINGUNA}"
  case "$TFS" in *robot_odom*) echo "             map→odom SÍ: AMCL está localizando";; *) echo "             map→odom NO: AMCL sin localizar → bash ~/summit_ab.sh pose A";; esac
  echo "amcl_pose:   $(pose_amcl)"
  echo "nav2 vivos:  $(timeout 8 ros2 node list 2>/dev/null | grep -cE 'amcl|bt_navigator|controller_server|planner_server|behavior_server|map_server') de 6 (amcl bt controller planner behavior map_server)"
  echo "mando:       joy $(timeout 5 ros2 topic hz --window 5 /robot/joy 2>&1 | grep -m1 -oE 'average rate: [0-9.]+' || echo 'SIN DATOS (¿mando conectado?)')"
  } | tee -a ~/ab/pre_$TS.txt ;;
pose)
  shift; read X Y YAW <<<"$(punto "$@")"; [ "$X" = ERR ] && exit 1; echo "=== pose inicial ($X, $Y, ${YAW}°) · el robot debe estar FÍSICAMENTE ahí ==="
  bash ~/summit_initialpose.sh "$X" "$Y" "$YAW" 2>&1 | tail -3; sleep 4
  echo "AMCL ahora:  $(pose_amcl)"; echo "(σ x,y < 0,3 m y yaw < 10° = localizado; si no, mover el robot 1 m adelante y atrás con el mando y repetir la lectura: bash ~/summit_ab.sh pre)" ;;
1m)
  # No se usa test_1m.sh: lee mal el cuaternión (toma 6 campos donde hay 7, así que confunde
  # orientation.y/z con z/w) y calcula un rumbo falso — el 11-sep mandó el objetivo 90° girado.
  read RX RY RYAW <<<"$(timeout 8 ros2 topic echo --once /robot/amcl_pose 2>/dev/null | python3 -c '
import sys, re, math
t = sys.stdin.read()
m = re.search(r"position:\s*\n\s*x: (-?[\d.e-]+)\s*\n\s*y: (-?[\d.e-]+)", t)
q = re.search(r"orientation:\s*\n\s*x: (-?[\d.e-]+)\s*\n\s*y: (-?[\d.e-]+)\s*\n\s*z: (-?[\d.e-]+)\s*\n\s*w: (-?[\d.e-]+)", t)
if not (m and q): print("0 0 nan"); sys.exit()
print("%.3f %.3f %.4f" % (float(m.group(1)), float(m.group(2)), 2*math.atan2(float(q.group(3)), float(q.group(4)))))')"
  [ "$RYAW" = nan ] && { echo "sin amcl_pose: no sé dónde está el robot"; exit 1; }
  TX=$(python3 -c "import math; print(round($RX + 1.0*math.cos($RYAW), 3))")
  TY=$(python3 -c "import math; print(round($RY + 1.0*math.sin($RYAW), 3))")
  echo "robot en ($RX, $RY) rumbo $(python3 -c "import math; print(round(math.degrees($RYAW),1))")° · objetivo 1 m AL FRENTE: ($TX, $TY)"
  { echo "=== ¿hay plan? ==="
    timeout 30 ros2 action send_goal /robot/compute_path_to_pose nav2_msgs/action/ComputePathToPose \
      "{goal: {header: {frame_id: robot_map}, pose: {position: {x: $TX, y: $TY}}}, use_start: false}" 2>&1 | grep -E "status|rejected"
    echo "=== navegando ==="
    timeout 90 ros2 action send_goal --feedback /robot/navigate_to_pose nav2_msgs/action/NavigateToPose \
      "{pose: {header: {frame_id: robot_map}, pose: {position: {x: $TX, y: $TY}}}}" 2>&1 | grep -E "distance_remaining|number_of_recoveries|status|rejected" | tail -6
    echo "=== llegada: $(pose_amcl)  (objetivo $TX, $TY)"; } | tee ~/ab/1m_$TS.txt ;;
marca)
  # Lee la pose del robot en el mapa ACTIVO por TF (vale con el SLAM encendido y con AMCL) y la guarda con nombre.
  N=${2:?uso: summit_ab.sh marca NOMBRE   (p. ej. A, B, vano_antes, vano_centro, vano_despues)}
  read MX MY MYAW <<<"$(python3 - <<'PY'
import math, time, rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = Node("lee_marca_summit"); b = Buffer(); TransformListener(b, n)
t0 = time.time(); r = None
while time.time() - t0 < 10 and r is None:
    rclpy.spin_once(n, timeout_sec=0.2)
    for m in ("robot_map", "robot/map", "map"):
        for f in ("robot_base_footprint", "robot/base_footprint", "base_footprint"):
            try:
                tr = b.lookup_transform(m, f, rclpy.time.Time()); q = tr.transform.rotation
                r = (tr.transform.translation.x, tr.transform.translation.y, math.degrees(2 * math.atan2(q.z, q.w))); break
            except Exception: pass
        if r: break
print("%.3f %.3f %.1f" % r if r else "ERR ERR ERR")
n.destroy_node(); rclpy.shutdown()
PY
)"
  [ "$MX" = ERR ] && { echo "sin TF mapa→robot: ¿SLAM o AMCL corriendo? (bash ~/mapea.sh estado)"; exit 1; }
  # Con el SLAM en marcha el mapa todavía no tiene nombre: se etiqueta SLAM_EN_CURSO y `asigna-marcas` le pone el
  # nombre al guardarlo. Sin SLAM, la marca pertenece al mapa activo de ros_config.env.
  if docker ps --format '{{.Names}}' 2>/dev/null | grep -qi creator; then ETQ="mapa=SLAM_EN_CURSO"
  else ETQ="mapa=$(grep -m1 ^MAP_NAME ~/deploy/config/environment/ros_config.env | cut -d= -f2)"; fi
  touch "$MARCAS"; grep -v "^$N " "$MARCAS" > "$MARCAS.tmp"; echo "$N $MX $MY $MYAW $(date '+%F_%H:%M:%S') $ETQ" >> "$MARCAS.tmp"; mv "$MARCAS.tmp" "$MARCAS"
  echo "marca $N = ($MX, $MY, ${MYAW}°) guardada en $MARCAS"; echo "--- marcas:"; cat "$MARCAS" ;;
asigna-marcas)
  # Tras `mapea.sh guarda` y `mapea.sh vuelve NOMBRE`: las marcas leídas durante el SLAM pasan a ser de NOMBRE.
  NM=${2:?uso: summit_ab.sh asigna-marcas NOMBRE_DEL_MAPA_GUARDADO}
  [ -d ~/deploy/config/maps/$NM ] || { echo "no existe ~/deploy/config/maps/$NM"; exit 1; }
  [ -f "$MARCAS" ] || { echo "no hay $MARCAS"; exit 1; }
  cp -a "$MARCAS" "$MARCAS.bak_$TS"; sed -i "s/mapa=SLAM_EN_CURSO/mapa=$NM/" "$MARCAS"; echo "marcas asignadas a $NM:"; cat "$MARCAS" ;;
plan)
  # Sólo cálculo: ¿hay camino desde donde está el robot a cada marca? No mueve el robot.
  [ -f "$MARCAS" ] || { echo "no hay $MARCAS: primero summit_ab.sh marca A / marca B"; exit 1; }
  while read N X Y YAW _; do
    r=$(timeout 30 ros2 action send_goal /robot/compute_path_to_pose nav2_msgs/action/ComputePathToPose \
      "{goal: {header: {frame_id: robot_map}, pose: {position: {x: $X, y: $Y}}}, use_start: false}" 2>&1 | grep -oE "status: [A-Z]+|rejected" | head -1)
    printf "   %-14s (%s, %s): %s\n" "$N" "$X" "$Y" "${r:-sin respuesta}"
  done < "$MARCAS" | tee ~/ab/plan_$TS.txt ;;
ir)
  shift; NOMBRE=$1; read X Y YAW <<<"$(punto "$@")"; [ "$X" = ERR ] && exit 1; read QZ QW <<<"$(quat "$YAW")"; SEG=${4:-${2:-180}}; [ "$NOMBRE" = "$X" ] && SEG=${4:-180}
  OUT=~/ab/ir_${NOMBRE}_$TS; mkdir -p "$OUT"
  echo "=== NAVEGAR a $NOMBRE ($X, $Y, ${YAW}°) · máx $SEG s · $(date '+%H:%M:%S') · $OUT ===" | tee "$OUT/resumen.txt"
  echo "desde:       $(pose_amcl)" | tee -a "$OUT/resumen.txt"
  echo "batería:     $(timeout 8 ros2 topic echo --once /robot/battery_monitor/data 2>/dev/null | grep -E "^\s*(voltage|percentage|is_charging|charging):" | tr -d " " | tr "\n" " ")" | tee -a "$OUT/resumen.txt"
  bash ~/summit_captura_persistente.sh marca "A→B: goal $NOMBRE" >/dev/null 2>&1
  timeout $((SEG+5)) candump -L can0,080:7FF > "$OUT/sync.log" 2>/dev/null &
  timeout $((SEG+5)) candump -L can0,360:7FC > "$OUT/statusword.log" 2>/dev/null &
  timeout $((SEG+5)) ros2 topic echo --csv --field pose.pose.position /robot/amcl_pose > "$OUT/amcl.csv" 2>/dev/null &
  timeout $((SEG+5)) ros2 topic echo --csv /robot/cmd_vel_nav > "$OUT/cmd_vel_nav.csv" 2>/dev/null &
  timeout $((SEG+5)) ros2 topic echo --csv --field effort /robot/joint_states > "$OUT/effort.csv" 2>/dev/null &
  timeout $((SEG+5)) ros2 topic echo /robot/navigate_to_pose/_action/status 2>/dev/null | grep --line-buffered -E "status:" | awk '{print strftime("%H:%M:%S"), $NF; fflush()}' > "$OUT/status.txt" &
  T0=$(date +%s); docker logs --since 1s $(docker ps -qf name=nav-loc-navigation) >/dev/null 2>&1
  timeout $SEG ros2 action send_goal --feedback /robot/navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: robot_map}, pose: {position: {x: $X, y: $Y}, orientation: {z: $QZ, w: $QW}}}}" > "$OUT/goal.txt" 2>&1 &
  GOAL=$!
  # Ctrl-C: SIGINT al cliente de la acción, que es quien cancela el goal en Nav2 (nunca mandar otro goal vacío)
  trap 'echo; echo "Ctrl-C: cancelando el goal"; kill -INT $GOAL 2>/dev/null; sleep 2' INT
  while kill -0 $GOAL 2>/dev/null; do sleep 5
    D=$(grep -oE 'distance_remaining: [0-9.]+' "$OUT/goal.txt" | tail -1 | awk '{printf "%.2f m", $2}'); R=$(grep -oE 'number_of_recoveries: [0-9]+' "$OUT/goal.txt" | tail -1 | awk '{print $2}')
    ST=$(tail -1 "$OUT/status.txt" 2>/dev/null | awk '{print $2}'); P=$(tail -1 "$OUT/amcl.csv" 2>/dev/null | awk -F, '{printf "(%.2f, %.2f)", $1, $2}')
    QS=$(awk -F'#' '{d=$2; if (substr(d,3,2) substr(d,1,2)=="0617") q++} END{print q+0}' "$OUT/statusword.log" 2>/dev/null)
    echo "$(date +%H:%M:%S) +$(( $(date +%s)-T0 ))s  estado=${ST:-?}  quedan=${D:-?}  recuperaciones=${R:-0}  pose=${P:-?}  quickstop=$QS" | tee -a "$OUT/resumen.txt"
  done; wait $GOAL 2>/dev/null; sleep 2; trap - INT
  {
  echo "--- resultado: $(grep -oE 'Goal (finished|accepted|rejected)[^\n]*|status: [A-Z]+' "$OUT/goal.txt" | tail -2 | tr '\n' ' ')"
  echo "--- transiciones: $(awk '{print $2}' "$OUT/status.txt" | uniq | sed 's/1/ACCEPTED/;s/2/EXECUTING/;s/4/SUCCEEDED/;s/5/CANCELED/;s/6/ABORTED/' | tr '\n' '>')"
  echo "--- llegada:     $(pose_amcl)   (goal: $X, $Y, ${YAW}°)"
  awk '{split($1,t,/[()]/); if(p&&t[2]-p<0.005)d++; n++; p=t[2]} END{printf "--- SYNC: %d tramas · dobles %d\n", n, d}' "$OUT/sync.log"
  echo "--- Quick Stop (0x0617) por nodo: $(awk -F'#' '{d=$2; if (substr(d,3,2) substr(d,1,2)=="0617") print substr($1,length($1)-2)}' "$OUT/statusword.log" | sort | uniq -c | tr '\n' ' ' | sed 's/^$/ninguno/')"
  echo "--- recuperaciones (log de navigation): $(docker logs --since ${T0}s $(docker ps -qf name=nav-loc-navigation) 2>&1 | grep -cE 'Running Spin|Running BackUp|Running Wait|recovery')"
  echo "--- esfuerzo por rueda (mediana |e| / máx): $(python3 -c '
import re,sys,statistics
e=[]
for l in open(sys.argv[1]):
    n=re.findall(r"-?\d+\.\d+(?:e-?\d+)?|-?\d+", l)
    if len(n)>=4: e.append([abs(float(x)) for x in n[:4]])
if e:
    c=list(zip(*e)); print("  ".join("%s %.1f/%.1f"%(k,statistics.median(v),max(v)) for k,v in zip(("FL","BL","FR","BR"),c)))
else: print("sin joint_states")' "$OUT/effort.csv")"
  } | tee -a "$OUT/resumen.txt" ;;
*) sed -n 2,12p "$0" ;;
esac
