#!/bin/bash
# Simulador 2D del Summit XL sobre el mapa real, con Nav2 (parámetros reales del 17-sep) y RViz, en gpuedge.
# 17-sep-2026, noche. Dominio DDS 40 (el robot va en el 39).
#
#   bash ~/summit_sim2d/summit_sim.sh arranca [--verdad] [--puerta X,Y,YAW,ANCHO] [--pose "x y yaw"]
#   bash ~/summit_sim2d/summit_sim.sh estado | para | pose NOMBRE|"x y yaw"
#
#   --verdad : sin AMCL; el simulador publica map→odom y /robot/amcl_pose exactos (para la pregunta geométrica pura).
#   --puerta : redibuja las jambas a esa anchura (m) para medir a qué hueco pasa cada huella.
#   Después: bash ~/summit_sim2d/summit_cruce.sh   (el mismo script que el robot; dominio 40)
set -u
D=~/summit_sim2d; L=$D/log; MAPA=${SIM_MAPA:-~/Desktop/extraccion_20260917/mapa/rbk_2026_09_17_16_23_20.yaml}
MAPA=$(eval echo "$MAPA")   # SIM_MAPA=~/summit_sim2d/mapa_fino/mapa_25mm.yaml para el mapa fino
DUMPS=~/Desktop/extraccion_20260917/params; PARAMS=$D/nav2_sim.yaml
set +u; source /opt/ros/humble/setup.bash; set -u
export ROS_DOMAIN_ID=40 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///tmp/cyclone_estacion.xml
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority
ACC=${1:-estado}; shift || true
VERDAD=0; PUERTA=""; RUIDO=""; ATIP=""; FILTRO=0; POSE="$(awk '$1=="A"{print $2,$3,$4}' ~/ab/marcas.txt)"
while [ $# -gt 0 ]; do case "$1" in --verdad) VERDAD=1;; --puerta) PUERTA=$2; shift;; --ruido) RUIDO=$2; shift;; --atipicos) ATIP=$2; shift;; --filtro) FILTRO=1;; --pose) POSE=$2; shift;; *) POSE="$1";; esac; shift; done
mata() { for p in $(pgrep -f "$1"); do kill $p 2>/dev/null; done; }
activo() { timeout 5 ros2 lifecycle get /robot/$1 2>/dev/null | grep -q "active \[3\]"; }

case "$ACC" in
para)
  # Todo lo de Nav2 vive bajo /opt/ros/humble/lib/nav2_*: un solo patrón, en vez de una lista que dejaba zombis
  # (el 17-sep quedaron tres waypoint_follower y dos gestores de ciclo de vida, y el bringup se quedaba esperando).
  mata "/opt/ros/humble/lib/[n]av2_"; mata "ros2 [l]aunch nav2"
  mata "summit_sim2d/[s]im2d.py"; mata "[r]viz2 -d $D/summit_sim.rviz"; mata "summit_sim2d/[s]can_latido.py"; mata "summit_sim2d/[f]iltra_scan.py"
  sleep 2; echo "parado · quedan: nav2 $(pgrep -fc "/opt/ros/humble/lib/[n]av2_") · sim $(pgrep -fc "summit_sim2d/[s]im2d.py") · rviz $(pgrep -fc "[r]viz2 -d")";;
arranca)
  bash $0 para >/dev/null; mkdir -p $L; TS=$(date +%H%M%S)
  [ -f $PARAMS ] || python3 $D/genera_params_sim.py $DUMPS $MAPA $PARAMS
  read PX PY PYAW <<<"$POSE"
  EXTRA=""; [ $VERDAD = 1 ] && EXTRA="--tf-verdad"; [ -n "$PUERTA" ] && EXTRA="$EXTRA --puerta $PUERTA"; [ -n "$RUIDO" ] && EXTRA="$EXTRA --ruido $RUIDO"; [ -n "$ATIP" ] && EXTRA="$EXTRA --atipicos $ATIP"
  if [ $FILTRO = 1 ]; then EXTRA="$EXTRA --topic-scan /robot/top_laser/scan_crudo"; nohup python3 $D/filtra_scan.py > $L/filtro_$TS.log 2>&1 </dev/null & fi
  nohup python3 $D/sim2d.py --mapa $MAPA --x $PX --y $PY --yaw $PYAW $EXTRA > $L/sim2d_$TS.log 2>&1 </dev/null &
  sleep 2; echo "sim2d: $(tail -1 $L/sim2d_$TS.log | cut -c1-140)"
  MODO=sin-amcl; [ $VERDAD = 1 ] || MODO=con-amcl
  bash $D/lanza_nav2.sh $PARAMS $L $MAPA $MODO
  for i in $(seq 1 30); do sleep 2; activo bt_navigator && activo behavior_server && activo controller_server && activo planner_server && break; done
  echo "Nav2: $(for n in map_server controller_server planner_server behavior_server bt_navigator smoother_server waypoint_follower velocity_smoother; do echo -n "$(activo $n && echo ok || echo NO) "; done)$( [ $VERDAD = 1 ] || echo "· amcl $(activo amcl && echo ok || echo NO)")"
  if [ $VERDAD = 0 ]; then bash $0 pose "$POSE"; fi
  nohup python3 $D/scan_latido.py > $L/latido_$TS.log 2>&1 </dev/null &
  nohup rviz2 -d $D/summit_sim.rviz > $L/rviz_$TS.log 2>&1 </dev/null &
  sleep 4; bash $0 estado;;
pose)
  P="$POSE"; read X Y YAW <<<"$(awk -v n="$P" '$1==n{print $2,$3,$4}' ~/ab/marcas.txt)"; [ -n "${X:-}" ] || read X Y YAW <<<"$P"
  Q=$(python3 -c "import math; print('%.6f %.6f' % (math.sin(math.radians($YAW)/2), math.cos(math.radians($YAW)/2)))"); read QZ QW <<<"$Q"
  timeout 8 ros2 topic pub --once -w 0 /sim/pose geometry_msgs/msg/PoseStamped "{header: {frame_id: robot_map}, pose: {position: {x: $X, y: $Y}, orientation: {z: $QZ, w: $QW}}}" >/dev/null 2>&1
  timeout 8 ros2 topic pub --once -w 0 /robot/initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: robot_map}, pose: {pose: {position: {x: $X, y: $Y}, orientation: {z: $QZ, w: $QW}}, covariance: [0.25,0,0,0,0,0,0,0.25,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.068]}}" >/dev/null 2>&1
  echo "pose puesta: ($X, $Y, $YAW°) en el simulador y en AMCL";;
estado)
  echo "procesos: sim2d $(pgrep -fc "summit_sim2d/[s]im2d.py") · nav2 $(pgrep -fc "/opt/ros/humble/lib/[n]av2_") · rviz $(pgrep -fc "[r]viz2 -d") · latido $(pgrep -fc "[s]can_latido.py")"
  echo "nodos: $(for n in bt_navigator controller_server planner_server behavior_server map_server amcl; do echo -n "$n=$(activo $n && echo ok || echo NO) "; done)"
  echo "latido: $(python3 $D/scan_latido.py edad 2>/dev/null)"
  echo "verdad: $(timeout 4 ros2 topic echo --once /sim/pose_verdad --field pose 2>/dev/null | grep -E '^\s*(x|y|z|w):' | awk '{printf "%s ", $2}' | python3 -c 'import sys,math; v=[float(t) for t in sys.stdin.read().split()]; print("(%.2f, %.2f) yaw %.1f°" % (v[0],v[1],math.degrees(2*math.atan2(v[5],v[6])))) if len(v)>=7 else print("sin datos")') · holgura $(timeout 3 ros2 topic echo --once /sim/holgura --field data 2>/dev/null) m · choque $(timeout 2 ros2 topic echo --once /sim/choque --field data 2>/dev/null || echo no)"
  echo "scan 3 s: $(timeout 3 ros2 topic echo --qos-reliability best_effort --field header.stamp.sec /robot/top_laser/scan 2>/dev/null | grep -c '^[0-9]') · amcl_pose: $(timeout 4 ros2 topic echo --once /robot/amcl_pose --field pose.pose.position 2>/dev/null | grep -E '^\s*(x|y):' | awk '{printf "%.2f ", $2}')";;
*) echo "uso: $0 arranca [--verdad] [--puerta X,Y,YAW,ANCHO] [--pose \"x y yaw\"] | estado | para | pose NOMBRE";;
esac
