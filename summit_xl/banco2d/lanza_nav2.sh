#!/bin/bash
# Lanza Nav2 bajo el namespace /robot, nodo a nodo. 17-sep-2026, noche.
#
# Por qué no `ros2 launch nav2_bringup navigation_launch.py namespace:=robot`: en este Humble ese launch **no** empuja
# el namespace (no hay PushRosNamespace), sólo usa `namespace` como `root_key` para reescribir el YAML. Resultado: los
# nodos arrancan en la raíz (`/controller_server`) y sus parámetros quedan bajo `robot:`, que ya no les corresponde;
# el controlador aborta con «No critics defined for FollowPath» y el gestor de ciclo de vida cancela el bringup.
# Aquí cada nodo lleva `-r __ns:=/robot` y el YAML va envuelto en `robot:` — medido: configura y activa.
#
# TF: el simulador publica en los topics globales /tf y /tf_static, como hace el robot real; un nodo con namespace
# usaría /robot/tf, así que se remapea `tf` → `/tf` en todos.
#
#   bash lanza_nav2.sh PARAMS.yaml DIR_LOG MAPA.yaml [con-amcl|sin-amcl]
set -u
PARAMS=$1; L=$2; MAPA=$3; MODO=${4:-sin-amcl}
NS=/robot; TS=$(date +%H%M%S); ENV=/tmp/nav2_sim_ns.yaml
python3 - "$PARAMS" "$ENV" <<'PY'
import sys, yaml
yaml.safe_dump({"robot": yaml.safe_load(open(sys.argv[1]))}, open(sys.argv[2], "w"), sort_keys=False)
PY
lanza() {  # lanza PAQUETE EJECUTABLE [remapeos/parámetros extra…]
  local pkg=$1 exe=$2; shift 2
  nohup ros2 run "$pkg" "$exe" --ros-args -r __ns:=$NS -r tf:=/tf -r tf_static:=/tf_static \
    --params-file "$ENV" -p use_sim_time:=false "$@" > "$L/${exe}_$TS.log" 2>&1 </dev/null &
}
NODOS="'map_server','controller_server','smoother_server','planner_server','behavior_server','bt_navigator','waypoint_follower','velocity_smoother'"
lanza nav2_map_server map_server -p yaml_filename:="$MAPA" -p topic_name:=map -p frame_id:=robot_map
lanza nav2_controller controller_server -r cmd_vel:=cmd_vel_nav
lanza nav2_smoother smoother_server
lanza nav2_planner planner_server
lanza nav2_behaviors behavior_server -r cmd_vel:=cmd_vel_nav
lanza nav2_bt_navigator bt_navigator
lanza nav2_waypoint_follower waypoint_follower
lanza nav2_velocity_smoother velocity_smoother -r cmd_vel:=cmd_vel_nav -r cmd_vel_smoothed:=cmd_vel
if [ "$MODO" = con-amcl ]; then
  lanza nav2_amcl amcl
  NODOS="'amcl',$NODOS"
fi
sleep 5
nohup ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __ns:=$NS -r __node:=lifecycle_manager_sim \
  -p use_sim_time:=false -p autostart:=true -p bond_timeout:=0.0 -p "node_names:=[$NODOS]" \
  > "$L/lifecycle_$TS.log" 2>&1 </dev/null &
echo "lanzados bajo $NS ($MODO): $NODOS"
