#!/bin/bash
# Summit XL · deja la sesión lista con un comando, desde gpuedge. 18-sep-2026.
#
#   bash ~/Desktop/summit_listo.sh            # comprueba y arregla lo que falte (robot + estación + RViz)
#   bash ~/Desktop/summit_listo.sh estado     # sólo mira, no toca nada
#   bash ~/Desktop/summit_listo.sh para       # cierra lo de la estación (RViz, relés); el robot no se toca
#
# Cada paso lleva el porqué de la trampa que evita. Todas están medidas el 17 y el 18 de septiembre de 2026, y
# cada una costó entre cinco y veinte minutos de sesión con Natan delante.
set -u
H=summit-wifi
ENV='set +u; source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=39 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp'
EST="$ENV CYCLONEDDS_URI=file:///tmp/cyclone_estacion.xml"
MAPA=/home/ros/Desktop/extraccion_20260917/mapa/rbk_2026_09_17_16_23_20.yaml
RVIZ=/home/ros/Desktop/summit_estacion.rviz
ok()   { printf '  \033[32m✓\033[0m %s\n' "$*"; }
mal()  { printf '  \033[31m✗\033[0m %s\n' "$*"; }
tocar(){ printf '  \033[33m→\033[0m %s\n' "$*"; }
rob()  { ssh -o ConnectTimeout=8 -o BatchMode=yes $H "$ENV; $1" 2>/dev/null; }
# pgrep con clase de caracteres: un patrón literal casa con el propio pgrep y con el shell que lo lanza, y entonces
# `pkill -f` mata su propia sesión SSH (pasó dos veces el 17 y el 18). [x]yz nunca casa consigo mismo.
vivos(){ pgrep -fc "$1" 2>/dev/null || echo 0; }

ACC=${1:-arregla}
[ "$ACC" = para ] && {
  for p in $(pgrep -x rviz2) $(pgrep -f "[p]ub_tfstatic.py") $(pgrep -f "[l]anza_estacion_map.sh") $(pgrep -f "nav2_map_server/[m]ap_server"); do kill $p 2>/dev/null; done
  sleep 1; ok "estación cerrada (rviz $(pgrep -xc rviz2), relés $(vivos "[p]ub_tfstatic.py")); el robot sigue como estaba"; exit 0; }

echo "=== 1) robot en red"
ssh -o ConnectTimeout=6 -o BatchMode=yes $H true 2>/dev/null || { mal "no responde: enciéndelo y espera ~1 min a que coja WiFi"; exit 1; }
ok "responde · $(rob 'uptime | grep -oE "up [^,]*"')"

echo "=== 2) reloj  ·  si está mal, la odometría explota"
# El robot arranca con el reloj en 2023 y sus contenedores YA en marcha. Ponerlo entonces mete un salto de tiempo en
# el controlador y la odometría se va a decenas de miles de metros: 2235 m el 17-sep, 41078 m el 18-sep. Por eso el
# paso 3 va SIEMPRE después de tocar el reloj, no sólo cuando se ve raro.
ANIO=$(rob 'date +%Y')
if [ "${ANIO:-0}" -lt 2026 ]; then
  mal "reloj en $ANIO · ponlo TÚ (pide contraseña de sudo del robot) y vuelve a lanzar esto:"
  echo "      ssh -t $H \"sudo date -u -s '\$(date -u '+%F %T')' && date\""
  exit 1
fi
ok "reloj $(rob 'date "+%F %H:%M"')"

echo "=== 3) odometría"
OD=$(rob 'timeout 5 ros2 topic echo --once /robot/robotnik_base_control/odom --field pose.pose.position 2>/dev/null | grep -E "^\s*(x|y):" | awk "{printf \"%.1f \", \$2}"')
GRANDE=$(python3 -c "
v='''${OD:-999 999}'''.split()
print(1 if not v or max(abs(float(x)) for x in v) > 5 else 0)" 2>/dev/null || echo 1)
if [ "$GRANDE" = 1 ]; then
  tocar "odometría en ($OD) → reseteando"
  # OJO con el tipo: es SetOdometry (no set_odometry) y la petición es un geometry_msgs/Pose completo, no x/y/z sueltos.
  rob "timeout 20 ros2 service call /robot/robotnik_base_control/set_odometry robotnik_msgs/srv/SetOdometry '{pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'" | grep -q "success=True" && ok "odometría a cero" || mal "el reseteo falló: míralo antes de mover el robot"
else
  ok "odometría ($OD)"
fi

echo "=== 4) base y sensores"
rob 'bash ~/summit_ab.sh pre 2>&1' | grep -E "maestro:|SYNC|statusword|láser:|odom:" | sed 's/^/  /' | head -8

echo "=== 5) latido del láser y vigilante  ·  en el robot"
# La sonda `ros2 topic echo` desde el host da 0 con el scan fluyendo (medido el 17-sep: 10 reinicios del conversor,
# la mayoría en falso). El latido es un nodo persistente y es lo que mira summit_cruce.sh.
[ "$(rob 'pgrep -fc "scan_latido.py$"')" = 0 ] && { tocar "arrancando latido"; ssh -o BatchMode=yes $H "$ENV; setsid nohup python3 ~/scan_latido.py >/dev/null 2>&1 </dev/null &" ; sleep 6; }
[ "$(rob 'pgrep -fc "^bash /home/robot/vigila_scan.sh"')" = 0 ] && { tocar "arrancando vigilante del scan"; ssh -o BatchMode=yes $H "$ENV; setsid nohup bash /home/robot/vigila_scan.sh >/dev/null 2>&1 </dev/null &"; sleep 2; }
ok "latido: $(rob 'python3 ~/scan_latido.py edad')"

echo "=== 6) estación: DDS, transformadas fijas y mapa"
# /tmp se limpia entre arranques de gpuedge y se lleva por delante los tres ficheros de la estación.
[ -f /tmp/cyclone_estacion.xml ] || { tocar "recreando /tmp/cyclone_estacion.xml"; printf '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enp6s0"/></Interfaces><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>\n' > /tmp/cyclone_estacion.xml; }
if [ "$(vivos "[p]ub_tfstatic.py")" = 0 ]; then
  tocar "capturando las transformadas fijas del robot y republicándolas"
  # /tf_static es transient_local y su único envío NO cruza el enlace: sin esto RViz no dibuja ni el robot ni el láser.
  ssh -o BatchMode=yes $H "$ENV; timeout 12 ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable /tf_static" 2>/dev/null > /tmp/tfstatic_robot.yaml
  # `ros2 topic echo --once` cierra con una línea `---`, y eso es un SEGUNDO documento YAML que revienta safe_load.
  python3 - <<'PY'
t = open("/tmp/tfstatic_robot.yaml").read()
open("/tmp/tfstatic_robot.yaml", "w").write(t.split("\n---\n")[0])
PY
  N=$(grep -c child_frame_id /tmp/tfstatic_robot.yaml)
  [ "$N" -gt 0 ] || { mal "no se capturaron transformadas fijas"; exit 1; }
  setsid nohup bash -c "$EST; python3 /tmp/pub_tfstatic.py /tmp/tfstatic_robot.yaml" > /tmp/tfstatic.log 2>&1 </dev/null &
  sleep 5
fi
ok "transformadas fijas: $(vivos "[p]ub_tfstatic.py") relé vivo"
if [ "$(vivos "nav2_map_server/[m]ap_server")" = 0 ]; then
  tocar "sirviendo el mapa como /estacion/map"
  # Lanzar esto EN LÍNEA por SSH falla (la sesión muere con 255 antes de arrancarlo): va en su propio script.
  cat > /tmp/lanza_estacion_map.sh <<EOF
#!/bin/bash
$EST
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$MAPA -p topic_name:=/estacion/map -p frame_id:=robot_map &
sleep 5
ros2 lifecycle set /map_server configure     # el nodo se llama /map_server: -r __node:= NO lo renombra
ros2 lifecycle set /map_server activate
wait
EOF
  chmod +x /tmp/lanza_estacion_map.sh
  setsid nohup /tmp/lanza_estacion_map.sh > /tmp/estacion_map.log 2>&1 </dev/null &
  sleep 12
fi
ok "mapa de la estación: $(grep -c Activating /tmp/estacion_map.log 2>/dev/null) activaciones"

echo "=== 7) ¿ve la estación al robot?"
VE=$(bash -c "$EST; echo \"topics \$(timeout 12 ros2 topic list 2>/dev/null | wc -l) · tf \$(timeout 4 ros2 topic echo /tf --field transforms 2>/dev/null | grep -c child_frame_id) · scan \$(timeout 4 ros2 topic echo --qos-reliability best_effort --field header.stamp.sec /robot/top_laser/scan 2>/dev/null | grep -c '^[0-9]')\"")
ok "$VE"

echo "=== 8) RViz"
if [ "$(pgrep -xc rviz2)" = 0 ]; then
  tocar "abriendo RViz en la pantalla de gpuedge"
  cat > /tmp/lanza_rviz.sh <<EOF
#!/bin/bash
$EST
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority
# el remapeo va DESPUÉS de --ros-args: "rviz2 -r ..." es 'Unknown option r' y no arranca
exec rviz2 -d $RVIZ --ros-args -r /tf_static:=/tf_static_estacion
EOF
  chmod +x /tmp/lanza_rviz.sh
  setsid nohup /tmp/lanza_rviz.sh > /tmp/rviz.log 2>&1 </dev/null &
  sleep 12
fi
[ "$(pgrep -xc rviz2)" -gt 0 ] && ok "RViz abierto" || mal "RViz no arrancó · mira /tmp/rviz.log"

echo "=== 9) Nav2 y localización"
ok "Nav2 $(rob 'for n in controller_server planner_server behavior_server bt_navigator waypoint_follower velocity_smoother smoother_server amcl map_server; do timeout 4 ros2 lifecycle get /robot/$n 2>/dev/null | grep -c "active \[3\]"; done | paste -sd+ | bc')/9 · map→odom $(rob 'timeout 4 ros2 topic echo --field transforms /tf 2>/dev/null | grep -c robot_odom') en 4 s"
AM=$(rob 'timeout 5 ros2 topic echo --once /robot/amcl_pose --field pose.pose.position 2>/dev/null | grep -E "^\s*(x|y):" | awk "{printf \"%.2f \", \$2}"')
echo
echo "AMCL en ($AM). Si el robot está en A y esto no es (≈0,0 · ≈0,0), dale la pose:"
echo "    ssh $H \"bash ~/summit_ab.sh pose A\""
echo "Y para cruzar:   ssh $H \"bash ~/summit_cruce.sh\"   · Natan a un lado del marco, con la seta."
