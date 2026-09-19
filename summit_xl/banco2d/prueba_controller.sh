#!/bin/bash
# Prueba directa del controller_server con nav2_sim.yaml bajo el namespace /robot (dominio 41, aislado).
set +u; source /opt/ros/humble/setup.bash; set -u
export ROS_DOMAIN_ID=41 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///tmp/cyclone_estacion.xml
cd ~/summit_sim2d
python3 - <<PY
import yaml; d=yaml.safe_load(open("nav2_sim.yaml")); yaml.safe_dump({"robot": d}, open("/tmp/nav2_sim_ns.yaml","w"), sort_keys=False)
PY
timeout 20 ros2 run nav2_controller controller_server --ros-args -r __ns:=/robot --params-file /tmp/nav2_sim_ns.yaml > /tmp/cs_ns.log 2>&1 &
sleep 4; timeout 8 ros2 lifecycle set /robot/controller_server configure 2>&1 | tail -1; sleep 2
grep -E "critic|Critic|ERROR|Configur|terminate|what\(\)" /tmp/cs_ns.log | head -10 | cut -c1-160
