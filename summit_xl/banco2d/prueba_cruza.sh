#!/bin/bash
# N cruces con cruza_vano.py desde vano_antes, con un descentrado lateral y un error de rumbo distintos cada vez.
#   bash prueba_cruza.sh "lat_cm:rumbo_deg lat_cm:rumbo_deg …"
set +u; source /opt/ros/humble/setup.bash; set -u
export ROS_DOMAIN_ID=40 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///tmp/cyclone_estacion.xml
cd ~/summit_sim2d; OK=0; N=0
for caso in $1; do
  LAT=${caso%%:*}; RUM=${caso##*:}
  P=$(python3 -c "
import math; ax,ay,eje=1.7345,-3.213,-89.8; th=math.radians(eje); vx,vy=-math.sin(th),math.cos(th)
print('%.3f %.3f %.1f' % (ax+$LAT/100*vx, ay+$LAT/100*vy, eje+$RUM))")
  bash summit_sim.sh pose "$P" >/dev/null 2>&1; sleep 3
  R=$(python3 cruza_vano.py --metros 1.50 --csv /tmp/cruce_${N}.csv 2>/dev/null | grep "cruce:")
  echo "  lat ${LAT} cm · rumbo ${RUM}° → $R"; N=$((N+1)); echo "$R" | grep -q "LLEGÓ" && OK=$((OK+1))
done
echo "  → $OK de $N"
