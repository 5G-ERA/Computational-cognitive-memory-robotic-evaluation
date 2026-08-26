#!/bin/bash
# Arranque del servidor de percepcion A PRUEBA DE REINICIOS (22-ago).
#
# POR QUE: --debug abre una ventana cv2 y necesita un X. Hasta hoy se usaba la sesion
# grafica :1, que SOLO existe si una persona inicio sesion fisicamente en el laboratorio.
# Tras un reinicio no hay nadie que la abra. Con Xvfb creamos una pantalla virtual propia:
# el servidor arranca igual, con overlay y todo, sin depender de nadie.
set -e
REPO=~/Documents/G1_UNITREE_ROBOT_META_REASONING
DISP=":99"

# 1) pantalla virtual (si no esta ya)
if ! xdpyinfo -display $DISP >/dev/null 2>&1; then
  echo "== levantando X virtual en $DISP"
  Xvfb $DISP -screen 0 1280x800x24 >/tmp/xvfb.log 2>&1 &
  sleep 3
fi
xdpyinfo -display $DISP >/dev/null 2>&1 && echo "X virtual OK en $DISP" || { echo "FALLO Xvfb"; exit 1; }

# 2) servidor de percepcion en tmux (sesion 'perc'), con overlay de suelo
tmux kill-session -t perc 2>/dev/null || true
tmux new-session -d -s perc "cd $REPO && DISPLAY=$DISP \
  PYTHONPATH=/home/ros/g1env/lib/python3.10/site-packages \
  /usr/bin/python3.10 src/perception_server.py --debug --floorcolor 1 2>&1 | tee /tmp/perc.log"

echo "== esperando a que carguen los modelos (hasta 120s)..."
for i in $(seq 1 24); do
  sleep 5
  if curl -s -m 3 http://127.0.0.1:8008/health | grep -q '"ok": true'; then
    curl -s http://127.0.0.1:8008/health
    echo
    echo "== PERCEPCION LISTA"
    exit 0
  fi
done
echo "== NO respondio a tiempo. Mira: tail /tmp/perc.log"
exit 1
