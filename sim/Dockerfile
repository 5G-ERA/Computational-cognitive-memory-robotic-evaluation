# syntax=docker/dockerfile:1
# =============================================================================
# Gemelo digital del G1 — entorno completo (ROS 2 Humble + Gazebo + noVNC)
#
# Reconstruye el contenedor `g1sim:humble` que se montó A MANO el 23-jun-2026 y
# que hasta hoy sólo existía en el Docker del Mac. Las capas de abajo NO son
# inventadas: salen del historial real de la imagen (`docker history`) más una
# auditoría del contenedor vivo hecha el 07-ago-2026.
#
# CONSTRUIR (desde la RAÍZ del repo, no desde sim/):
#   docker buildx build --platform linux/amd64 -f sim/Dockerfile -t g1sim:humble .
#
# La imagen es amd64 a propósito: el original corre emulado en Apple Silicon y
# la física de Gazebo está calibrada con esa emulación. Cambiar de arquitectura
# cambiaría los tiempos de las travesías -> rompería la comparabilidad.
#
# AVISO HONESTO: este Dockerfile está derivado de la imagen real pero NO se ha
# construido todavía (la build emulada descarga varios GB y tarda). La primera
# vez que lo uses, valídalo con la comprobación del final de RECONSTRUIR_Y_LANZAR.md.
# =============================================================================

FROM --platform=linux/amd64 tiryoh/ros2-desktop-vnc:humble

# --- 1) Fuentes apt de ROS 2 y Gazebo -----------------------------------------
# La imagen base trae las listas caducadas; se rehacen con keyrings firmados.
RUN rm -f /etc/apt/sources.list.d/ros*.list /etc/apt/sources.list.d/gazebo*.list 2>/dev/null || true \
 && install -d -m 0755 /usr/share/keyrings \
 && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
      > /etc/apt/sources.list.d/ros2.list \
 && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/gazebo-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
      > /etc/apt/sources.list.d/gazebo-stable.list

# --- 2) Paquetes ROS ----------------------------------------------------------
# OJO: `ros-humble-rosbridge-suite` NO estaba en la imagen original — se instaló
# a mano DENTRO del contenedor vivo (auditoría 07-ago-2026: 6 paquetes añadidos,
# 1788 ficheros). Sin él no hay puente websocket 8765 y el adaptador del Mac no
# puede hablar con la simulación. Aquí queda fijado en la receta.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-slam-toolbox \
      ros-humble-navigation2 \
      ros-humble-nav2-bringup \
      ros-humble-pointcloud-to-laserscan \
      ros-humble-teleop-twist-keyboard \
      ros-humble-foxglove-bridge \
      ros-humble-rosbridge-suite \
      git \
 && rm -rf /var/lib/apt/lists/*

# --- 3) Entorno de shell (idéntico al del contenedor original) ----------------
RUN printf '%s\n' \
      'source /opt/ros/humble/setup.bash' \
      '[ -f /home/ubuntu/g1_ws/install/setup.bash ] && source /home/ubuntu/g1_ws/install/setup.bash' \
      'export LIBGL_ALWAYS_SOFTWARE=1' \
      'export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH}' \
      >> /etc/bash.bashrc

# --- 4) Descripción del robot (paquete OFICIAL de Unitree) --------------------
# NO se vendoriza en el repo: son 110 MB de terceros. Se genera aquí con el
# script propio, que extrae g1_description del monorepo de Unitree, rigidiza las
# juntas, arregla las rutas de las mallas y añade base_footprint + LiDAR + el
# plugin planar_move (cmd_vel -> movimiento + odometría).
ARG UNITREE_ROS_REPO=https://github.com/unitreerobotics/unitree_ros.git
ARG UNITREE_ROS_REF=master
WORKDIR /home/ubuntu/g1_ws/src
COPY sim/setup_g1.sh sim/regen_g1_nav.sh ./
RUN git clone --depth 1 --branch ${UNITREE_ROS_REF} ${UNITREE_ROS_REPO} unitree_ros \
 && bash setup_g1.sh \
 && bash regen_g1_nav.sh \
 && rm -rf unitree_ros unitree_ros_backup

# --- 5) El paquete de simulación propio ---------------------------------------
# g1_sim contiene los MUNDOS: lab.world es el escaneo real de la vivienda en el
# mismo marco de referencia que el robot físico (por eso valen los waypoints,
# el mapa de referencia y la puerta reales sin traducir nada).
COPY sim/g1_sim_pkg ./g1_sim

# --- 6) Build del workspace ---------------------------------------------------
WORKDIR /home/ubuntu/g1_ws
RUN bash -lc "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# El entrypoint (VNC + noVNC + escritorio) lo hereda de la imagen base.
# Puertos: 6080 noVNC (http), 5900 VNC crudo, 8765 rosbridge websocket.
EXPOSE 6080 5900 8765
