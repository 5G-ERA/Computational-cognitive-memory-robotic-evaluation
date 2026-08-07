# G1 simulation desktop (ROS 2 Humble + Gazebo Classic) viewable in a web browser.
# Base image bundles a Linux desktop served over noVNC (web VNC).
#
# PROVENANCE: this is the ORIGINAL recipe used to build g1sim:humble on 2026-06-23,
# rescued from ~/Downloads/g1_sim_docker/ and versioned here on 2026-08-07 (it had
# never been in git). Verified against `docker history` of the live image.
# The ROS workspace is NOT baked in: it is bind-mounted from the host, see
# docker-compose.yml and RECONSTRUIR_Y_LANZAR.md.
FROM tiryoh/ros2-desktop-vnc:humble

# The base image carries a stale/duplicate ROS apt source -> packages.ros.org is not
# indexed -> "Unable to locate package ros-humble-*".  Remove old ROS lists, then add a
# clean ROS 2 source (fresh key) and the OSRF Gazebo Classic repo.
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

# ROS 2 sim + navigation stack
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-teleop-twist-keyboard \
    ros-humble-foxglove-bridge \
    ros-humble-rosbridge-suite \
  && rm -rf /var/lib/apt/lists/*
# NOTE (audit 2026-08-07): rosbridge-suite was MISSING from the original image — it had
# been apt-installed by hand inside the running container (6 packages, 1788 files), so a
# rebuild from the old Dockerfile produced a twin with no websocket bridge on 8765 and the
# Mac-side adapter could not talk to the simulation. Pinned in the recipe from now on.

# Convenience: auto-source ROS + workspace, force software OpenGL (no GPU needed).
# Write to the SYSTEM-wide bashrc: the 'ubuntu' user/home is created at container start,
# so /home/ubuntu/.bashrc does not exist yet at build time.
RUN printf '%s\n' \
    'source /opt/ros/humble/setup.bash' \
    '[ -f /home/ubuntu/g1_ws/install/setup.bash ] && source /home/ubuntu/g1_ws/install/setup.bash' \
    'export LIBGL_ALWAYS_SOFTWARE=1' \
    'export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH}' \
    >> /etc/bash.bashrc
