# sim/ — G1 simulation container (campaña de simulación del tutor)

Contenedor Docker `g1sim:humble` (id `48e7c50b26d6`), base tipo Tiryoh **docker-ros2-desktop-vnc**:
ROS2 Humble Desktop + escritorio MATE servido por noVNC, con **Gazebo, Nav2, slam_toolbox,
foxglove_bridge y pointcloud_to_laserscan** preinstalados.

## Acceso

| Qué | Cómo |
|---|---|
| Escritorio (navegador) | `http://localhost:6080/vnc_lite.html` (cliente ligero) o `/vnc.html` — contraseña `ubuntu` |
| VNC crudo (cliente nativo) | puerto 5900 (no es HTTP: Safari no abre esto) |
| foxglove_bridge (websocket ROS) | puerto **8765** — ya mapeado; lo usará el adaptador |

Ver mapeos reales: `docker port 48e7c50b26d6`.

## Volcados de inspección (`container_dump/`)

- `00_inventario.txt` — env + entrypoint (VNC/noVNC/supervisor).
- `01_ros.txt` — paquetes ROS del sistema; topics/nodos (¡solo válido con la sim CORRIENDO!).
- `02_files.txt` — vacío (find a poca profundidad); repetir como `03_ws.txt` con `-maxdepth 8`.

Comandos de la segunda ronda (con la sim lanzada en el VNC):

```bash
cd "<carpeta G1 ROBOT>"
docker exec 48e7c50b26d6 bash -lc 'find /root /home -maxdepth 8 \( -name "package.xml" -o -name "*.launch.py" -o -name "*.world" -o -name "*.sdf" -o -name "*.xacro" -o -name "*.urdf" \) 2>/dev/null | grep -v /opt/ | head -100' > sim/container_dump/03_ws.txt 2>&1
docker exec 48e7c50b26d6 bash -lc 'tail -60 /home/*/.bash_history /root/.bash_history 2>/dev/null; echo ===LS===; ls -la /home/*/ 2>/dev/null' > sim/container_dump/04_home_hist.txt 2>&1
docker exec 48e7c50b26d6 bash -lc 'source /opt/ros/humble/setup.bash; timeout 5 ros2 topic list; echo ===; timeout 5 ros2 node list' > sim/container_dump/05_topics_con_sim.txt 2>&1
# cuando 03 revele la ruta del workspace:
# docker cp 48e7c50b26d6:/home/ubuntu/ros2_ws/src sim/ws_src
```

## Plan de integración (mismo código que el robot real)

`g1_sim_adapter.py` (pendiente, tras ver topics): un **FakeCDP** que implementa la interfaz del
puente USB real — pose, nube `location` en formato plano `[x,y,z,...]`, comando
`{lx,ly,rx,ry}` → `/cmd_vel` — hablando con el contenedor por el websocket de foxglove (:8765).
Así `g1_goto.py` + META2 + engagement corren **sin cambios** contra la simulación.

Cada run de sim se lanza con `G1_ENV=sim G1_SIM_ID=<escenario>` → queda etiquetada en dataset,
goto.log y `runs_summary.csv` (columnas `env`/`sim_id`) y no se mezcla jamás con las runs de
robot real en el análisis. Además harán falta: mapa/waypoints del escenario sim (`G1_REFMAP`,
`waypoints.json` propio) y la puerta del mundo simulado (`G1_DOOR_X/Y/AXIS`), o un escenario
que replique la geometría del lab.
