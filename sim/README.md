# sim/ — G1 simulation container (campaña de simulación del tutor)

> **07-ago-2026 — el gemelo ya es REPRODUCIBLE.** Antes, el contenedor sólo existía en el
> Docker del Mac: si se perdía, se perdía el entorno. Ahora hay receta completa:
> **`sim/Dockerfile`** (derivado del historial real de la imagen) + los scripts
> **`setup_g1.sh`** y **`regen_g1_nav.sh`** (que generan la descripción del robot desde el
> repositorio público de Unitree) + **`RUN_AND_REBUILD.md`** (lanzar, reconstruir,
> verificar y salvaguardar). Hallazgo de la auditoría: `rosbridge-suite` se había instalado
> a mano dentro del contenedor y **no estaba en la imagen** — sin él no hay puente 8765;
> ya está fijado en el Dockerfile.

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

## Integración (IMPLEMENTADA): `g1_sim_adapter.py` — mismo código que el robot real

El adaptador suplanta el objeto CDP del WebView con un **SimCDP** que resuelve los mismos
snippets JS contra ROS2 vía **rosbridge** (JSON/websocket, puerto 8765):
`window.__cmd{lx,ly,rx,ry}`→`/cmd_vel` (misma física calibrada: deadzone 0.3, lx>0=DERECHA→vy<0,
rx→wz≈−1.55·rx, ly 0.4→0.30 m/s) · `/odom`→pose · `/scan` 360°→nube `location` plana en frame
mapa. Escenario sim aislado: `sim/waypoints_sim.json`, `sim/ref_map_room.json` (generado del
room.world: 4 paredes + pilar), `sim/nav_map_sim.json` — **no toca los ficheros del lab**.
Defaults de sim: `G1_ENV=sim`, `G1_SIM_ID=room_v1`, `G1_NOVIS=1` (sin cámara), engagement de
puerta OFF (room.world no tiene puerta), reloc-guard OFF (pose perfecta).

### Lanzar una run de simulación

```bash
# Contenedor, terminal 1 (docker exec -it 48e7c50b26d6 bash):
source /opt/ros/humble/setup.bash && source <ws>/install/setup.bash
ros2 launch g1_sim sim.launch.py gui:=false

# Contenedor, terminal 2:
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=8765

# Mac, carpeta G1 ROBOT (venv con websocket-client + matplotlib):
python g1_sim_adapter.py gotoviz B                  # A->B esquivando el pilar
G1_META2=1 python g1_sim_adapter.py gotoviz B       # con gobernanza en shadow
G1_META2=2 python g1_sim_adapter.py gotoviz B       # gobernanza activa
```

Las runs quedan en `dataset/` etiquetadas `env=sim / sim_id=room_v1` (dataset, goto.log y
columnas `env`/`sim_id` del CSV) — jamás se mezclan con las runs de robot real. Instalación
única en el contenedor: `apt update && apt install -y ros-humble-rosbridge-suite`.

Siguiente iteración del escenario: añadir al `room.world` una pared divisoria con un vano de
0.8 m replicando la puerta del lab → activar engagement con `G1_DOOR_ENGAGE=1 G1_DOOR_X/Y/AXIS`
del mundo sim, y A/B de la matriz de condiciones también en simulación.
