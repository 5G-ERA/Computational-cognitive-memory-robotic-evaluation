# Cómo lanzar la SIMULACIÓN del G1 (chuleta operativa)

Tres piezas: **contenedor** (Gazebo + ROS2) → **rosbridge** (puente websocket) → **adaptador**
en el Mac (corre el stack real de navegación contra la sim). Tiempo total: ~1 minuto.

## 0. Contenedor arriba

```bash
docker ps | grep g1sim          # ¿está corriendo?
docker start 48e7c50b26d6       # si no lo está
```

## 1. La simulación (terminal 1 del contenedor)

```bash
docker exec -it 48e7c50b26d6 bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/g1_ws/install/setup.bash
ros2 launch g1_sim sim.launch.py gui:=false
```

Debe terminar mostrando `Successfully spawned entity [g1]` y `planar_move: Subscribed to
[/cmd_vel]`. (El error de `gzclient` si olvidas `gui:=false` es inofensivo: es la GUI sin
pantalla; la física sigue.)

## 2. rosbridge (terminal 2 del contenedor)

```bash
docker exec -it 48e7c50b26d6 bash
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=8765
```

Debe decir `Rosbridge WebSocket server started on port 8765`.
(Instalación única si faltara: `apt update && apt install -y ros-humble-rosbridge-suite`.)

## 3. La run (Mac, carpeta G1 ROBOT)

```bash
source /Users/adrianlendinezibanez/unitree_webrtc_connect/.venv/bin/activate

python g1_sim_adapter.py gotoviz B              # con ventana en vivo
python g1_sim_adapter.py goto B                 # sin ventana (headless)
G1_META2=1 python g1_sim_adapter.py gotoviz B   # + gobernanza DCE en shadow
G1_META2=2 python g1_sim_adapter.py gotoviz B   # + gobernanza ACTIVA
python g1_sim_adapter.py waypoint A             # re-grabar waypoints del escenario
```

Checks de arranque: `backend grafico: TkAgg` · `/odom OK` · `/scan: OK` · `ENTORNO: SIM`.

Análisis después, igual que con el robot:

```bash
python autopsy.py dataset/<run>.json     # informe HTML (trayectoria, timelines, eventos)
python summarize_runs.py                 # CSV (las runs sim salen con env=sim / sim_id)
```

## Qué hace el adaptador por ti (defaults de sim)

`G1_ENV=sim` + `G1_SIM_ID=room_v1` (etiquetado) · `G1_NOVIS=1` (no hay cámara) ·
`G1_DOOR_ENGAGE=0` (room.world aún no tiene puerta) · `G1_RELOCGUARD=0` y `G1_NOGATE=1`
(la pose de la sim es exacta) · escenario propio en `sim/` (waypoints_sim, ref_map_room,
nav_map_sim) — **no toca los ficheros del lab** · misma física de sticks que el robot real
(deadzone 0.3, signos de strafe/giro calibrados).

## Problemas típicos

| Síntoma | Causa / arreglo |
|---|---|
| `command not found: python` | activa el venv (paso 3, primera línea) |
| `SIN /odom` | la sim del paso 1 no está corriendo |
| conecta pero no navega: "RELOCALIZACION DUDOSA" | adaptador viejo — `git pull` (lleva G1_NOGATE=1) |
| ventana invisible con `backend grafico: MacOSX` | bug de MacOSX+pause → `brew install python-tk@3.11` (el adaptador ya prefiere TkAgg) |
| `No module named '_tkinter'` | `brew install python-tk@3.11` |
| RViz "no tf data" | opcional; `rviz2 --ros-args -p use_sim_time:=true` y Fixed Frame `odom` a mano |

## Escenario actual y siguiente

`room.world`: sala 8×6 m + pilar 0.6 m en (1,−0.5). Waypoints sim: A(2.5,−1.8) · B(−2.5,1.8)
· C(−2.5,−1.8). SIGUIENTE: pared divisoria con vano de 0.8 m para replicar la puerta del lab
→ activar engagement en sim (`G1_DOOR_ENGAGE=1` + `G1_DOOR_X/Y/AXIS` del mundo) → matriz de
condiciones también en simulación.
