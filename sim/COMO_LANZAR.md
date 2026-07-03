# Cómo lanzar la SIMULACIÓN del G1 (chuleta operativa)

Tres piezas: **contenedor** (Gazebo + ROS2) → **rosbridge** (puente websocket) → **adaptador**
en el Mac (corre el stack real de navegación contra la sim). Tiempo total: ~1 minuto.

## 0. Contenedor arriba

```bash
docker ps | grep g1sim          # ¿está corriendo?
docker start 48e7c50b26d6       # si no lo está
```

## 1. La simulación (terminal 1 del contenedor) — HEADLESS, el modo de las runs de datos

```bash
docker exec -it 48e7c50b26d6 bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/g1_ws/install/setup.bash
ros2 launch g1_sim lab.launch.py           # escenario LAB (defecto; gui:=false ya es el default)
# ros2 launch g1_sim sim.launch.py gui:=false   # escenario room (sala sintética, sin puerta)
```

Debe terminar mostrando `Successfully spawned entity [g1]` y `planar_move: Subscribed to
[/cmd_vel]`. Las runs de DATOS van siempre headless (el render por CPU roba física).

**Modelo del robot** (`G1_SIM_MODEL`, default `full`): `full` = G1 completo con la caja de
colisión del ENVOLVENTE REAL (0.44 ancho de hombros × 1.10 alto, dato P2; tope bajo el plano
del lidar a z≈1.20 para que el láser no se vea a sí mismo). `G1_SIM_MODEL=box` = caja simple
0.30×0.25 (la física antigua, más permisiva). OJO 2026-07-03: con la caja vieja el robot
"cruzaba" la puerta con los hombros visuales DENTRO de la pared (falso éxito); con el
envolvente real se descubrió que el vano carvado medía 0.42 m — re-carvado al físico real:
vano 0.85–1.2 m, punto más estrecho del corredor 0.71 m. Validado: 3 runs seguidas
reached / 0 colisiones (181212 / 182620 / 182859).

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

## Ver la simulación en 3D (GUI, NO headless) — el patrón que funciona

REGLA DE ORO: **un solo gzserver**. Nunca lances un segundo `gazebo`/`ros2 launch` desde el
VNC (ni desde otro exec) con otro ya vivo: el segundo gzserver muere con `exit code 255`
(puerto del master ocupado) y su spawn falla con `Entity [g1] already exists` — esa PAREJA de
errores es la firma exacta de "hay otro gzserver vivo". Comprobar y limpiar ANTES de relanzar:

```bash
ps aux | grep -E "gzserver|gzclient" | grep -v grep   # ¿quién vive?
kill -9 <PID>       # matar por PID; pkill -f "gzserver" puede colgarse (se auto-empareja)
```

El patrón validado para ver el mundo en 3D:

```bash
# 1) limpiar (terminal exec):
pkill -f gzserver; pkill -f gzclient; sleep 2
# 2) UNA sim headless (terminal exec, con ambos source):
ros2 launch g1_sim lab.launch.py
# 3) en la terminal DEL ESCRITORIO VNC, SOLO el cliente (se engancha al gzserver vivo):
source /opt/ros/humble/setup.bash && source ~/g1_ws/install/setup.bash
gzclient
```

Notas GUI: el robot puede salir sin meshes (el URDF usa package:// que gzclient no siempre
resuelve) → menú **View → Collisions** lo muestra como cajas naranjas; el arreglo de meshes es
`export GAZEBO_MODEL_PATH=/home/ubuntu/g1_ws/install/g1_description/share:$GAZEBO_MODEL_PATH`
(ya añadido a ~/.bashrc del contenedor). `gui:=true` desde un docker exec normal NO pinta
ventana (gzclient se cuelga creando el contexto GL por software; X está bien — xeyes funciona).
Alternativa 3D ligera: `rviz2 --ros-args -p use_sim_time:=true` + Fixed Frame `odom` (a mano)
+ LaserScan `/scan` + TF. Para pegar texto en el VNC: usa `http://localhost:6080/vnc.html`
(el completo tiene panel de portapapeles; el lite no) — o mejor, no tecleas en el VNC: todo
por exec. Las runs de DATOS siempre headless (el render por CPU roba física).

## Problemas típicos

| Síntoma | Causa / arreglo |
|---|---|
| `command not found: python` | activa el venv (paso 3, primera línea) |
| `SIN /odom` | la sim del paso 1 no está corriendo |
| gzserver `exit code 255` + spawn `Entity [g1] already exists` | hay OTRO gzserver vivo (regla de oro): `ps aux \| grep gzserver` y `kill -9 <PID>` antes de relanzar |
| el robot "atraviesa" paredes visualmente | cuerpo visual > caja de colisión; mira View→Collisions. Si atraviesa DE VERDAD (odom cruza pared), el URDF del contenedor está viejo: re-copia `sim/g1_nav.urdf` a src+install de g1_description y relanza |
| el robot presiona el marco de la puerta sin avanzar (ENG-GO, prog=0.00) | física legítima con el envolvente 0.44 si va descentrado; si pasa SIEMPRE, mide el vano del mundo (¿re-carveo aplicado? commit 4b4e2d4) |
| conecta pero no navega: "RELOCALIZACION DUDOSA" | adaptador viejo — `git pull` (lleva G1_NOGATE=1) |
| ventana invisible con `backend grafico: MacOSX` | bug de MacOSX+pause → `brew install python-tk@3.11` (el adaptador ya prefiere TkAgg) |
| `No module named '_tkinter'` | `brew install python-tk@3.11` |
| RViz "no tf data" | opcional; `rviz2 --ros-args -p use_sim_time:=true` y Fixed Frame `odom` a mano |

## Escenarios

**`lab` (DEFECTO)** — el mundo `lab.world` GENERADO DEL MAPA REAL del laboratorio (paredes del
refmap summit a 2.2 m + muebles de nav_map a 0.8 m, mismo frame G1). Usa los waypoints REALES
A/B/C de `waypoints.json`, el refmap real, y la puerta real con **engagement activo**
(G1_DOOR_X/Y/AXIS de siempre). El robot spawnea en A. Las runs sim son directamente
comparables con las del robot. Instalación en el contenedor (una vez, desde G1 ROBOT):

```bash
docker cp sim/g1_sim_pkg/worlds/lab.world      48e7c50b26d6:/home/ubuntu/g1_ws/src/g1_sim/worlds/
docker cp sim/g1_sim_pkg/launch/lab.launch.py  48e7c50b26d6:/home/ubuntu/g1_ws/src/g1_sim/launch/
docker cp sim/g1_sim_pkg/worlds/lab.world      48e7c50b26d6:/home/ubuntu/g1_ws/install/g1_sim/share/g1_sim/worlds/
docker cp sim/g1_sim_pkg/launch/lab.launch.py  48e7c50b26d6:/home/ubuntu/g1_ws/install/g1_sim/share/g1_sim/launch/
# y el URDF del modelo full (¡a los DOS sitios! olvidarlo = robot fantasma con URDF viejo):
docker cp sim/g1_nav.urdf  48e7c50b26d6:/home/ubuntu/g1_ws/src/g1_description/urdf/g1_nav.urdf
docker cp sim/g1_nav.urdf  48e7c50b26d6:/home/ubuntu/g1_ws/install/g1_description/share/g1_description/urdf/g1_nav.urdf
```

OJO: los ficheros copiados solo afectan a la SIGUIENTE sim — un gzserver ya corriendo mantiene
el mundo/robot con los que arrancó. Tras un docker cp, reinicia el launch.

Lanzar (paso 1 alternativo): `ros2 launch g1_sim lab.launch.py`  (gui:=false por defecto).
Runs: `python g1_sim_adapter.py gotoviz B` — sin más; el adaptador usa escenario lab por defecto.
Preview del mundo generado: `sim/lab_world_preview.png`. Nota: `waypoint`/`sweep` en sim
escriben en `sim/nav_map_lab.json` (copia), nunca en el `nav_map.json` real.

**`room`** — la sala sintética 8×6 con pilar (v1). `G1_SIM_SCENARIO=room python
g1_sim_adapter.py gotoviz B` (waypoints sim propios, sin puerta, engagement OFF).
