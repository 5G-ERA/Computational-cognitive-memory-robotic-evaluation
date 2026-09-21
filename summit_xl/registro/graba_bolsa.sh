#!/bin/bash
# graba_bolsa.sh — la captura que pide la reconstrucción en frío del rechazo del
# comprobador de colisión. Se lanza EN EL ROBOT, antes de la travesía.
#
#   bash ~/registro/graba_bolsa.sh <etiqueta>      # p.ej: recto_1
#   ...la travesía...
#   Ctrl+C, o:  bash ~/registro/graba_bolsa.sh para
#
# POR QUÉ ASÍ. La primera pose que el behavior server rechaza no se publica en
# ningún topic: se comprueba ANTES de mandar el cmd_vel, así que `cmd_vel` no
# basta y no hay forma de verla en vivo sin parchear Nav2. Lo que sí se puede es
# grabar todo lo que el comprobador MIRA y repetir su cuenta después, en frío.
# El resultado de eso no es «la pose que se rechazó» sino «la primera pose
# rechazada en la reconstrucción», y conviene llamarlo por su nombre.
#
# LA CABECERA NO ES ADORNO. Una bolsa sin los parámetros efectivos, la versión
# de los paquetes y el identificador de la imagen no se puede reconstruir: el
# comprobador depende del padding, de la resolución y del `simulate_ahead_time`
# que estuvieran puestos ESE día, y los contenedores se recrean. Por eso la
# cabecera se escribe primero y la grabación no arranca si falla.
set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-39} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
source /opt/ros/humble/setup.bash 2>/dev/null

NS=${NS:-/robot}
BASE=${BASE:-$HOME/bolsas}
PIDF=/tmp/graba_bolsa.pid

# --- parar -----------------------------------------------------------------
if [ "${1:-}" = "para" ]; then
    [ -f "$PIDF" ] || { echo "no hay ninguna grabación en marcha"; exit 0; }
    kill -INT "$(cat "$PIDF")" 2>/dev/null
    for _ in $(seq 1 20); do kill -0 "$(cat "$PIDF")" 2>/dev/null || break; sleep 0.5; done
    rm -f "$PIDF"; echo "grabación parada"; exit 0
fi

ETIQUETA=${1:-}
[ -n "$ETIQUETA" ] || { sed -n '2,12p' "$0"; exit 2; }
[ -f "$PIDF" ] && kill -0 "$(cat "$PIDF")" 2>/dev/null && { echo "ya hay una grabación en marcha ($(cat "$PIDF")); párala primero"; exit 1; }

DIR="$BASE/$(date +%Y%m%d_%H%M%S)_$ETIQUETA"
mkdir -p "$DIR"
CAB="$DIR/cabecera.txt"

say() { echo "$*" | tee -a "$CAB"; }

say "=== $ETIQUETA · $(date -Iseconds) ==="
say ""

# --- 1 · qué está corriendo -------------------------------------------------
say "--- contenedores (imagen e identificador: sin esto no se reconstruye) ---"
docker ps --format '{{.Names}}  {{.Image}}  {{.ID}}' 2>&1 | tee -a "$CAB" >/dev/null
docker ps --format '{{.Names}}  {{.Image}}  {{.ID}}' 2>&1 | sed 's/^/    /'

say ""
say "--- nodos vivos ---"
timeout 15 ros2 node list 2>/dev/null | sed 's/^/    /' | tee -a "$CAB" >/dev/null

# --- 2 · los parámetros EFECTIVOS, no los del fichero ----------------------
for N in behavior_server local_costmap/local_costmap global_costmap/global_costmap controller_server planner_server amcl; do
    F="$DIR/param_$(echo "$N" | tr '/' '_').yaml"
    timeout 20 ros2 param dump "$NS/$N" > "$F" 2>/dev/null && say "param  $N -> $(basename "$F") ($(wc -l < "$F") líneas)" \
        || say "param  $N -> NO CONTESTA"
done

# --- 3 · quién publica y con qué QoS ---------------------------------------
# Importa para la reconstrucción y para la propia grabación: un topic
# transient_local no se captura igual que uno volátil.
say ""
say "--- QoS y publicadores de lo que mira el comprobador ---"
for T in "$NS/local_costmap/costmap_raw" "$NS/local_costmap/published_footprint" /tf_static; do
    say "  $T"
    timeout 15 ros2 topic info -v "$T" 2>/dev/null | grep -E "Type|Node name|Reliability|Durability|Depth|count" | sed 's/^/      /' | tee -a "$CAB" >/dev/null
done

# --- 4 · versiones ----------------------------------------------------------
say ""
say "--- versiones de los paquetes que deciden ---"
for P in nav2_behaviors nav2_costmap_2d nav2_controller; do
    V=$(timeout 10 ros2 pkg xml "$P" 2>/dev/null | grep -m1 -oE '<version>[^<]+' | cut -c10-)
    say "  $P ${V:-desconocida}"
done

# --- 5 · la grabación -------------------------------------------------------
# costmap_raw, published_footprint y tf_static son transient_local: sin el
# override, `ros2 bag record` los abre como volátiles y se pierde el mensaje
# latcheado, que es justo el que hace falta para el primer tick.
cat > "$DIR/qos.yaml" <<'YAML'
/tf_static:
  reliability: reliable
  durability: transient_local
  history: keep_last
  depth: 100
YAML
for T in local_costmap/costmap_raw local_costmap/published_footprint; do
    cat >> "$DIR/qos.yaml" <<YAML
$NS/$T:
  reliability: reliable
  durability: transient_local
  history: keep_last
  depth: 1
YAML
done

TOPICS=(
  "$NS/local_costmap/costmap_raw"            # el costmap que el comprobador consulta
  "$NS/local_costmap/published_footprint"    # la huella YA transformada, con su sello
  /tf /tf_static                             # para recuperar la huella en su propio sello
  "$NS/top_laser/scan"                       # lo que alimenta la capa de obstáculos
  /rosout                                    # los literales: Collision Ahead, TEB, paciencia
  "$NS/robotnik_base_control/cmd_vel_limited"  # la orden efectiva, tras el mux
  "$NS/cmd_vel_nav" "$NS/move_base/cmd_vel"  # quién manda: Nav2 o el cruce por láser
  "$NS/amcl_pose" "$NS/joint_states"         # para cruzarlo con el registro de runs
)
say ""
say "--- grabando ${#TOPICS[@]} topics en $DIR/bolsa ---"

cat > "$DIR/arranca.sh" <<SH
cd "$DIR" && exec ros2 bag record -o bolsa --qos-profile-overrides-path qos.yaml ${TOPICS[*]}
SH
setsid nohup bash "$DIR/arranca.sh" > "$DIR/bag.log" 2>&1 < /dev/null &
echo $! > "$PIDF"
sleep 4
if kill -0 "$(cat "$PIDF")" 2>/dev/null; then
    echo "grabando (pid $(cat "$PIDF")) · para con:  bash $0 para"
    echo "  $DIR"
else
    echo "LA GRABACIÓN NO ARRANCÓ. Mira $DIR/bag.log"; rm -f "$PIDF"; exit 1
fi
