#!/bin/bash
# summit_con_registro.sh — envuelve CUALQUIER travesia con el registro de run, sin tocar el script que la hace.
#
#   bash ~/summit_con_registro.sh DESTINO BRAZO -- <comando de la travesia>
#   bash ~/summit_con_registro.sh B C1-LASER -- bash ~/summit_ab_holgura.sh B
#   bash ~/summit_con_registro.sh B C0-NAV2  -- bash ~/summit_ab.sh ir_marca B
#
# DESTINO es un nombre de ~/ab/marcas.txt. BRAZO es la etiqueta de la condicion (va a la columna 'arm').
# El registro es pasivo: si muere o no arranca, la travesia sigue igual. Deja la run en ~/dataset/.
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-39} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
DEST=$1; BRAZO=$2; shift 2; [ "$1" = "--" ] && shift
[ -n "$DEST" ] && [ -n "$BRAZO" ] && [ $# -gt 0 ] || { sed -n '2,9p' "$0"; exit 2; }
AQUI=$(cd "$(dirname "$0")" && pwd); RES=$(mktemp /tmp/run_result.XXXXXX); : > "$RES"
python3 "$AQUI/summit_run_logger.py" --label "$DEST" --marca "$DEST" --arm "$BRAZO" --result-file "$RES" \
        > /tmp/summit_run_logger.log 2>&1 &
REG=$!
sleep 4
kill -0 $REG 2>/dev/null || echo "AVISO: el registro no ha arrancado (mira /tmp/summit_run_logger.log). La travesia sigue SIN registro."
"$@"; RC=$?
sleep 2                                   # que entren las ultimas muestras con el robot ya parado
kill -INT $REG 2>/dev/null
for _ in $(seq 1 20); do kill -0 $REG 2>/dev/null || break; sleep 0.5; done
kill -0 $REG 2>/dev/null && { echo "AVISO: el registro no cerraba; lo termino"; kill -TERM $REG; sleep 2; }
tail -4 /tmp/summit_run_logger.log; rm -f "$RES"
exit $RC
