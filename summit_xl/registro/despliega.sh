#!/bin/bash
# despliega.sh — deja el registro en el robot. g1_metrics.py NO vive en este directorio: es src/g1_metrics.py,
# el mismo fichero del G1; aquí se copia junto al script porque en el robot no está el repo.
#   bash summit_xl/registro/despliega.sh [alias-ssh-del-robot]        # por defecto: summit-wifi
set -e
H=${1:-summit-wifi}; AQUI=$(cd "$(dirname "$0")" && pwd); REPO=$(cd "$AQUI/../.." && pwd)
python3 "$AQUI/prueba_core.py" | tail -1 | grep -q "TODO OK" || { echo "la prueba en frío NO pasa: no despliego"; exit 1; }
ssh "$H" 'mkdir -p ~/registro ~/dataset'
scp -q "$AQUI"/summit_run_logger.py "$AQUI"/summit_run_core.py "$AQUI"/summit_con_registro.sh "$REPO"/src/g1_metrics.py "$H":~/registro/
ssh "$H" 'cd ~/registro && sha256sum g1_metrics.py summit_run_*.py | cut -c1-16,65-'
echo "desplegado en $H:~/registro · las runs irán a ~/dataset"
