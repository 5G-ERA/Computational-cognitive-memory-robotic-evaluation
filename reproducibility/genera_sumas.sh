#!/bin/sh
# Regenera reproducibility/SHA256SUMS con rutas RELATIVAS a la raiz del repo.
# (Leccion 1, del paquete v0.5.1: rutas absolutas = manifiesto inverificable.
#  Leccion 2, del 26-ago: un `ls` con un fichero ausente + set -e mataba el bloque
#  a mitad y el manifiesto salia PARCIAL en silencio. Ahora cada candidato se
#  filtra por existencia y los ausentes se AVISAN en stderr, nunca se callan.)
set -u
cd "$(dirname "$0")/.."

candidatos() {
  # nucleo de puntuacion y escenificacion
  echo src/dcc_omega.py; echo src/dcc_conditions.py; echo src/dcc_roles.py; echo src/dcc_secundarios.py
  echo src/guion.py; echo src/campana_dcc_v2.py; echo src/g1_goto.py; echo src/g1_sim_adapter.py
  echo sim/isaac/isaac_bridge.py; echo sim/isaac/office3d.py; echo sim/emulador_deteccion.py
  echo src/vox_rayos.py
  ls analysis/*.py 2>/dev/null
  # asignacion de ensayos y manifiestos
  ls tasks/manifiestos/*.txt 2>/dev/null; echo dataset/campana_isaac.json
  # certificados de referencia y T12
  ls dataset/*_omega_ref.json 2>/dev/null; echo dataset/certificado_T12.json
  # mapas, referencia de sesion y curvas
  echo summit/ref_map_g1.json; echo nav_map.json
  ls dataset/visibilidad_gemelo_sesion*.json 2>/dev/null
  echo dataset/curvas_etiqueta.json
  # runs de la campana v2 (las que lista su manifiesto)
  awk -F'|' '/\|/ && $3 != "" {print $3}' tasks/manifiestos/campana_dcc_v2.txt 2>/dev/null
  # resultados, documentos declarativos y el propio paquete
  echo tasks/RESULTADOS_ISAAC_V2.md; echo tasks/VARIANZA_N30.md
  echo tasks/DECISIONES_PENDIENTES_RENXI.md; echo tasks/SESSION_PREP_GATE_AB.md
  echo REPRODUCIBILITY.md
  echo reproducibility/requirements-frozen.txt; echo reproducibility/EXCLUSIONES.md
  echo reproducibility/exporta_resultados.py
  echo reproducibility/resultados_stage2_dev.json; echo reproducibility/resultados_stage2_dev.csv
}

TMP="$(mktemp)"
candidatos | sort -u | while IFS= read -r f; do
  if [ -f "$f" ]; then
    echo "$f"
  else
    echo "AVISO: candidato ausente, fuera del manifiesto: $f" >&2
  fi
done > "$TMP"

xargs sha256sum < "$TMP" > reproducibility/SHA256SUMS
rm -f "$TMP"
wc -l < reproducibility/SHA256SUMS | xargs echo "SHA256SUMS regenerado, entradas:"
