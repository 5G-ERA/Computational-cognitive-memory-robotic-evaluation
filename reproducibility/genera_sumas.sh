#!/bin/sh
# Regenera reproducibility/SHA256SUMS con rutas RELATIVAS a la raiz del repo.
# (Leccion del paquete v0.5.1: un manifiesto con rutas absolutas es inverificable
# para cualquier destinatario. Este se comprueba con `sh reproducibility/verifica.sh`.)
set -e
cd "$(dirname "$0")/.."

{
  # nucleo de puntuacion y escenificacion
  ls dcc_omega.py dcc_conditions.py dcc_roles.py dcc_secundarios.py \
     guion.py campana_dcc_v2.py g1_goto.py g1_sim_adapter.py \
     sim/isaac/isaac_bridge.py sim/isaac/office3d.py sim/emulador_deteccion.py \
     vox_rayos.py 2>/dev/null
  ls analysis/*.py
  # asignacion de ensayos y manifiestos
  ls tasks/manifiestos/*.txt dataset/campana_isaac.json
  # certificados de referencia y T12
  ls dataset/*_omega_ref.json dataset/certificado_T12.json
  # mapas, referencia de sesion y curvas
  ls summit/ref_map_g1.json nav_map.json dataset/visibilidad_gemelo_sesion*.json \
     dataset/curvas_etiqueta.json 2>/dev/null
  # runs de la campana v2 (las que lista su manifiesto)
  awk -F'|' '/\|/ && $3 != "" {print $3}' tasks/manifiestos/campana_dcc_v2.txt
  # resultados y documentos declarativos
  ls tasks/RESULTADOS_ISAAC_V2.md tasks/VARIANZA_N30.md \
     tasks/DECISIONES_PENDIENTES_RENXI.md tasks/SESSION_PREP_GATE_AB.md \
     REPRODUCIBILITY.md reproducibility/requirements-frozen.txt
} | sort -u | xargs sha256sum > reproducibility/SHA256SUMS

wc -l < reproducibility/SHA256SUMS | xargs echo "SHA256SUMS regenerado, entradas:"
