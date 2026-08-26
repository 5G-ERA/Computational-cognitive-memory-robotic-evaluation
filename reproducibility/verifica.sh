#!/bin/sh
# Verifica la integridad del paquete Stage 2 contra reproducibility/SHA256SUMS.
# Rutas relativas a la raiz del repo: funciona en cualquier clon.
set -e
cd "$(dirname "$0")/.."
sha256sum -c reproducibility/SHA256SUMS --quiet && echo "PAQUETE INTEGRO: todas las sumas verifican."
