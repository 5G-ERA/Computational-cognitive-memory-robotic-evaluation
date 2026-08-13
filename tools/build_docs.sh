#!/usr/bin/env bash
# Regenera los PDFs de procedimiento desde sus fuentes HTML versionadas en docs/src/.
#
# POR QUE EXISTE: el 07-ago-2026 los HTML fuente vivian en una carpeta temporal, se
# perdieron, y una regeneracion contra ficheros inexistentes dejo dos PDFs convertidos
# en una pagina de error ("no se ha podido acceder al archivo"). Se restauraron desde
# git y las fuentes pasaron al repo. Regenera SIEMPRE con este script.
#
# Uso:  bash tools/build_docs.sh          (desde la raiz del repo)
set -euo pipefail

# Cualquier navegador Chromium sirve (--headless --print-to-pdf es comun a todos).
# El 13-ago-2026 Chrome desaparecio del Mac y el script murio: por eso se busca en lista.
CHROME=""
for c in "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" \
         "/Applications/Microsoft Edge.app/Contents/MacOS/Microsoft Edge" \
         "/Applications/Chromium.app/Contents/MacOS/Chromium" \
         "/Applications/Brave Browser.app/Contents/MacOS/Brave Browser"; do
  [ -x "$c" ] && { CHROME="$c"; break; }
done
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$HERE/docs/src"
OUT="$HERE/docs"

[ -n "$CHROME" ] || { echo "ERROR: no encuentro ningun navegador Chromium (Chrome/Edge/Chromium/Brave)"; exit 1; }
echo "  navegador: $(echo "$CHROME" | sed -E "s|.*/Applications/([^/]+)\.app/.*|\1|")"
[ -d "$SRC" ] || { echo "ERROR: no existe $SRC"; exit 1; }

for f in "$SRC"/*.html; do
  name="$(basename "$f" .html)"
  tmp="$(mktemp -t g1doc).pdf"
  "$CHROME" --headless --disable-gpu --no-sandbox --no-pdf-header-footer \
            --print-to-pdf="$tmp" "$f" 2>/dev/null

  # VERIFICACION antes de sobrescribir: un PDF de 1 pagina cuyo texto empieza por
  # "No se ha podido acceder" es la pagina de error de Chrome -> NO se publica.
  pages=$(osascript -l JavaScript -e "ObjC.import('PDFKit'); \$.PDFDocument.alloc.initWithURL(\$.NSURL.fileURLWithPath('$tmp')).pageCount" 2>/dev/null || echo 0)
  first=$(osascript -l JavaScript -e "ObjC.import('PDFKit'); \$.PDFDocument.alloc.initWithURL(\$.NSURL.fileURLWithPath('$tmp')).pageAtIndex(0).string.js.slice(0,40)" 2>/dev/null || echo "")

  if [ "$pages" -lt 1 ] || [[ "$first" == *"No se ha podido"* ]] || [[ "$first" == *"cannot be reached"* ]]; then
    echo "  FALLO $name (pags=$pages, inicio='$first') -> NO se sobrescribe el PDF existente"
    rm -f "$tmp"; continue
  fi

  mv "$tmp" "$OUT/$name.pdf"
  echo "  OK    $name.pdf ($pages pags)"
done

# El runbook y la hoja de sesion se copian tambien a tasks/ (lo que hay que hacer ahora)
for d in G1_Test_Protocol_Operator_Runbook G1_R1_Session_Checklist; do
  [ -f "$OUT/$d.pdf" ] && cp "$OUT/$d.pdf" "$HERE/tasks/$d.pdf"
done
echo "  (copiados a tasks/ el runbook y la hoja de sesion)"
