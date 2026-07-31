# Rotación de goto.log

`goto.log` es un log **append-only compartido entre el Mac y GPUEDGE**, versionado en git.
Cada run le añade líneas; los merges entre máquinas se resuelven por **UNION**
(desde 2026-07-31 automático vía `.gitattributes`: `goto.log merge=union`).

## El problema

GitHub avisa a partir de **50 MB por fichero** en cada push. El 2026-07-31 goto.log
llegó a ~53-56 MB (main y tutor-feedback-metareasoner) y cada commit nuevo del log
creaba otro blob >50 MB → aviso en cada push.

## Rotación automática (desde 2026-07-31)

`g1_goto.py::_open_goto_log()` se usa en los tres puntos que abren el log (GOTO,
TURNTEST, GOTOVIZ). Al arrancar un run, si goto.log supera **`G1_GOTO_LOG_MAX_MB`
(por defecto 25 MB)**:

1. comprime el log entero a `archive/goto_hasta_<YYYYmmdd_HHMMSS>.log.gz`
   (**gitignored** — el archivo queda SOLO en la máquina que rotó);
2. arranca un goto.log nuevo con una línea de cabecera `=== ROTADO ... ===`.

Si la rotación falla por lo que sea, avisa y sigue escribiendo en el log grande
(nunca bloquea un run).

## Protocolo de coordinación Mac ↔ GPUEDGE

La rotación trunca un fichero que la otra máquina puede tener grande. Con
`merge=union` no se pierden líneas (los appends de la otra máquina sobreviven al
merge), pero si la otra máquina no hace pull, su copia grande se re-mezcla y el
log vuelve a crecer. Tras cualquier rotación:

1. **Máquina que rotó**: `git add goto.log && git commit && git push`
   (el push ya no avisa: el blob nuevo es pequeño).
2. **La otra máquina**, ANTES de su próximo run: si tiene appends locales sin
   commitear, commitearlos primero; después `git pull`. Sus líneas nuevas
   sobreviven al union-merge encabezando el goto.log rotado (no están en el
   archivo .gz de la otra máquina, pero sí en git).
3. Las dos ramas activas (`main` y `tutor-feedback-metareasoner`) llevan cada una
   su goto.log: la rotación se aplica por rama.

## Dónde está el historial

- **En git**: todo lo commiteado sigue en el historial —
  `git log --oneline -- goto.log` y `git show <commit>:goto.log`.
  (Los blobs grandes antiguos siguen en el repo; GitHub solo avisa por blobs
  *nuevos* en cada push, así que tras la rotación los avisos cesan sin
  reescribir historia.)
- **En archive/** (local, gitignored): los .log.gz de cada rotación. Primera
  rotación (2026-07-31, en el Mac):
  - `archive/goto_hasta_20260731_101916.log.gz` (56.8 MB → 5.1 MB): rama
    tutor-feedback-metareasoner, generada por la propia rotación automática en
    el primer run tras el cambio.
  - `archive/goto_hasta_20260731_main.log.gz` (52.2 MB → 5.2 MB): contenido de
    goto.log en la rama main (hecha a mano desde el blob `main:goto.log`; NO es
    prefijo del de tutor — los union-merges reordenaron colas — por eso se
    archivan las dos versiones).

Se decidió NO usar git-LFS: exige instalación/configuración en ambas máquinas y
cuota en GitHub, y el historial en git + los .gz locales ya cubren la trazabilidad.
