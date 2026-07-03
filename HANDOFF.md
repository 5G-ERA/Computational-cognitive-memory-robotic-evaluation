# HANDOFF — G1 ROBOT (navegación A→B + evaluación DCE/meta-razonamiento)

Documento de traspaso para continuar el trabajo en otra cuenta de Claude (misma máquina).
**Cómo usarlo:** abre la carpeta `G1 ROBOT` en la nueva cuenta y pídele a Claude que lea este archivo primero.

_Última actualización: 2026-07-02 (sesión Claude Cowork; ver sección 8). Autor del contexto: Adrian (dev RPA + doctorando en robótica: metacognición y analogías)._

---

## 1. Qué es el robot

- **Unitree G1 "Air"**: humanoide de consumo. **Solo WebRTC** vía la app de iPhone (no hay ROS2/EDU nativo).
- Se controla "pinchando" la sesión web del WebView: `ios_webkit_debug_proxy` (puerto 9221) + CDP por USB, e **inyectando** `rt/wirelesscontroller {lx,ly,rx,ry}` a 20 Hz.
- **Zona muerta ~0.3**: por debajo de |0.3| en un stick el robot NO se mueve. La app usa 0.5–0.73. Todo comando útil va por encima de 0.3.
- **Láser en vivo** = nube `location` (frame del MAPA, Z-up: idx0=x, idx1=y, idx2=altura). Es **muy ruidoso**: LiDAR en la cabeza de un bípedo (vibra con la marcha) proyectado por la pose de relocalización.
- `loc_match` = solape scan-vs-mapa (confianza de localización estimada por nosotros; el firmware no da covarianza).

## 2. Repo y ramas

- GitHub: `https://github.com/5G-ERA/G1_UNITREE_ROBOT_META_REASONING`
- `master` → `origin/main` = versión estable de navegación.
- `feature/escmr-meta-reasoning` = meta-razonador DCE (lanzar con `G1_META=1`).
- `feature/fsm-baseline` = baseline FSM (lanzar con `G1_FSM=1`).

**Flujo git (IMPORTANTE):** la carpeta del Mac es el repo de trabajo; **Adrian hace el push él mismo** (`git push origin HEAD:main`). El robot corre desde un **Ubuntu** que es otro clon (allí se hace `git pull`). Rechazos non-fast-forward frecuentes porque las runs se pushean desde Ubuntu → resolver con `git pull --no-rebase` antes del push. Claude **no** mete credenciales de GitHub: solo commitea y da el comando de push.

## 3. Archivos principales

- `g1_goto.py` — navegación A→B en vivo: A* (tipo firmware) + DWA local, obstáculos de la nube `location`, maniobra de puerta, métricas. **Es el fichero de control.**
- `g1_nav_v2.py` (importado como `g`) — conexión, A*, DWA, costmap, cámara, helpers. `AV_TURN=0.45`, `OCELL=0.2`, `NEAR_BLIND=0.60`.
- `perception_server.py` + `g1_perception.py` — servidor GPU offboard (YOLO + depth) que ve la **mesa invisible al LiDAR** en la puerta. Cliente async (`PerceptionWorker`) para no congelar el control.
- `g1_metrics.py` — métricas SEI: clearance (espacio libre) + progression (avance a B) + sensing reliability.
- `g1_meta.py` (rama escmr) — `MetaController`, analogías (efficient_nav / cautious_nav / payload_sensitive / human_aware), `build_reasoner(ablation)`.
- `g1_fsm.py` (rama fsm) — `FSMBaseline` (árbol de decisión por umbrales).
- `calibrate_cam.py` — calibración de cámara (modo `wall`, sin ajedrez).
- `plot_metrics.py` / `plot_health.py` / `plot_trajectory.py` / `organize_run.py` / `summarize_runs.py` — análisis de runs (dataset/ → `runs_summary.csv`).
- `waypoints.json` — A=(0.99,0.57), B=(−4.73,3.04), C=(−0.03,−1.49); pcd `Qw_20260625`.

## 4. Cómo lanzar una run (con visión ON — necesario para la puerta)

En el **Ubuntu**, tras `git pull`:

```bash
# Terminal 1: servidor de percepción (calibración correcta para frame de 320px)
source venv/bin/activate
python perception_server.py --host 0.0.0.0 --port 8008 \
    --fx 300 --fy 300 --cx 160 --cy 120 --cam-h 1.10 --cam-pitch -10

# Terminal 2: navegación con visión enchufada
G1_PERC=127.0.0.1:8008 python g1_goto.py gotoviz B
```
Al arrancar debe imprimir `[perc] ... -> OK`. En el log, `perc_n > 0` = la visión aporta.

**Requisitos del robot:** relocalizado en la app, batería OK, espacio despejado, kill switch a mano. Pruebas con **agua** (no café).

## 5. Estado ACTUAL de la navegación (dónde estamos)

**RESUELTO — la puerta se cruza.** Era el muro real durante muchas runs. El combo que lo desatascó:
- Banda de alineación de puerta ensanchada 12°→25° (mata la oscilación de yaw; no se puede bajar el giro por la zona muerta) — commit `92030c7`.
- **Visión ON**: el LiDAR ve la mesa/marco del umbral como pared (c0 baja a ~0.09); la visión abre la compuerta de avance (`DOOR-GOv`). Runs `134458` y `135819` cruzaron y llegaron a **0.52 m y 1.12 m de B**.

**Aplicado, PENDIENTE de validar en robot** (2 cambios, uno por fallo observado):
1. `1858520` — **último metro**: por debajo de `DOOR_MIN_GOAL=1.3 m` se desactiva la maniobra de puerta (cerca de B no hay puerta, es el goal con un mueble) para que el DWA **rodee** el obstáculo en vez de empujar recto. (Run 135819 se clavaba a 1.5 m contra una mesa a 13 cm.)
2. `45e5e69` — **filtro de ruido del láser / confiar más en el mapa**: una celda del mapa estático (pared conocida) entra al instante; una celda que **solo** ve el láser necesita aparecer en ≥2 de los últimos 3 barridos (`PERSIST_K=2`, `PERSIST_N=3`). El ruido y la nube desplazada por saltos de reloc parpadean 1 barrido → se descartan. **No añade paredes fantasma** (eso rompió el movimiento antes), solo quita ruido. La visión no se filtra.

**PENDIENTE — fix 2 (siguiente):** guardia anti-divergencia de reloc. En la run `134458` la relocalización explotó (78 `reloc_jump`, `path_m=582 m`, posición final a 538 m). Falta re-añadir el guardia que para (STOP) cuando la reloc salta repetidamente y no integra el salto. (Se fue en el rollback a la versión de ayer.)

**Qué mirar en el próximo log:** `obs=` que no crezca solo (ruido acumulado); cerca de B que rodee y no se clave; `DOOR-GOv` en la puerta; si la reloc salta, que NO aparezcan paredes nuevas de golpe.

## 6. Gotchas aprendidos (no repetir)

- **NO hacer el mapa autoritario para AÑADIR paredes.** El `map_full.json` del G1 está desalineado (el waypoint A cae sobre una pared ~0.08 m) → metía paredes fantasma en el arranque y el robot no se movía. Confiar en el mapa solo para **rechazar ruido**, nunca para inventar obstáculos. Por defecto `G1_REFMAP="summit"` (el mapa alineado).
- **Un cambio cada vez y validar en el robot** antes del siguiente. Meter varios cambios juntos nos costó un día (un fix tapaba el fallo del otro).
- Cámara: para frame de 320 px usar `cx=160 cy=120 fx=300 fy=300` (NO 320/240, que es de 640 px).
- BrokenPipe en el server: cliente async + warmup del modelo + el server ignora BrokenPipe.
- YOLO falsos positivos → `--det-conf 0.45`.
- Ventana cv2 no va en SSH/headless → endpoints `/debug.jpg` y `/debug.mjpg` en el navegador.

## 7. Contexto de investigación (el "para qué")

Paper del tutor sobre **DCA/DCE** (Decentralised Capability Abstraction/Ecosystem). `ESCMR` (`ExperienceScopedCapabilityMetaReasoner`) = forma runtime del Cap.5 de la tesis (razonamiento por analogías): vectores de atención, zonas semánticas de QoE, tensión de desplegabilidad, creencia/plausibilidad Dempster-Shafer, decisiones keep/switch/fallback/help/insufficient.

**Evaluación buscada:** DCE (meta-razonador) vs FSM baseline + ablaciones, en el robot real, cruzando la puerta con distintas condiciones (payload sin tapa, humano cerca, batería baja). Métricas instrumentadas: clearance, progression, sensing reliability, spill ground-truth (un humano marca con Enter cuando se derrama agua), colisiones, salud por articulación. `summarize_runs.py` junta `dataset/*.json` en `runs_summary.csv` (rellenar a mano `condition` y `notes`).

---

## 8. Sesión 2026-07-02 — hallazgos y estado (MEMORIA: leer antes de tocar nada)

### ⚡ TL;DR — ESTADO ACTUAL (si solo lees una cosa, lee esto)

- **Funciona**: A→B llega en 72–84 s, 0 colisiones, sin modo agresivo (run 130231). Nube láser a
  ~2.2 Hz. Canal de moqueta VIVO en el server (`G1_FLOORCOLOR=1`, verificar `floorcolor=ON` en el
  PERC-TEST del arranque). Strafe con signo corregido (mapeo gamepad: lx>0=DERECHA; default -1).
- **VALIDADO EN ROBOT (noche 07-02)**: plan global sobre mapa ESTÁTICO (`G1_GLOBALMAP=static`,
  P8b/8.12) + **ENGAGEMENT de puerta** (`G1_DOOR_ENGAGE=1`, P2c/8.13: pre-entrada → parar →
  alinear ±8° al eje 135° → cruzar recto). Adrian: "ha funcionado muy bien" — puerta cruzada
  limpia. ES LA CONFIGURACIÓN ACTUAL (defaults ON). No tocar sin A/B.
- **META2 (07-03)**: Meta-Reasoner 2.0 integrado (`G1_META2=1` shadow / `=2` activo, 8.14) y
  VALIDADO en shadow (decisiones coherentes, FALLBACK precede a las colisiones). Tras la run
  100927 (584 s de bucle en puerta): **escalada de experiencia** `G1_M2_ABORT=1` (HELP≥8 s o
  ventana mala sin progreso → ABORT; replay: t=94 s vs 584 s, 0/6 falsos) y **canal de
  resistencia `mobility`** con veto duro (8.15). Modo y entorno auto-registrados (8.16:
  `meta2_mode`/`meta2_capped_ticks`, `G1_ENV=real|sim`+`G1_SIM_ID`). PENDIENTE: primera run
  confirmada en `G1_META2=2` y arrancar la campaña de simulación del tutor.
- **Bugs cerrados hoy (con mediciones)**: dedup de barrido · clamp 0.4→0.7 (NEAR_BLIND se comía los
  avisos) · signo del strafe · anti-jaula (clamp solo central+alto; visión por score, sin bypass).
- **PENDIENTE de validar en robot (en orden, UNA por run)**: ① B→A con fix anti-jaula
  ② `G1_HARDGUARD=1` (paredes no negociables — idea de Renxi) ③ `G1_HBAND_LO=-0.7` (objetos bajos).
- **Herramientas**: `python autopsy.py dataset/<run>.json` → informe HTML completo (trayectoria,
  timelines, eventos, fotos). `summarize_runs.py` → CSV comparativo. Ventana live: server `--debug`
  + navegador en `http://IP:8008/`.
- **Reglas operativas**: git pull en el Ubuntu ANTES de lanzar · push siempre `origin HEAD:main` ·
  un cambio por run · nunca subir PERSIST_K · el color nunca resta obstáculos (fusión por unión).

### 8.1 Cambios aplicados a `g1_goto.py` (compilan; PENDIENTES de validar en robot)

1. **Dedup de barrido fresco**: `reloc_cells.fresh` (hash del buffer `__relocbuf`). La persistencia y el
   score/decay solo votan con nube NUEVA — antes el mismo barrido leído 2-3 ticks se autoconfirmaba solo
   en el filtro 2-de-3. `SCAN-STALE` en el log si la nube se congela ~3 s.
2. **Diagnóstico en vivo** (solo observación): log `nz=` (laser_noise) `flt=` (fracción del barrido rechazada
   por persistencia) `dmap=+A/-R` (churn del mapa activo) `shz=` (Hz real del topic location). Summary por
   run: `laser_noise_mean/max, filt_rej_mean, scan_hz, stale_pct, gated_pct, safer_inserts, map_adds/dels,
   obs_max, reloc_jumps, tick_ms_p95`. `summarize_runs.py` con esas columnas (runs viejas en blanco).
3. **Guardia anti-divergencia de reloc** (fix 2 del handoff, re-añadido): ≥4 saltos/10 s → STOP + aborta +
   nube postmortem `_relocdiv`. Validado offline vs run 134458: paraba a 0.9 s del inicio de la divergencia
   (los 78 saltos fueron TODOS a partir de t=273.9 s, uno por tick; 44 m→573 m de path en los últimos 26 s).
   0 falsos positivos en runs sanas. `G1_RELOCGUARD=0` desactiva; `G1_RELOC_N/WIN` ajustan.
4. **Gate DURO de visión**: al arrancar, test real frame→`/perceive`; sin YOLO verificado NO navega
   (`PERC-GATE BLOCKED`). Override consciente `G1_NOVIS=1`. Motivo: colisiones del 07-01 (164306/164456)
   fueron con c0=2.50 y perc_n=0 (visión apagada + gate v1 por yaw medido congelaba el mapa andando recto).
5. **`PERSIST_N/K` por entorno** (`G1_PERSIST_N/K`, defaults 3/2) para A/B en campo sin editar código.

### 8.2 Investigación del laser noise (2026-07-02, con fuentes)

- El Mid-360 usa **escaneo NO repetitivo** (Livox): cada frame muestrea direcciones DISTINTAS. El parpadeo
  celda-a-celda es INHERENTE al sensor, no solo ruido de marcha. ⇒ integrar barridos (persistencia/score)
  es la forma correcta de consumir un Livox. **REGLA: nunca subir PERSIST_K** (retrasa obstáculos finos
  reales que también parpadean); si el ruido gana, subir N manteniendo K=2.
- Precisión del sensor (≤2 cm @10 m, <0.15°) es sub-celda: las celdas falsas vienen de la MARCHA BÍPEDA
  (pitch/roll de cabeza + pose de reloc retrasada), confirmado por literatura de humanoides 2025.
- Sin timestamps por punto ni IMU vía WebRTC no se puede hacer deskew "de libro" (FAST-LIO): el filtrado
  temporal a posteriori es LA única familia de soluciones disponible. Nuestro score hit/miss ≈ occupancy
  grid log-odds (Thrun) y el decay sin raycasting ≈ Spatio-Temporal Voxel Layer de Nav2 — práctica estándar.
- Livox: detección NO garantizada a 0.1–1 m en superficies oscuras/pulidas/finas → la mesa LiDAR-ciega
  tiene explicación de fábrica. El Mid-360 del G1 va además montado físicamente invertido (repo deepglint).

### 8.3 Visión por color de moqueta (`g1_floorcolor.py` + `floorcolor_calib.json`, en main SIN conectar)

- Moqueta del lab azul-gris uniforme. Modelo HSV mediana+MAD (S y V discriminan; hue casi ruido en baja
  saturación). Calibrado con `crash_01_151142` (moqueta pura).
- **Validado con las 148 imágenes de crashes/**: 73% BLOQUEADO en el momento del choque (mediana
  free_center=0.01). De los 40 "libre": 15 = serie `1512xx` (obstáculo FUERA del FOV de la cámara),
  ~19 colisiones laterales (vista frontal genuinamente libre), ~6 borderline con el obstáculo YA marcado
  en rojo. **0 fallos claros del clasificador**; sombras apenas dan falsos rojos (lado conservador).
- **Umbrales validados para integrarlo**: veto con `free_center < 0.45` o `near_run ≥ 5`.
- Ve CABLES y muebles que YOLO no reconoce, a coste CPU. Es COMPLEMENTO de depth+YOLO, no sustituto.
- Hallazgo para la tesis: 15 choques repetidos con vista limpia ⇒ ningún sensor de cabeza cubre el campo
  cercano bajo ⇒ la detección de contacto (IMU) es una capability de pleno derecho para el DCE.
- Plan: tras validar la run de hoy, integrarlo en `perception_server.py` tras flag `G1_FLOORCOLOR=1`
  (default OFF) para A/B limpio con/sin color → ablación extra para el paper.
- **MULTI-MODO (2026-07-02 tarde)**: la MISMA moqueta cambia con la exposición de la cámara: oficina
  iluminada = gris lavado (S~11), pasillo oscuro = azul saturado (H105 S~91 V~86, medido en
  crash_03_162351). Con 1 modo el color VETABA el cruce de la puerta (falso bloqueado). Calibración
  final: modo1 med/mad + modo2 límites explícitos H105±6, S[35,140], V[64,120]. Validación: 98/148
  bloqueadas (66%), puerta free=0.63 ✓, mesa free=0.00 ✓.
- **LÍMITE ESTRUCTURAL descubierto**: bajo la mesa hay moqueta real en sombra → el color la ve como
  suelo (correcto cromáticamente). Los VOLADIZOS (mesa) son trabajo de depth+YOLO. REGLA DE FUSIÓN:
  obstáculo si CUALQUIER canal lo dice (unión); el color nunca resta obstáculos de otros canales.
- **DETECTOR DE PUERTA `find_door()`** (sin entrenamiento): corredor de moqueta profunda flanqueado
  por verticales BLANCAS (marco S<60 V>150) → bearing_deg del vano + width_frac. Probado en
  crash_03_162351: detecta con ambos flancos. Uso previsto: rumbo ESTABLE para DOOR-AL (el `ddir`
  del A* tiembla con el láser y hacía oscilar la alineación). Entrenar YOLO con las 10 fotos de
  iPhone (images_iphone/door): NO merece la pena (dataset mínimo + salto de dominio); si se quiere
  aprendizaje, etiquetar frames de la cámara del robot y fine-tune yolov8n en el Ubuntu.

### 8.4-bis VALIDADO EN ROBOT (run 20260702_113327, 11:33)

**PRIMERA LLEGADA LIMPIA A B**: reached en 71.8 s, 13.2 m (eficiencia 0.48), **0 colisiones,
0 saltos de reloc**, c0min=0.33. Ayer el mejor intento no llegó (0.84 m a falta, 300 s, 30.6 m, 1 col).
Diagnósticos clave: **scan_hz=2.17 / stale_pct=30.5%** → la nube refresca MÁS LENTO que el tick:
el dedup era CRÍTICO (cada barrido votaba ~1.4x sin él). filt_rej=8% (filtro suave con votación
honesta), obs_max=421 (vs 788 ayer), dmap quieto +6.5/-1.9 (residual, aceptable), gated_pct=27%
(vigilar: umbral de alarma 30%), tick_ms_p95=347. Visión activa (179 queries; la mesa entró por
depth+SAFE_R=234 inserciones, YOLO nunca dijo "table" → otro argumento pro-floorcolor).
Combo dedup+score/decay+gate-rx+SAFE_R+gate de visión: **VALIDADO**.

### 8.4-ter Run 114603 (11:46) + INTEGRACIÓN FLOORCOLOR

Run 2: reached en 142 s / 20.5 m / **1 colisión** (t=9.1 s, cajonera de madera, roce lateral derecho):
c0=0.7 (mapa decía hueco), **perc_n=0 y dets=None** — ni depth ni YOLO la vieron (sin clase para
cajoneras). El canal de COLOR la veía entera (6/16 columnas derechas a 0.0). gated_pct subió a 35.7%
(>30%: vigilar RX_GATE). scan_hz≈2.2 confirmado otra vez.

**INTEGRADO el canal de color en `perception_server.py`** tras `G1_FLOORCOLOR=1` (default OFF, A/B):
- `color_to_scan()`: por columna, fila donde acaba la moqueta continua = base del obstáculo →
  proyección al suelo → [bearing, range]; obstáculo tocando el borde inferior → clamp 0.4 m.
- Bandas no-moqueta FINAS con moqueta encima (umbral de madera de la puerta, cinta amarilla) se
  SALTAN: son marcas planas, no obstáculos (sin esto el umbral tapaba el vano con fantasmas a 0.4 m).
- FUSIÓN POR UNIÓN: el color solo añade puntos al scan; nunca resta. `door` (find_door) y `color_pts`
  van en la respuesta (el cliente actual los ignora; DOOR-AL por visión = mejora futura).
- Validado offline: cajonera 11 pts ✓ (habría evitado el choque), puerta limpia (solo base de la
  hoja) ✓, mesa 48 pts ✓, moqueta pura 0 falsos ✓.
- OJO para revisar algún día: en `depth_to_scan` la fórmula de altura con cam_pitch=-10 parece
  inflar la altura con la distancia (¿convención de signo invertida?). El canal de color usa
  abs(pitch) y es inmune. Puede explicar el perc_n bajo a 1-2 m.

**A/B**: misma run B, servidor con y sin `G1_FLOORCOLOR=1` → comparar collisions, path_m, time_s,
perc_n medio y comportamiento en puerta (runs_summary.csv). Es la ablación extra del paper.

### 8.5 CIERRE DEL DÍA 2026-07-02 (8 runs) — estado FINAL VALIDADO

Run final **130231**: reached 84 s / 13.0 m / eficiencia 0.49 / **0 colisiones / SIN modo agresivo**
(primera vez que cruza la puerta con holgura normal) / gated 43%→21%. Tres bugs de campo cerrados
HOY con mediciones:
1. **Dedup de barrido** (la nube location refresca a ~2.2 Hz < tick; sin dedup cada barrido votaba ~1.4x).
2. **NEAR_CLAMP 0.4→0.7** (los avisos cercanos del canal de color morían en el anillo NEAR_BLIND=0.6).
3. **DOOR_STRAFE_SIGN default -1**: el mapeo físico de lx es tipo gamepad (lx>0 = DERECHA), medido en
   3 runs con ambos signos (123933: 46 órdenes izq → 38 cm dcha; 125209: ambos signos consistentes;
   130231 con fix: 37/48 strafes hacia el lado libre). DOOR-CTR llevaba centrando HACIA el obstáculo.
   Verificable en vivo con STRAFE-CAL / STRAFE-CAL-RESUMEN en goto.log.
OJO al orden operativo que costó una run: en el Ubuntu **git pull ANTES de lanzar** (la 125209 corrió
con código viejo y pareció refutar el fix del strafe).

### 8.6 Bug de la JAULA (run 130524, B→A atascada) — arreglado

Con el canal de color VIVO, en la sala abarrotada de B el robot se ENJAULO solo: clamp de 0.7 m en
todo el FOV + pivotar buscando ruta = anillo de celdas sinteticas alrededor (holgura ~0.6-0.7 m EN
TODAS las direcciones, medido) → A* sellado → SEEK → mas pivoteo. Fix doble: (1) el clamp del server
solo en columnas CENTRALES (±30°) y con obstruccion ALTA (≥35% de la columna) — es para "me lo como
andando", no para clutter lateral que el laser ya ve; (2) la VISION ya no salta el gate por SAFE_R:
pasa por el score normal (+1/frame, ~1 s para entrar; mientras PIVOTA no inserta nada → jaula
imposible). Solo el laser confirmado mantiene el bypass de seguridad. Regresion offline OK
(escritorio/pared frontales siguen detectandose; vano y moqueta pura sin fantasmas).

### 8.7 Instrumentación total + capa de confianza (tarde 2026-07-02)

- **Telemetría completa** por tick en samples: canal de color (color_pts/carpet_pct/color_near/
  color_rmin/door_b), plan (carrot/goal_err/carrot_err/plan_n=0 si A* sin ruta), confianza
  (c0_hard/n_hard). Eventos al dataset: astar_fail, aggressive_on. PELÍCULA: frame cada 3 s
  (G1_FILM, tNNNs.jpg). Colisiones: pre-frames t−1/−2/−3 s + omap_near en el evento.
- **Capa de ALTA CONFIANZA (Renxi)**: hard_set = refmap confirmado | score saturado | colmap.
  HARD-GUARD tras `G1_HARDGUARD=1` (default OFF): <0.45 m de pared → lento; <0.22 m → corta avance,
  incluso en agresivo. Lo blando sigue negociable (la puerta necesita 0.13).
- **Objetos BAJOS**: ciegos para el láser por HBAND_LO=-0.5 (anti-suelo). Los cubren depth (0.10 m+)
  y el canal de moqueta (a ras de suelo). A/B pendiente: `G1_HBAND_LO=-0.7` con el stack anti-ruido.
- Overlay live: rojo 60 % (sobre muebles oscuros el 28 % desaparecía), verde 22 %.
- ORDEN DE VALIDACIÓN: ① B→A con ESCAPE (8.10) + calib v3 ② G1_HARDGUARD=1 ③ G1_HBAND_LO=-0.7. Una cosa por run.

### 8.8 Runs 142725/143039 (instrumentación completa estrenada)

- A→B **perfecta** (70 s, 0 col) CON canal de color vivo (carpet 0.43, color_pts 13).
- B→A atascada 39/54 s: **NO es el canal ni la jaula** (plan_n siempre >0, 0 astar_fails, el color
  decía la verdad: carpet 0.06 en el choque porque la cámara estaba EMPOTRADA en el sofá crema).
  Causa: **el waypoint B aparca al robot de morros contra el sofá** en un bolsillo (sofá+cajas+cajonera);
  la vuelta arranca sin sitio para girar. FIX operativo: re-grabar B ~0.5 m hacia espacio abierto
  (menú `waypoint B`). Adrian prefirió NO mover B → maniobra de ESCAPE implementada (ver 8.10).
- Nota técnica: el filtro "central ±30°" del clamp es un no-op (FOV de la cámara = ±28°); el que
  discrimina es el de obstrucción alta (≥35 % de columna). No tocar salvo que moleste.

### 8.9 PRINCIPIO RENXI: el LiDAR decide, la visión apoya (runs 143511/143646)

- **Bug cazado con datos**: en la boca de la puerta el marco/hoja llenan la cámara (carpet 3 %) →
  45 columnas CLAMP a 0.7 m → muro fantasma de color cruzando un vano que el LÁSER veía pasable
  (c0 0.7–0.85) → el robot, perfectamente alineado (goal_err −2°), giraba 180° y huía. Mismo
  mecanismo estrangulaba la salida del bolsillo de B.
- **Fix (principio de Renxi)**: los puntos CLAMP ("lo tengo encima", sintéticos) YA NO entran al
  mapa — van aparte en la respuesta (`clamp`) y solo MODERAN la velocidad (≤0.24 si ≥8 columnas
  clampeadas). Los puntos PROYECTADOS (base visible en el suelo, geometría real) siguen entrando.
  La visión nunca veta un paso que el láser ve libre; sí obliga a acercarse despacio.
- **Limitación descubierta**: el SOFÁ CREMA lee como moqueta (modo 1 con hue sin acotar: cualquier
  superficie pálida poco saturada pasa). Falso-pisable ACOTADO: el sofá no es LiDAR-ciego y la
  fusión por unión no borra obstáculos del láser. TEXTURA como discriminador: probado y DESCARTADO
  (mediana |Laplaciano|: moqueta 4.7–5.2 vs sofá 3.7 — margen fino que el motion blur invierte).
- Para la tesis (comentario de Renxi): "stuck real vs fake" = el caso de arbitraje de capacidades
  del DCE; este fix es la versión cableada de la política que el meta-razonador debería decidir.

### 8.10 Maniobra de ESCAPE al arranque (implementada — Adrian prefirió no re-grabar B)

- **El backlog de 8.8 tal cual estaba especificado NO habría funcionado**: el disparo por láser solo
  (c0<0.45 en tick 0) no salva ninguna de las 3 vueltas fallidas. El sofá a 40 cm cae en la banda
  ciega del Mid-360: en 150440 el mapa decía c0=1.92 "libre" y solo cayó a 0.22 EN el golpe (t=4.4 s);
  en 143039 nunca bajó de 1.0. La cámara SÍ lo ve: enterrada en el sofá lee carpet_pct 0.00–0.43
  (el sofá crema clasifica como moqueta e INFLA el valor → umbral 0.50, no 0.12); los arranques
  buenos leen 0.86–0.95.
- **Disparo** (G1_ESCAPE=1 por defecto; ESC_TRIG/ESC_CARPET/ESC_DIST por env): primeros 5 s
  + sin haberse movido (<0.30 m del arranque — mata el falso positivo de 142725: carpet 0.46 en
  t=4.7 s tras andar 1.5 m hacia la puerta, run perfecta) + d_goal>1.5
  + (**c0<0.45 o carpet_pct<0.50**). Una sola vez por run.
- **Maniobra**: retroceso recto ly=−0.35 (deadzone ~0.3) hasta 0.5 m / 6 s / rear<0.50 / despejado.
  "Despejado" exige AMBOS sensores contentos (c0≥0.85 Y carpet≥0.55 si hay visión): con c0 solo,
  el escape por visión acabaría en el primer tick con la nariz aún en el cojín. Fase `ESC-BK`,
  eventos `escape_start`/`escape_end`. Va ANTES de RECUPERACION/DESATASCO (una colisión real sigue
  mandando); HARD-GUARD y el moderador Renxi no interfieren (solo actúan sobre cmd[1]>0).
- **Replay sobre las 26 runs con dataset**: dispara SOLO en las 3 vueltas del bolsillo
  (143039 / 145306 / 150440, todas en t=0.6 s, ANTES del primer golpe) y en ninguna más.
- Para la tesis: segundo caso cableado del arbitraje "stuck real vs fake" de Renxi — el LiDAR dice
  "libre", la visión dice "enterrado", y la visión gana SOLO para una maniobra acotada (0.5 m
  atrás antes de planificar), nunca para pintar el mapa.

### 8.11 Lote de carga (2026-07-02 tarde) + SIMULADOR de fixes

- **PLAN GLOBAL sobre mapa ESTABLE (hallazgo de Adrian, crítico)**: el A* global planificaba
  sobre el láser VIVO (`build_costmap(oset)`) y solo caía al mapa cargado como último recurso.
  Efectos medidos con `sim_globalplan.py` (nube postmortem real de 150440 + nav_map.json real):
  ① seis "frames" Livox desde la misma pose → caminos que difieren 0.27 m de media (el ddir
  tembloroso que DOOR-AL peleaba); ② el plan vivo ATRAVIESA paredes mapeadas que el láser no ve
  en ese instante ("va recto a la pared y gira tarde"). Fix: `G1_GLOBALMAP=hard` (defecto) =
  mapa cargado + persistentes saturados + colisiones; DWA local sigue con el láser vivo;
  agresivo replanifica con lo reciente. `=live` revierte, `=ref` solo mapa.
- **HARD-GUARD ON por defecto**: replay de 29 runs → 0 activaciones en runs limpias (coste
  cero) y 4 STOPs pre-colisión en 150440. Paredes no negociables (Renxi).
- **G1_AGGR_R 0.13→0.20**: semiancho físico con brazos ~0.28-0.30 m; 0.13 autorizaba huecos
  imposibles (roces con omap_near 42/48/81 en 152030/152330/152532).
- **PRESS-guard: RECHAZADO por simulación** (resultado negativo honesto): la firma pre-impacto
  real (3-4 s raspando a 0.05-0.15 m/s con cnear 21-28) es INDISTINGUIBLE del cruce cuidadoso
  de puerta con solo odom+visión (6 falsos en 145010, 3 en 142725, ambas limpias). Se retomará
  con el par de pierna (legtau, ya logueado) en `g1_replay.py` antes de tocar el robot.
- **Herramientas nuevas**: `g1_replay.py` (replay contrafactual de reglas sobre todas las runs;
  regla de aceptación: dispara antes de los fallos que ataca + 0 falsos en runs limpias) y
  `sim_globalplan.py` (estabilidad del plan global, vivo vs estático). ESCAPE pasó 4/4 + 0/25.
- **perc_age** en samples y en la línea [VIS]: edad de la última respuesta real del server de
  percepción (separa "server colgado" de "escena vacía legítima" — P3 del tutor).
- Runs 152030/152330/152532 (pre-lote): PRIMERA B→A completada (152532, 81.6 s) — el pocket
  sigue costando 1 colisión al salir; A→B con 1 roce en boca de puerta cada una. La lista de
  problemas del tutor vive en `PROBLEMS.md` (inglés).

### 8.12 P8b — plan GLOBAL sobre el mapa estático COMPLETO, con muebles BLANDOS (tarde 07-02, sesión Renxi)

- **Herramienta nueva `g1_get_static_map.py`** (local | webview | pcd): exporta ENTERO el mapa
  estático a maps_out/ (.json/.pcd/.png). `local` = refmap+nav_map (lo que ve el plan global);
  `webview` = mapa cargado capturado de la app (hook MAPGRAB standalone); `pcd` = probe de
  descarga del .pcd del robot vía api 1934 (protocolo de chunks sin confirmar; imprime crudo).
- **Descubierto reproduciendo offline con ese mapa** (¡con el A* exacto del repo!):
  ① el modo `hard` (P8 v1) NUNCA planificaba de verdad: con INFL_HARD=1 la puerta (~4 celdas,
  0.8 m) queda sellada al inflar → el A* fallaba en CADA replan y caía al fallback sin inflar
  (solo paredes, sin hard_set); ② nav_map tiene celdas acumuladas de marco/hoja EN la boca de
  la puerta (x[-3.6,-3.2] y[0.8,1.6]) que la sellan si se tratan como pared dura (la única ruta
  A→B que quedaba era FANTASMA, por un hueco sin mapear arriba en y≈+7).
- **Fix `G1_GLOBALMAP=static` (nuevo DEFECTO)**: paredes = inf SIN inflar; muebles conocidos
  (nav_map) + persistentes saturados + colisiones = coste BLANDO 9.0 + halo 4.0
  (`G1_GLOB_SOFT`/`G1_GLOB_HALO`). El plan rodea lo conocido pero JAMÁS sella un paso; el DWA
  local sigue mandando en seguridad. Validado offline: A↔B/C→B cruzan SIEMPRE la puerta real
  (34 celdas), plan determinista, B→A pisa 1 celda de mueble vs 4. Revertir: `G1_GLOBALMAP=hard`.
  Marcador de log al arrancar: `GLOBALMAP src=... walls=... static=...`.

### 8.13 P2b+P2c — holgura y ENGAGEMENT de puerta (noche 07-02, sesión Renxi)

- Run 171431: **PRIMERA llegada a B con plan estático** (87.8 s / 13.2 m) pero 2 roces de hombro
  dcho en el marco + mano dcha en la cajonera (-1,0.5) que el IMU ni vio. Firma medida en ambas
  colisiones: 2-3.5 s EMPUJANDO (comando adelante, cuerpo <0.08 m/s) antes del IMU → material
  para el press-guard con legtau. **P2b**: `G1_AGGR_R` 0.20→0.24 + halo blando de pared en el
  plan global (`G1_GLOB_WHALO=6.0`): celdas del plan pegadas a obstáculo 19→7, sigue cruzando.
- Runs 172840/173422: hombro OTRA VEZ en (-3.75,1.22), yaw de impacto 101-119° vs eje ~135° →
  entra under-rotated. **P2c (pedido por Renxi/Adrian): ENGAGEMENT anclado al mapa estático**
  (`G1_DOOR_ENGAGE=1`, centro/eje `G1_DOOR_X/Y/AXIS` = -3.90/1.25/135): pre-entrada a 0.85 m en
  el eje → PARAR → alinear a ±8° → cruzar RECTO a 0.28 re-alineando si deriva >14°; strafe al
  eje si se descentra >0.14 m. Sale a 0.75 m pasado el centro; vale A→B y B→A. Abortos acotados
  (bloqueo/timeout/presión) → lógica normal con 8 s de cooldown. Replay offline en las 5 runs
  de hoy: dispara a ~1.7 m del vano, 12-20 s ANTES de cada colisión real, 0 misfires.
  Fases nuevas en el log: ENG-T/F/WT/AL/AL./RE/C/GO + eventos door_engage/door_crossed.
- **VALIDADO EN ROBOT (misma noche)**: la secuencia engagement corrió en el robot y la puerta se
  cruzó limpia ("ha funcionado muy bien" — Adrian). P8b+P2b+P2c quedan como configuración de
  referencia. Falta hacer git pull de la run buena para anotar id/tiempos/colisiones aquí y en
  runs_summary.csv, y repetir B→A para validar el engagement en dirección de vuelta.

### 8.14 META2 — Meta-Reasoner 2.0 (DCE runtime de Renxi) integrado tras flag (noche 07-02)

- **Paquete de Renxi en el repo**: `meta-reasoner-2.0/` (reasoner configuration-first: DST por
  analogía → región semántica con memoria/dirección → tensión → fulfillment → gates → KEEP/
  SWITCH/FALLBACK/HELP/INSUFFICIENT; 11 tests OK). Guía completa en su .docx; contrato: SWITCH
  ⇒ `switch_to` no nulo. Ablaciones por config (`evaluation_controls`: DST analogía/tarea on/off).
- **Puente `g1_meta2_bridge.py`** + **`config_meta2_g1door.json`** (QoE calibrada con las
  distribuciones reales del 07-02: crucero clear~1.0/prog~0.48; puerta clear p25 0.43/prog 0.10;
  OJO pre-colisión clear mediana 1.0 → safety no puede ser solo láser). Mapeo: safety←clearance,
  progression←progression SEI, reliability/uncertainty←SensingMonitor+laser_noise, bat←telem.
  Bridge-side (el reasoner NO se toca): mediana de 5 muestras, persistencia de switch (3) y de
  acción (2), switch_margin 0.08, retorno al preferido en empate, warmup 4. Analogías v1:
  Efficient_Nav (sin techo) / Cautious_Nav (techo 0.28); FALLBACK→0.24, HELP→avance 0.
- **En g1_goto tras `G1_META2`**: `=1` SHADOW (decide y loguea `[META2]` + eventos + campos
  meta2_* en samples, no toca control) · `=2` ACTIVO (techo de avance por analogía, solo
  cmd[1]>0, marca `!M` en la fase; recuperaciones/ESCAPE intactos) · `=0`/ausente OFF (defecto).
- **Replay offline** (`python g1_meta2_bridge.py dataset/<run>.json`): en 171431 → 3 switches
  coherentes (Cautious en arranque, Efficient en pasillo, Cautious al llegar al clutter) y
  bandas FALLBACK en los atascos de puerta que PRECEDEN a las colisiones reales (13-18s y
  33-37s vs impactos en 47.6/60.0). Sintético: pasillo KEEP / puerta SWITCH / unsafe HELP.
- **Plan de validación**: ① una run con `G1_META2=1` (shadow) y comparar timeline META2 vs
  eventos reales (autopsy) ② si cuadra, `G1_META2=2` ③ ablaciones DST para el paper (4 modos)
  contra la rama `baseline` congelada. Afinar QoE con el flujo de calibración del paquete
  (calibration_meta_reasoner_2_0.json como plantilla, casos etiquetados de nuestras runs).

### 8.15 P9 escalada de experiencia + canal de RESISTENCIA (mañana 07-03, feedback del supervisor)

- **Run 100927 (B→A): 584 s de bucle en la puerta, 3 col, aborto manual** — con META2 diciendo
  FALLBACK 68% + HELP 14% durante minutos. Supervisor: "the experience should inform the robot
  that all actions are not good and abort". Tenía razón: el reasoner concluía bien, nadie
  escalaba. **Fix `G1_M2_ABORT=1`**: HELP firme ≥8 s O ventana de 75 s con ≥60% FALLBACK/HELP
  y <0.4 m de progreso → en ACTIVO aborta la run (STOP + `aborted_meta2_help`); en SHADOW avisa
  (`META2-ABORT-SHADOW`). Replay: dispara a t=94 s en la 100927 (ahorra 8+ min), 0/6 falsos.
- **Canal `mobility` (P2d)**: velocidad real/comandada en 1.2 s — la "resistance" que faltaba
  (colisiones con clearance=1.0: el marco no se ve, se SIENTE). Meta-parámetro con HARD VETO,
  atención de tarea 0. QoE conservadora (dangerous 0.08-0.12), calibrar con datos de robot.
- **Instrumentación de modo** (la pareja de runs 10:07 no se pudo verificar como activo: el
  goto.log del repo no traía la sesión y el sample graba la fase PRE-moderación, así que `!M`
  nunca aparece en dataset): ahora cada run emite el evento `meta2_mode` y el campo
  `meta2_cap` por sample → el dataset se autodescribe. PENDIENTE: confirmar en el Ubuntu
  (`grep "META2 mode" goto.log`) si las 100739/100927 corrieron con mode=1 o 2.
- Runs limpias del día (shadow): A→B 54.9/57.1/71.6 s 0 col · B→A 71.7 s 1 col y 108.2 s 0 col
  (primera B→A sin colisión). El engagement B→A funciona pero el marco derecho sigue costando
  presión: el canal mobility es la respuesta de gobernanza; el micro-ajuste `G1_DOOR_AXIS≈-48`
  para BA queda como A/B de campo.

### 8.16 Registro de MODO y de ENTORNO (sim vs real) — mañana 07-03, tras la duda de las 10:07

- **Modo META2 a prueba de dudas** (las runs 100739/100927 no se pudieron verificar como
  activo/shadow): ahora queda en 4 capas independientes — ① cabecera del dataset
  (`meta2_mode`, `meta2_enabled`), ② summary (`meta2_mode` + `meta2_capped_ticks` = ticks
  realmente capados; >0 ⟺ activo ACTUANDO), ③ evento de flanco `meta2_cap_on` + línea
  `META2-CAP ON` en goto.log cada vez que un techo empieza a recortar, ④ columnas
  `meta2_mode`/`meta2_capped_ticks` en runs_summary.csv. Regla de lectura: mode=2 con
  capped_ticks=0 = activo pero nunca hizo falta capar; mode vacío = run pre-META2.
- **Campaña de SIMULACIÓN (pedido del tutor)**: nuevo `G1_ENV=real|sim` (defecto real) +
  `G1_SIM_ID=<etiqueta del contenedor/escenario>`. Se marca en el RunRecorder → lo heredan
  TODOS los tipos de run. Va en cabecera del dataset (`env`, `sim_id`), en la cabecera de run
  del goto.log (`env=sim/...`), aviso en consola ("no cuenta como run de robot") y columnas
  `env`/`sim_id` en el CSV (filtrar env=real para la tabla del paper). Runs viejas = real.
- **Estado de la sim (07-03, contenedor inspeccionado)**: imagen tipo Tiryoh docker-ros2-desktop-vnc
  (`g1sim:humble`, id 48e7c50b26d6): ROS2 **Humble Desktop completo + escritorio MATE por noVNC**,
  con **Gazebo, Nav2 completo, slam_toolbox, foxglove_bridge, pointcloud_to_laserscan** ya
  instalados. Acceso web VERIFICADO: `http://localhost:6080/vnc_lite.html` (pass `ubuntu`);
  el 5900 es VNC crudo (no HTTP), y el **8765 (foxglove_bridge) ya está mapeado** → no hace
  falta relanzar el contenedor para el adaptador. Volcados en `sim/container_dump/` (00/01 ok;
  02 salió vacío por profundidad del find). PENDIENTE: dump 03/04 (workspace + bash_history =
  cómo se lanza la sim), topics/nodos CON la sim corriendo, y `docker cp` del workspace a
  `sim/ws_src`. **ADAPTADOR IMPLEMENTADO** (`g1_sim_adapter.py`, tests offline OK): SimCDP que
  resuelve los snippets JS del stack contra ROS2 via rosbridge:8765 (instalado en el
  contenedor) — /odom→pose, /scan 360°→nube location, __cmd→/cmd_vel con la física calibrada
  del real (deadzone 0.3, signos de strafe/giro medidos). Escenario sim aislado en sim/
  (waypoints_sim, ref_map_room generado del .world, nav_map_sim) sin tocar los ficheros del
  lab; sim.launch.py con `gui:=false` para headless. Lanzamiento completo en sim/README.md.
  PENDIENTE: primera run `python g1_sim_adapter.py gotoviz B` (A→B esquivando el pilar) y
  luego añadir pared+vano de 0.8 m al room.world para replicar la puerta del lab.
- **GEMELO DEL LAB (tarde 07-03)**: `lab.world` GENERADO DEL MAPA REAL (paredes summit 2.2 m,
  522 cajas, recortado a la zona de juego, puerta carvada r=0.5 en (-3.90,1.25), spawn en A
  con yaw real, `-timeout 120`). v1 SIN muebles (los marrones de nav_map se quitaron; volverán
  como cajas limpias con `G1_SIM_FURN=1` para condiciones). Escenario `lab` es el DEFECTO del
  adaptador: waypoints/refmap/puerta REALES + engagement ON; nav_map de sim vacío en
  `sim/nav_map_lab.json`. GUI validada: un solo gzserver headless + `gzclient` lanzado DESDE
  la terminal del VNC (nunca 2 gazebos: mundo vacío + topics rotos); robot visible via
  View→Collisions; runs de datos siempre headless. Receta completa en sim/COMO_LANZAR.md.
  PENDIENTE: primera run sim A→B cruzando la PUERTA REAL (DOOR-ENG ... CROSSED) hasta reached.

### 8.17 P10 margen FEW-SHOT (Renxi, 07-03): confianza histórica en la plausibilidad

- Renxi: la analogía puede salir "muy plausible" con margen de calibración fino y aun así no
  valer en la realidad — la decisión era ONE-SHOT (incertidumbre DST solo instantánea). Fix en
  el bridge (su reasoner intacto): la incertidumbre de cada lectura suma la desviación típica
  HISTÓRICA de la métrica (`u = u_inst + k·σ_ventana`, tope 0.35, `G1_M2_HIST_K=0.4`, 0=one-shot).
  Una métrica que oscila sobre una frontera QoE ensancha su intervalo belief/plausibility →
  cae la belief_fulfillment (base del gate) y crece el uncertainty_gap → la plausibilidad exige
  consistencia entre varios shots. A/B offline (7 runs): limpias +9pp de FALLBACK (moderado),
  la run mala separada (72%), y con la escalada P9: 0 abortos falsos, la mala aborta a t=93 s.

### 8.4 Próximos pasos (en orden)

1. **Prueba goto B** en el Ubuntu (percepción ON; el gate ya lo exige). Mirar en el log, en este orden:
   `shz` (si <3 Hz el dedup era crítico), `stale_pct`, `flt` (pasillo vs puerta), `dmap` con robot parado
   (→ +0/-0), `gated_pct` (si >30% afinar `RX_GATE`), `nz`/`rel` al girar, y cerca de B que RODEE.
2. **Análisis post-run**: comparar vs runs del 07-01 con `summarize_runs.py` (columnas nuevas).
3. **Integrar floorcolor** en perception_server tras `G1_FLOORCOLOR=1` y repetir la run → comparativa.
4. Investigar la serie `1512xx` (¿qué golpeaba en x+1.20,y+0.19 invisible a la cámara?).

_Fin del traspaso. Para continuar: valida en robot los cambios de la sección 8.1 y sigue con 8.4._
