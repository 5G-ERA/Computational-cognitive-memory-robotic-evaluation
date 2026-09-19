# Banco 2D del Summit XL — gpuedge, `~/summit_sim2d`

17-sep-2026, noche. El robot de la oficina, su mapa, sus parámetros de Nav2 y su láser, sin robot. Sirve para
probar la puerta las veces que haga falta sin que nadie tenga que darle a la seta.

## Arrancar

```bash
SIM_MAPA=~/summit_sim2d/mapa_fino/mapa_25mm_vano.yaml bash ~/summit_sim2d/summit_sim.sh arranca --verdad
```

Deja listo: simulador, Nav2 (8 nodos activos), RViz en la pantalla de gpuedge y el latido del láser. Sin `--verdad`
añade AMCL y pone la pose inicial en A. Otros comandos: `estado`, `para`, `pose A`.

- **`--verdad`**: sin AMCL; el simulador publica `map→odom` y `/robot/amcl_pose` exactos. Es la pregunta geométrica
  pura: ¿cabe y planifica?
- **sin `--verdad`**: con AMCL, como el robot. Es la pregunta de verdad: ¿cabe *sabiendo dónde está*?
- **`SIM_MAPA=`**: qué mapa. `mapa_25mm_vano.yaml` es el bueno (2,5 cm y vano a la medida de la cinta).

Dominio DDS **40** — el robot vive en el 39 y no se tocan.

## Probar

```bash
python3 ~/summit_sim2d/experimento.py plan B          # ¿planifica NavFn hasta B?
python3 ~/summit_sim2d/experimento.py va B 300        # navega de verdad, con holgura y choque
python3 ~/summit_sim2d/experimento.py cruza           # la maniobra del robot: vano_antes → alinear → recto → B
python3 ~/summit_sim2d/experimento.py barrido footprint_padding 0.02 0.01 0.0
python3 ~/summit_sim2d/mira_costmap.py                # coste bajo el robot y corredor libre en el vano
```

El simulador publica `/sim/holgura` (m de la huella al obstáculo más cercano, **verdad del mapa**, no estimación) y
`/sim/choque`; cuando la huella toca pared, el robot se para y lo dice. Eso es lo que el gemelo de Isaac no tenía.

## Lo que ya está medido (17-sep, noche)

| mapa | padding | plan A→B | travesía |
|---|---|---|---|
| 5 cm (el del robot) | 0,02 (nominal) | **ABORTED** | — |
| 5 cm | 0,01 / 0,005 | ABORTED | — |
| 5 cm | 0,00 | SUCCEEDED | aborta en marcha |
| 2,5 cm | 0,02 | ABORTED | — |
| 2,5 cm | ≤ 0,013 | SUCCEEDED | choca en la jamba |
| 2,5 cm + vano a la cinta | **0,02 (nominal)** | **SUCCEEDED** | **3/3 con pose perfecta, holgura mín 2,5 cm** |
| 2,5 cm + vano a la cinta | 0,02, **con AMCL** | a veces | **0 de 3: dos abortan, una choca con el mobiliario** |

### Las cuatro cosas que esto dice

1. **`resolution: 0.02` en los costmaps no hace nada.** La capa estática impone la resolución del **mapa**: el
   costmap publicado va a 5 cm. Medido en `mira_costmap.py` (`res 0.050` con el parámetro a 0,02).
2. **El mapa de SLAM estrecha la puerta 10 cm.** Paso medido en el PNG: **0,640 m**; la cinta, 0,742. Con 0,640 y un
   robot de 0,613 quedan 1,35 cm por lado y **ningún ajuste de Nav2** hace que quepa con padding nominal. `abre_vano.py`
   devuelve el vano a la medida de la cinta (no inventa espacio: corrige el barrido).
3. **El `Spin` del behavior server no puede alinear.** Arranca a `min_rotational_vel` 0,4 rad/s y la base frena a
   0,5 rad/s²: cualquier giro se pasa ~9°, así que el bucle de media ganancia oscila entre ±5° para siempre. Con un
   mando propio de `cmd_vel` a 0,03–0,15 rad/s converge a 1,0° a la primera.
4. **El vano no es lo que estrecha el paso: lo que hay detrás, sí.** Con pose perfecta cruza 3/3 con la seguridad
   nominal y 50 mm de holgura por lado; con AMCL, 0 de 3 —dos abortan por el planificador y una **choca contra el
   mobiliario que hay 0,75 m detrás de la puerta**, no contra la jamba (ver el perfil de holgura, abajo)—. Para la
   interfaz de holgura de Renxi el dato es: la holgura útil no es `vano − robot`, es
   `vano − robot − 2·(error de localización)`, y aquí son 50 mm contra 28,6 mm de p95.

## El error de AMCL contra la verdad (medido el 17-sep por la noche)

`error_amcl.py` compara la pose que **usa Nav2** —la transformada `map→base_footprint`, a la marca de tiempo de la
verdad— con `/sim/pose_verdad`, y descompone el error en **lateral al eje del vano**, que es el que mete en la jamba.
`analiza_error.py` lo resume. Cinco pasadas por la puerta guiadas por la verdad, 3229 muestras dentro del vano:

| | lateral |
|---|---|
| mediana | **2,9 mm** |
| p95 | **28,6 mm** |
| máximo | **42,8 mm** |
| rumbo p95 | 0,8° |

**Margen por lado, medido con la huella real dentro del vano corregido: 50 mm.** Ninguna de las 3229 muestras se
pasa: **cabe**, pero el peor caso deja 7 mm. Y con el mapa **sin corregir** (paso 0,640 m → margen 13,5 mm), el p95
del error de AMCL ya se lo come él solo: el contacto está garantizado, que es lo que pasó hoy.

**Con una salvedad grande, y es del banco:** estas pasadas van guiadas por la verdad, así que AMCL sólo tenía que
*seguir*, no corregir; y el láser se traza sobre el mismo mapa contra el que AMCL se localiza, o sea el caso más
favorable que existe. En las travesías completas **AMCL sí llegó a divergir** —el log del costmap local llegó a
decir «sensor origin at (2,56, 18,18) is out of map bounds», 18 m fuera—, y no he separado todavía cuánto de eso es
una carrera entre el teletransporte del banco y la pose inicial y cuánto sería real.

### Perfil de holgura del paso (`experimento.py perfil`)

Teletransportando la huella real a lo largo del eje y leyendo la holgura contra la verdad del mapa:

- de −0,35 a +0,35 m del centro del vano: **50 mm constantes** — el paso;
- de +0,40 a +0,65 m: 75 mm — ya fuera;
- **a +0,75 m: 0 mm, y a partir de ahí choca.** Hay mobiliario justo detrás de la puerta.

Esto último es un fallo concreto de `summit_cruce.sh` en el robot: manda un recto de 1,75 m desde `vano_antes`, que
acaba en s = +0,96 — **20 cm dentro del mobiliario**. El tramo recto no debe pasar de ~1,35 m.

## 18-sep: por qué aborta el recto, y `cruza_vano.py`

- **El ruido del láser tumba a `DriveOnHeading`**: robot perfectamente alineado, 50 mm reales por lado, todo nominal →
  4/4 sin ruido, 1/4 con σ 1 cm, 0/4 con σ 2 cm, **0/6 con ruido realista** (σ 5 mm + 3 % de atípicos). El láser real
  medido en la bolsa: σ 4 mm, p95 22 mm, p99 112 mm. El chequeo de Nav2 es un máximo sobre celdas de 2 cm: basta un
  atípico. `--ruido`, `--atipicos` y `--filtro` en `summit_sim.sh arranca`; `experimento.py rectos N`.
- **El eje de −96,6° es oblicuo**: la normal de la puerta es ≈ −89°. Cruzar a 8° exige 0,71 m de los 0,742.
  `prueba_cruza.sh` entra ya por la normal (1,7345, −3,213, −89,8°).
- **`cruza_vano.py`** cruza midiendo él la holgura en el láser (sin costmap, sin AMCL, sin mapa) y deja un CSV por
  ciclo — el replay de holgura. **13 de 16** con ruido realista y arranques de hasta 5 cm / 6°. Pendiente: en los 3
  fallos su holgura medida no avisó del contacto. No va al robot sin la persona de la seta.
  `bash prueba_cruza.sh "0:0 3:3 -4:4"` → pares descentrado_cm:rumbo_grados.
- **Metaparámetro**: ρ = margen por lado / incertidumbre de percepción. Tabla en la ficha Nav2 del vault.

## Lo que este banco NO prueba

Es cinemático: no hay deslizamiento de las ruedas, ni moqueta, ni PhysX. El láser se traza sobre el mismo mapa que
usa Nav2, así que **no hay discrepancia mapa/mundo** ni obstáculos que no estén en el mapa (cajas, sillas, gente) —
y una de esas cajas es la que el robot se llevó por delante. Los tiempos son de reloj de pared, no de simulación.
Sirve para geometría, configuración y lógica de maniobra; no acredita nada sobre la dinámica ni sobre el mundo real.

## Ficheros

| | |
|---|---|
| `summit_sim.sh` | arranca / para / estado / pose · el único comando que hace falta |
| `sim2d.py` | el simulador: huella real, láser por trazado de rayos, choque y holgura contra la verdad |
| `lanza_nav2.sh` | Nav2 nodo a nodo bajo `/robot` (el launch de `nav2_bringup` **no** empuja el namespace en este Humble) |
| `genera_params_sim.py` | `nav2_sim.yaml` a partir de los `ros2 param dump` reales del robot |
| `experimento.py` | banco: `plan`, `va`, `recto`, `cruza`, `pasa`, `perfil`, `barrido`, `holgura` |
| `cruza_vano.py` · `prueba_cruza.sh` | cruce guiado por holgura medida en el láser, y su tanda de pruebas |
| `filtra_scan.py` | filtro de atípicos del scan que conserva bordes |
| `error_amcl.py` · `analiza_error.py` | error de AMCL contra la verdad, descompuesto lateral/longitudinal |
| `mira_costmap.py` | costmap vivo: coste bajo el robot, corredor libre en el vano, ¿hay camino? |
| `remuestrea_mapa.py` | mapa a celdas más finas sin cambiar la geometría |
| `abre_vano.py` | devuelve al vano su anchura de cinta métrica |
| `compara_mapas.py` | compara el mapa y su copia `_nav` y mide el paso más estrecho |
| `mapa_fino/` | `mapa_25mm`, `mapa_10mm`, `mapa_25mm_vano` (el bueno) |
| `log/` | un fichero por nodo y arranque |

## Siguiente

1. **Acortar el recto de `summit_cruce.sh`** a 1,35 m: hoy lleva 1,75 y eso son 20 cm dentro del mobiliario.
2. **AMCL**: medido aquí en las mejores condiciones posibles (p95 28,6 mm). Falta saber cuánto empeora con la
   discrepancia mapa/mundo y el deslizamiento reales, que el banco no tiene. Más rayos (60 → 180) y `update_min_d`
   más fino son las palancas; el banco ya da la forma de medirlo.
3. **Cruzar sin mapa**: alinear y cruzar contra el vano **como lo ve el láser**, no contra una pose del mapa. Es lo
   que haría una interfaz de holgura y no depende de AMCL.
4. **Llevar al robot** sólo lo que aquí pase: el mapa corregido y la alineación en lazo cerrado.
