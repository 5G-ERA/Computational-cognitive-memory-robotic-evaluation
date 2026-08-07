# Gemelo digital del G1 — lanzar, reconstruir y salvaguardar

Documento operativo, 2026-08-07. Complementa a `README.md` (qué es el contenedor) y a
`COMO_LANZAR.md` (chuleta rápida). Aquí está lo que faltaba: **cómo rehacer el gemelo desde
cero si el contenedor se pierde**, y cómo guardarlo para que eso no importe.

---

## 0. Las tres piezas (y dónde vive cada una)

| Pieza | Qué es | Dónde vive |
|---|---|---|
| **Contenedor** `g1sim:humble` | ROS 2 Humble + Gazebo + escritorio por noVNC | Docker del Mac — **ahora también reproducible** con `sim/Dockerfile` |
| **Paquete `g1_sim`** | Los mundos (`lab.world` = escaneo real de la casa), lanzadores, URDF | Versionado en `sim/g1_sim_pkg/` ✔ |
| **Adaptador** | Corre el stack real de navegación contra la simulación | `g1_sim_adapter.py` en la raíz del repo ✔ |

La clave del diseño: `lab.world` está en el **mismo marco de referencia** que el robot real,
así que los waypoints, el mapa de referencia y la puerta reales se usan **sin traducir nada**,
y `g1_goto.py` corre **sin modificar** en ambos mundos.

---

## 1. Lanzar el gemelo (día a día)

### 1.1 Arrancar el contenedor
```bash
docker ps | grep g1sim || docker start g1sim
```
Si el puerto 6080 está ocupado por otro contenedor de ROS que arranca solo:
```bash
docker stop ros2_humble
```

### 1.2 Simulación (headless — el modo de las travesías de datos)
```bash
docker exec -d g1sim bash -c "source /opt/ros/humble/setup.bash && source /home/ubuntu/g1_ws/install/setup.bash && ros2 launch g1_sim lab.launch.py gui:=false > /tmp/lab_launch.log 2>&1"
```
Comprobar que arrancó (debe aparecer `Successfully spawned entity [g1]`):
```bash
docker exec g1sim grep -c "Successfully spawned" /tmp/lab_launch.log
```

### 1.3 Puente websocket (SIEMPRE a mano tras reiniciar el contenedor)
Este es el paso que más veces se ha olvidado: **relanzar Gazebo no relanza el puente**.
```bash
docker exec -d g1sim bash -c "source /opt/ros/humble/setup.bash && source /home/ubuntu/g1_ws/install/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=8765"
```

### 1.4 Colocar el robot y lanzar una travesía
El adaptador **no reposiciona** el robot: hay que teleportarlo antes de cada travesía manual.
```bash
docker exec g1sim bash -c "gz model -m g1 -x 0.99 -y 0.57 -z 0.05 -Y -2.1"     # posición A
docker exec g1sim bash -c "gz model -m g1 -x -4.75 -y 2.60 -z 0.05 -Y -0.9"    # posición B
```
La travesía se lanza desde el Mac con el entorno del experimento (ver
`docs/G1_Test_Protocol_Operator_Runbook.pdf`, §1.3) y el adaptador:
`g1_sim_adapter.py goto A|B`.

### 1.5 Ver la simulación (escritorio noVNC)
| Qué | Cómo |
|---|---|
| Escritorio en el navegador | `http://localhost:6080/vnc_lite.html` (cliente ligero) o `/vnc.html` |
| Contraseña | `ubuntu` |
| VNC nativo | puerto 5900 (no es HTTP: Safari no lo abre) |
| Puente ROS | puerto 8765 |

Para **ver** el robot moverse hay que lanzar con `gui:=true`, pero las travesías de datos van
siempre headless: el render por CPU roba física y falsea los tiempos.

---

## 2. Reconstruir el gemelo desde cero

Necesario si se pierde el contenedor, o para montarlo en otra máquina.

```bash
cd "<raíz del repo G1>"
docker buildx build --platform linux/amd64 -f sim/Dockerfile -t g1sim:humble .
```

Arrancarlo con los puertos y permisos correctos (la base Jammy exige `seccomp=unconfined`):
```bash
docker run -d --name g1sim --security-opt seccomp=unconfined -p 6080:80 -p 5900:5900 -p 8765:8765 --shm-size=1g g1sim:humble
```

Qué hace el Dockerfile, en orden: parte de la imagen base con escritorio y noVNC, rehace las
fuentes apt de ROS y Gazebo, instala los paquetes (**incluido `rosbridge-suite`, que en el
contenedor original se había instalado a mano y no estaba en la imagen**), replica el entorno
de shell, clona el repositorio público de Unitree para generar `g1_description` con
`setup_g1.sh` + `regen_g1_nav.sh`, copia el paquete `g1_sim` y compila el workspace.

**Por qué `g1_description` no está en el repo**: son 110 MB de terceros (paquete oficial de
Unitree Robotics). Se regenera con el script incluido. Si algún día quieres una copia
hermética e independiente de que el repo de Unitree siga existiendo, la opción es meterlo con
Git LFS — decisión pendiente, hoy no está hecho.

### Verificación tras reconstruir (no te fíes: mídelo)
1. `ros2 pkg list | grep -E "g1_sim|g1_description|rosbridge"` → los tres presentes.
2. Lanzar headless (§1.2) → debe aparecer `Successfully spawned entity [g1]`.
3. Levantar el puente (§1.3) y teleportar a A (§1.4).
4. Una travesía A→B con la máquina meta: **debe llegar en 95–115 s con 0 colisiones**.
   Si sale muy fuera de esa banda, la física no es equivalente a la del contenedor original
   (arquitectura, versión de Gazebo o modelo de colisión) y **los tiempos no son comparables**
   con las campañas anteriores: hay que rehacer el baseline del gemelo antes de usarlo.

---

## 3. Salvaguardar la imagen actual

El contenedor original tiene 6 semanas de ajustes; el Dockerfile lo reproduce, pero **hasta
que no se haya construido y verificado una vez, la única copia fiel es la imagen del Mac**.

### 3.1 Copia local (rápida, sin credenciales, no va a git)
```bash
docker save g1sim:humble | gzip > ~/Documents/g1sim_humble_$(date +%Y%m%d).tar.gz
```
Restaurar: `gunzip -c <fichero>.tar.gz | docker load`. Ocupa ~2,6 GB comprimidos.

### 3.2 Publicar en el registro de GitHub (GHCR)
Esto **lo tienes que lanzar tú**: Docker en este Mac tiene sesión en registros de Azure del
trabajo, no en GHCR, y las credenciales las manejas tú, no el asistente.

```bash
echo $GITHUB_TOKEN | docker login ghcr.io -u <tu-usuario> --password-stdin
docker tag g1sim:humble ghcr.io/5g-era/g1sim:humble
docker push ghcr.io/5g-era/g1sim:humble
```
El token necesita permiso `write:packages`. Sube ~2,6 GB. Después, en cualquier máquina:
`docker pull ghcr.io/5g-era/g1sim:humble`.

**Antes de publicar, decide la visibilidad**: el paquete hereda la del repositorio de la
organización. La imagen contiene el escaneo de una vivienda particular (el mundo `lab.world`),
así que si el repositorio es público, esto también lo será.

---

## 4. Avisos

- **Arquitectura**: la imagen es `linux/amd64` y en Apple Silicon corre emulada. Es
  deliberado: la física está calibrada así. Construirla nativa en arm64 cambiaría los tiempos.
- **El puente es manual**: reiniciar el contenedor deja el puente caído aunque Gazebo vuelva.
- **Modelo de colisión**: el robot usa el envolvente real (0,44 m de hombros × 1,10 m). Con la
  caja antigua y permisiva el robot "cruzaba" la puerta con los hombros dentro de la pared —
  falso éxito documentado el 03-jul-2026. No volver a la caja simple para "mejorar" resultados.
- **El Dockerfile aún no se ha construido**: está derivado del historial real de la imagen y
  de la auditoría del contenedor, pero la primera build hay que validarla con §2.
