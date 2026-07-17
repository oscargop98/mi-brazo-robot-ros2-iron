# Trimestre 26-P — Reporte de Servicio Social

**Programa:** Apoyo para implementar la Arquitectura Cognitiva Inspirada en Neuronas Espejo
**Alumno:** Oscar David González Pintor — Matrícula: 2173071702
**Licenciatura:** Ingeniería en Computación — UAM Cuajimalpa
**Responsable del proyecto:** Dra. Alicia Montserrat Alvarado González (Núm. económico: 41051)
**Periodo reportado:** 1 de abril de 2026 — 30 de junio de 2026 (Meses 1, 2 y 3)

---

# Introducción

El presente reporte trimestral documenta las actividades realizadas durante los primeros tres meses del servicio social, inscrito bajo el programa **Apoyo para implementar la Arquitectura Cognitiva Inspirada en Neuronas Espejo**. Este proyecto busca dotar a un brazo robótico simulado de una arquitectura de cómputo inspirada en los principios neurocognitivos de las neuronas espejo: regiones corticales especializadas que se activan tanto al ejecutar una acción motora como al observar a otro agente ejecutarla, estableciendo la base neurológica de la imitación, el aprendizaje y la predicción motora.

El sustrato físico de la arquitectura es el brazo robótico antropomórfico **Xolobot**, un manipulador de 21 grados de libertad con una mano de cinco dedos. El trabajo de este trimestre se centró en migrar y modernizar el entorno de simulación del Xolobot — necesario para que las cortezas cognitivas pudieran ejecutarse de forma reproducible — y en implementar los módulos computacionales que corresponden a la **Corteza Premotora**, la **Corteza Motora Primaria**, las **Cortezas Somatosensorial y Parietal**, y los primeros pasos hacia la integración del brazo y el robot simulados en un mismo espacio computacional.

## Antecedentes

El proyecto heredó una versión funcional del brazo Xolobot bajo ROS 2 Iron Irwini con Gazebo Clásico en Ubuntu 22.04. Si bien el robot podía levantarse y ejecutar trayectorias básicas, el entorno de simulación no era reproducible: dependía de una configuración manual del sistema operativo y no contaba con ningún mecanismo de containerización. Esto impedía distribuir el entorno entre los integrantes del laboratorio y hacía inviable la ejecución de los módulos cognitivos en máquinas con distintas configuraciones de hardware.

Adicionalmente, existía un referente de la arquitectura cognitiva en el proyecto **Robotic-Swarms**, que implementaba el módulo de **Corteza Premotora** sobre ROS 2 Foxy y servía como caso de estudio para comprender la interfaz de los nodos cognitivos con Gazebo.

---

# Objetivo

Apoyar la implementación de la Arquitectura Cognitiva Inspirada en Neuronas Espejo para el robot Xolobot, mediante:

1. La migración del entorno de simulación de ROS 2 Iron con Gazebo Clásico a **ROS 2 Jazzy Jalisco con Gazebo Harmonic**, garantizando su reproducibilidad mediante una imagen Docker pre-compilada de cinco capas.
2. El desarrollo del **nodo de Corteza Premotora** (`SimulationController`) en C++, capaz de generar trayectorias articulares reactivas y enviar su resultado a la memoria procedural del robot simulado.
3. La adaptación de las **Cortezas Somatosensorial y Parietal** al robot simulado, materializada en la configuración de siete sensores de contacto (bumpers) en Gazebo y su integración a través del bridge de comunicación.
4. La configuración de la **Corteza Motora Primaria** mediante el `JointTrajectoryController` de `ros2_control`, que traduce las órdenes del nodo cognitivo en movimiento articular ejecutado por el simulador.
5. La integración del brazo y el robot simulados en el **mismo espacio computacional**, conectando los contenedores `sim` (Gazebo) y `brain` (nodo cognitivo) a través de una red compartida y memoria IPC común.

---

# Planeación

## Calendario Oficial del Proyecto

La siguiente tabla reproduce las actividades oficiales del programa de servicio social tal como figuran en la carta de aceptación emitida el 31 de marzo de 2026. El reporte actual cubre los **Meses 1, 2 y 3**.

| Actividad | Mes 1 | Mes 2 | Mes 3 | Mes 4 | Mes 5 | Mes 6 |
|---|:---:|:---:|:---:|:---:|:---:|:---:|
| Apoyo en la implementación del nodo Corteza Premotora y enviar a la memoria procedural: robot simulado | ✦ | ✦ | | | | |
| Apoyo para levantar el brazo en ROS1 | | | | | | |
| Apoyo para migrar el robot de ROS1 a ROS2 | ✦ | ✦ | ✦ | ✦ | ✦ | ✦ |
| Apoyo para adaptar lo que ya se tiene de las cortezas Parietal, Somatosensorial, motora primaria al robot simulado | | ✦ | | | | |
| Apoyo para poner en el mismo espacio el brazo y el robot simulados | | | ✦ | ✦ | | |
| Apoyo en la documentación del proyecto | ✦ | ✦ | ✦ | ✦ | ✦ | ✦ |
| Informe trimestral | | | ✦ | | | ✦ |
| Informe final | | | | | | ✦ |

**Correspondencia semana–mes para este trimestre:**

| Mes | Semanas | Actividades ejecutadas |
|---|---|---|
| 1 (Abril) | 1–4 | Entorno Docker, xhost, estudio de Robotic-Swarms (referencia de Corteza Premotora) |
| 2 (Mayo) | 5–8 | Arquitectura Xolobot, Corteza Motora Primaria (JTC), Cortezas Somatosensorial y Parietal (sensores) |
| 3 (Junio) | 9–12 | Imagen Docker 5 capas, Corteza Premotora (SimulationController), integración brain+sim, VM |

## Justificación de la Estrategia de Containerización con Docker

Antes de iniciar el desarrollo técnico, se evaluaron las opciones de entorno disponibles. La conclusión fue que Docker era la única solución viable para cumplir los requisitos de reproducibilidad que la arquitectura cognitiva exige: los módulos deben poder ejecutarse en cualquier equipo del laboratorio sin intervención manual.

### Aislamiento

Los módulos cognitivos (Corteza Premotora, Motora, Somatosensorial) se desarrollan como nodos ROS 2 independientes. Docker permite ejecutar diferentes versiones de ROS 2 en contenedores distintos sin interferencias entre sus dependencias, lo que facilita el desarrollo paralelo de módulos.

### Limpieza

El historial de instalaciones del laboratorio incluía residuos de paquetes `.deb` de software incompatible que contaminaban el entorno de ROS 2. Con Docker, cualquier estado corrupto se resuelve eliminando el contenedor y recreándolo desde la imagen, sin afectar el sistema operativo del host.

### Compatibilidad

ROS 2 Jazzy Jalisco requiere Ubuntu 24.04 como base. Docker provee el sistema de archivos de Ubuntu 24.04 dentro del contenedor con independencia del sistema operativo del host, permitiendo que equipos con Ubuntu 22.04 ejecuten la arquitectura cognitiva sin reinstalación del sistema.

### Portabilidad para la Arquitectura Cognitiva

La imagen Docker pre-compilada garantiza que cualquier nodo cognitivo futuro (Corteza Prefrontal, Hipocampo, etc.) pueda incorporarse al sistema ejecutando un único comando de construcción, sin que diferencias de plataforma afecten la ejecución.

---

# Inicio

Al comenzar el trimestre se verificó que el entorno preexistente de ROS 2 Iron era funcional en el equipo original pero no portable. Se estableció como primer hito la obtención de un entorno Docker completamente operativo antes de proceder con la implementación de los módulos cognitivos.

---

# Desarrollo

---

## Semana 1

### Proceso: Preparación del entorno gráfico y diagnóstico del sistema

La arquitectura cognitiva requiere un simulador 3D para ejecutar el robot en su entorno. Gazebo Harmonic, el simulador objetivo, utiliza el protocolo X11 para renderizar su interfaz gráfica. El primer obstáculo técnico fue configurar el forwarding de esta interfaz hacia el contenedor Docker.

#### Paso A: Habilitar el acceso gráfico (crítico para Gazebo)

Sin esta configuración, Gazebo termina con el siguiente error al intentar inicializar su renderer:

```
qt.qpa.xcb: could not connect to display :0
qt.qpa.plugin: Could not load the Qt platform plugin "xcb"
[ERROR] [gz-5]: process has died [pid 8423, exit code 1]
```

La solución es autorizar al demonio Docker para usar el socket X11 del host antes de lanzar cualquier contenedor:

```bash
xhost +local:docker
```

Este comando modifica la lista de control de acceso del servidor X11, permitiendo conexiones desde procesos locales no autenticados. Debe ejecutarse una vez por sesión de trabajo.

#### Paso B: Verificación de Docker Engine

```bash
docker --version
# Docker version 29.6.1, build 8900f1d

docker compose version
# Docker Compose version v5.3.0
```

Se confirmó que el usuario no pertenecía al grupo `docker`, lo que requería `sudo` en todos los comandos. Se corrigió con:

```bash
sudo usermod -aG docker $USER
newgrp docker
```

---

## Semana 2

### Investigación: Estructura de las imágenes ROS 2 en Docker

Se estudió la organización de las imágenes oficiales de OSRF para comprender cómo construir el entorno cognitivo de forma estratificada. La jerarquía base es:

```
osrf/ros:jazzy-desktop
  └── Ubuntu 24.04 (Noble Numbat)
      └── ROS 2 Jazzy Jalisco (tools + desktop)
          └── Gazebo Harmonic + ros2-control + bridge
              └── Workspace compilado (nodos cognitivos Xolobot)
```

Se estableció el principio de **workspace pre-compilado**: la imagen compila todos los nodos cognitivos durante el `docker build`, haciendo que el arranque del sistema sea instantáneo (`docker compose up`). Esto es crítico para las sesiones de laboratorio, donde el tiempo de setup debe ser mínimo.

---

## Semana 3

### Configuración del repositorio y control de versiones

Se organizó el repositorio Git con una estrategia de ramas por área funcional:

| Rama | Módulo cognitivo correspondiente |
|---|---|
| `main` | Código estable de producción |
| `fix/cinematica-estable` | Correcciones de Corteza Motora Primaria |
| `feat/agarre-autonomo-waypoint` | Corteza Premotora — algoritmo de agarre reactivo |
| `infra/docker-gazebo-harmonic` | Infraestructura de integración (mismo espacio brain+sim) |

Se implementó el script `setup_aliases.sh` para automatizar la configuración del entorno en nuevas máquinas. El script es idempotente: verifica la existencia del bloque antes de escribir, evitando duplicados en `~/.bashrc`:

```bash
#!/bin/bash
MARKER="ALIAS PARA MIGRACION XOLOBOT (ROS 2 JAZZY)"

if grep -qF "$MARKER" ~/.bashrc; then
    echo "[xolobot] Aliases ya existen. Sin cambios."
    exit 0
fi

cat >> ~/.bashrc << 'EOF'
# ALIAS PARA MIGRACION XOLOBOT (ROS 2 JAZZY)
alias xolo_sim="xolo_kill && source /opt/ros/jazzy/setup.bash && \
    cd ~/migration_ws && colcon build --packages-select xolobot_arm_server && \
    source install/setup.bash && \
    ros2 launch xolobot_arm xolobot_arm_control.launch.py"

alias xolo_brain="source /opt/ros/jazzy/setup.bash && \
    cd ~/migration_ws && source install/setup.bash && \
    ros2 run xolobot_arm_server xolobot_arm_server \
    --ros-args -p use_sim_time:=true"

alias xolo_kill="pkill -9 -f ros2; pkill -9 -f gz; \
    pkill -9 -f xolobot_arm_server"
EOF

echo "[xolobot] Aliases inyectados. Ejecuta: source ~/.bashrc"
```

---

## Semana 4

### Antecedente: Proyecto Robotic-Swarms — Referencia de la Corteza Premotora

Como primera actividad formal dentro del Mes 1 del calendario oficial (*"Apoyo en la implementación del nodo Corteza Premotora"*), se estudió y ejecutó el proyecto **Robotic-Swarms**, que implementa una arquitectura cognitiva completa sobre ROS 2 para robótica de enjambre. Su paquete `cognitive_architecture` contiene nodos de planificación de acción que replican el comportamiento de la Corteza Premotora en un contexto de múltiples robots, y constituyó el **referente directo** del módulo que se implementaría para el Xolobot en la Semana 10.

El proceso documentado para ejecutar Robotic-Swarms bajo ROS 2 Foxy es el siguiente:

```bash
# Obtener la imagen base de ROS 2 Foxy
docker pull osrf/ros:foxy-desktop

# Lanzar el contenedor con soporte gráfico X11
docker run -it --rm \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd):/root/ros2_ws/src/Robotic-Swarms-main \
    osrf/ros:foxy-desktop
```

#### Dentro del contenedor — Compilación y ejecución del módulo cognitivo

```bash
# Activar el entorno de ROS 2 Foxy
source /opt/ros/foxy/setup.bash
apt-get update

# Instalar dependencias declaradas en package.xml
cd /root/ros2_ws/
rosdep install --from-paths src --ignore-src -r -y

# Compilar con colcon
colcon build

# Activar el workspace compilado
source install/setup.bash

# Lanzar la arquitectura cognitiva completa en Gazebo
ros2 launch cognitive_architecture gazebo.launch.py
```

**Observación sobre la Corteza Premotora en este contexto:** El paquete `cognitive_architecture` implementa nodos de planificación que procesan el estado del entorno y generan comandos motores. Esta estructura — un nodo que recibe señales sensoriales y produce trayectorias articulares — es exactamente la que se replicaría en `SimulationController.cpp` para el Xolobot. El estudio de este proyecto permitió establecer la interfaz conceptual entre el módulo cognitivo y Gazebo.

**Observación de red:** El flag `--net=host` comparte el stack de red del host con el contenedor, permitiendo el descubrimiento de nodos DDS de ROS 2 sin configuración adicional. Esta práctica se adoptó directamente para el entorno Xolobot.

---

## Semana 5

### Análisis comparativo: ROS Noetic vs ROS 2 Foxy/Jazzy en el contexto de la arquitectura cognitiva

El proyecto Robotic-Swarms también documentaba su ejecución bajo ROS Noetic (ROS 1), lo que permitió establecer un comparativo formal entre las dos generaciones del framework:

```bash
# Versión ROS 1 (Noetic) — para referencia comparativa
docker pull osrf/ros:noetic-desktop-full
docker run -it --rm --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd):/root/catkin_ws/src/Robotic-Swarms-main \
    osrf/ros:noetic-desktop-full
```

| Aspecto | ROS 1 Noetic | ROS 2 Foxy/Jazzy |
|---|---|---|
| Compilador | `catkin_make` | `colcon build` |
| Entorno compilado | `devel/setup.bash` | `install/setup.bash` |
| Comando de lanzamiento | `roslaunch pkg file.launch` | `ros2 launch pkg file.launch.py` |
| Comunicación internodos | XMLRPC + rosmaster | DDS (Data Distribution Service) |
| Ciclo de vida de nodos | Ninguno (solo start/stop) | Lifecycle nodes (unconfigured → active) |
| Soporte a largo plazo | EOL Mayo 2025 | Activo — Jazzy LTS hasta 2029 |
| Idoneidad para arq. cognitiva | Limitada | Óptima — lifecycle permite inicialización ordenada de cortezas |

Esta comparativa justificó técnicamente la decisión de desarrollar la arquitectura cognitiva del Xolobot exclusivamente sobre ROS 2 Jazzy.

---

## Semana 6

### Estudio del proyecto base Xolobot en ROS 2 Iron

Se procedió a estudiar la versión original del Xolobot bajo ROS 2 Iron y Gazebo Clásico, documentada en el Manual de Instalación del proyecto. El proceso de instalación en Ubuntu 22.04 es el siguiente:

```bash
# Prerrequisitos del sistema
sudo apt update && sudo apt install \
    ros-iron-desktop ros-dev-tools \
    ros-iron-gazebo-ros-pkgs \
    ros-iron-ros2-control \
    ros-iron-ros2-controllers -y

# Clonar el workspace
cd ~
git clone https://github.com/monmejss/ros2_ws.git
cd ros2_ws

# Activar entorno de ROS 2 Iron
source /opt/ros/iron/setup.bash

# Instalar dependencias del proyecto
rosdep install -i --from-path src --rosdistro iron -y

# El proyecto contiene paquetes duplicados en src_lata/
touch src_lata/COLCON_IGNORE

# Compilar
colcon build

# Activar workspace compilado
source install/setup.bash

# Lanzar la simulación
ros2 launch xolobot_arm xolobot_arm_control.launch.py
```

#### Errores catalogados durante la configuración inicial

| Error | Causa | Solución |
|---|---|---|
| `Cannot locate rosdep definition` | Terminal sin `source` de ROS 2 | Activar entorno antes de `rosdep` |
| `Duplicate package names` | Paquetes en `src/` y `src_lata/` simultáneamente | `touch src_lata/COLCON_IGNORE` |
| `Package 'xolobot_arm' not found` | `install/setup.bash` no activado | `source install/setup.bash` post-build |
| `robot_description MUST not be empty` | Launch file incorrecto | Usar `xolobot_arm_control.launch.py` |

Este relevamiento estableció la línea base a partir de la cual se ejecutaría la migración a Jazzy/Harmonic.

---

## Semana 7

### Corteza Motora Primaria — Configuración del JointTrajectoryController

Correspondiente a la actividad del Mes 2: *"Apoyo para adaptar lo que ya se tiene de las cortezas Parietal, Somatosensorial, motora primaria al robot simulado"*.

En la arquitectura cognitiva inspirada en neuronas espejo, la **Corteza Motora Primaria** traduce los planes de acción generados por la Corteza Premotora en señales motoras dirigidas a los efectores. En el sistema Xolobot, este rol lo cumple el **`JointTrajectoryController` (JTC)** de `ros2_control`: recibe trayectorias articulares del nodo cognitivo y las ejecuta sobre las 21 articulaciones del brazo en Gazebo Harmonic.

**Estructura articular del Xolobot (21 DOF):**

| Índice | Joint | Segmento |
|---|---|---|
| 0 | `jnt_pecho_hombro` | Rotación de base |
| 1 | `jnt_hombro_hombro` | Abducción lateral |
| 2 | `jnt_hombro_biceps` | Flexión del hombro |
| 3 | `jnt_biceps_codo` | Flexión del codo |
| 4 | `jnt_codo_antebrazo` | Pronación/supinación |
| 5 | `jnt_antebrazo_palma` | Flexión de muñeca |
| 6–8 | Pulgar (3 falanges) | Mano |
| 9–11 | Índice (3 falanges) | Mano |
| 12–14 | Cordial (3 falanges) | Mano |
| 15–17 | Anular (3 falanges) | Mano |
| 18–20 | Meñique (3 falanges) | Mano |

**Configuración de tolerancias del JTC** (`xolobot_control/config/xolobot_control.yaml`):

Para que la Corteza Motora acepte planes generados por la Corteza Premotora con tiempos de llegada inherentes a la ejecución reactiva, se configuraron tolerancias relajadas:

```yaml
joint_trajectory_controller:
  ros__parameters:
    allow_partial_joints_goal: true
    constraints:
      goal_time: 5.0
      stopped_velocity_tolerance: 0.5
      jnt_pecho_hombro:    {trajectory: 0.5, goal: 0.5}
      jnt_hombro_hombro:   {trajectory: 0.5, goal: 0.5}
      jnt_hombro_biceps:   {trajectory: 0.5, goal: 0.5}
      jnt_biceps_codo:     {trajectory: 0.5, goal: 0.5}
      jnt_codo_antebrazo:  {trajectory: 0.5, goal: 0.5}
      jnt_antebrazo_palma: {trajectory: 0.5, goal: 0.5}
      # ... (15 articulaciones de dedos con los mismos valores)
```

**Problema crítico resuelto — paquetes de ros2-control faltantes:**

Se identificó que `ros-jazzy-ros2-control` y `ros-jazzy-ros2-controllers` no estaban instalados. Sin estos paquetes, el `controller_manager` no disponía de los plugins del JTC y la Corteza Motora no existía como subsistema activo:

```bash
# Diagnóstico — JTC no aparece
ros2 control list_controllers
# (sin salida — subsistema de control no operativo)

# Solución
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
ros2 daemon stop && ros2 daemon start

# Confirmación post-instalación
ros2 control list_controllers
# joint_trajectory_controller  active
```

---

## Semana 8

### Cortezas Somatosensorial y Parietal — Sensores de contacto y bridge de comunicación

Correspondiente a la actividad del Mes 2: *"Apoyo para adaptar lo que ya se tiene de las cortezas Parietal, Somatosensorial, motora primaria al robot simulado"*.

En la arquitectura cognitiva, la **Corteza Somatosensorial** procesa la información táctil del cuerpo, mientras que la **Corteza Parietal** integra esa información para construir un modelo espacial de la interacción con el entorno. Para el Xolobot simulado, estos módulos se implementan como **siete sensores de contacto (bumpers)** distribuidos en la mano y el antebrazo, cuyas señales fluyen hacia el nodo de Corteza Premotora.

#### Migración de Gazebo Clásico a Gazebo Harmonic — Cambio en rutas de tópicos

En Gazebo Harmonic, los sensores publican en rutas completas del formato:

```
/world/{world_name}/model/{model}/link/{link}/sensor/{sensor}/contact
```

El bridge `ros_gz_bridge` requiere un archivo YAML explícito para mapear estas rutas largas a nombres cortos de ROS 2 que el nodo cognitivo puede consumir directamente:

```yaml
# xolobot_arm/config/bridge.yaml

# Reloj de simulación — crítico para sincronización de las cortezas
- ros_topic_name: /clock
  gz_topic_name: /clock
  ros_type_name: rosgraph_msgs/msg/Clock
  gz_type_name: gz.msgs.Clock
  direction: GZ_TO_ROS

# Corteza Somatosensorial: palma (receptor táctil principal)
- ros_topic_name: /bumper_states_palma
  gz_topic_name: /world/default/model/xolobot_arm/link/link_palma_izq/sensor/palma_sensor/contact
  ros_type_name: ros_gz_interfaces/msg/Contacts
  gz_type_name: gz.msgs.Contacts
  direction: GZ_TO_ROS

# Corteza Somatosensorial: yemas de los 5 dedos
- ros_topic_name: /bumper_states_pulgar_3
  gz_topic_name: /world/default/model/xolobot_arm/link/link_pulgar_3/sensor/pulgar_sensor/contact
  ros_type_name: ros_gz_interfaces/msg/Contacts
  gz_type_name: gz.msgs.Contacts
  direction: GZ_TO_ROS

- ros_topic_name: /bumper_states_indice_3
  gz_topic_name: /world/default/model/xolobot_arm/link/link_indice_3/sensor/indice_sensor/contact
  ros_type_name: ros_gz_interfaces/msg/Contacts
  gz_type_name: gz.msgs.Contacts
  direction: GZ_TO_ROS

# ... (cordial, anular, meñique con la misma estructura)
```

#### Migración de plugins SDF — diferencia crítica entre versiones de Gazebo

```xml
<!-- Gazebo Clásico (Iron) -->
<plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so"/>

<!-- Gazebo Harmonic (Jazzy) — plugins nativos gz-sim -->
<plugin filename="gz-sim-physics-system"
        name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-sensors-system"
        name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
```

El bridge `ros_gz_bridge` actúa como la **capa de traducción** entre el sistema nervioso periférico simulado (Gazebo) y el sistema nervioso central cognitivo (nodos ROS 2). Sin este mapeo, la Corteza Somatosensorial no recibiría ninguna señal táctil del entorno.

---

## Semana 9

### Integración en el mismo espacio — Imagen Docker de 5 capas

Correspondiente al inicio de la actividad del Mes 3: *"Apoyo para poner en el mismo espacio el brazo y el robot simulados"*.

Poner el brazo y el robot simulados "en el mismo espacio" implica, en términos computacionales, que los nodos cognitivos (`brain`) y el simulador físico (`sim`) puedan comunicarse como si estuvieran en la misma máquina, con latencias mínimas y descubrimiento automático de tópicos. La imagen Docker de 5 capas es el mecanismo que hace posible esta coubicación en cualquier equipo del laboratorio.

```dockerfile
# CAPA 1 — Base oficial ROS 2 Jazzy
FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

# CAPA 2 — Repositorio Gazebo Harmonic + dependencias
RUN apt-get update && apt-get install -y curl && \
    curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
         -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [...] http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
         > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y \
        gz-harmonic \
        ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-interfaces \
        ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
        ros-jazzy-gz-ros2-control ros-jazzy-controller-manager \
        ros-jazzy-robot-state-publisher \
        python3-colcon-common-extensions python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# CAPA 3 — rosdep
RUN rosdep init 2>/dev/null || true && rosdep update

# CAPA 4 — Workspace pre-compilado (nodos cognitivos)
WORKDIR /ros2_ws
COPY src/ src/
RUN . /opt/ros/jazzy/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build/ log/

# CAPA 5 — Entorno de ejecución
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash"  >> /root/.bashrc
COPY docker-entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
```

**`docker-compose.yml`** — definición del espacio compartido entre brazo y robot simulados:

```yaml
services:
  sim:   # El cuerpo: Gazebo + JTC (Corteza Motora Primaria)
    image: xolobot:jazzy
    network_mode: host        # mismo espacio de red que el host
    environment:
      - DISPLAY=${DISPLAY}
      - ROS_DOMAIN_ID=0
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:ro
    devices:
      - /dev/dri/renderD128   # GPU para Gazebo Harmonic
    command: ros2 launch xolobot_arm xolobot_arm_control.launch.py

  brain: # La mente: SimulationController (Corteza Premotora)
    image: xolobot:jazzy
    network_mode: host        # mismo espacio de red — DDS descubre sim
    depends_on:
      - sim
    command: >
      ros2 run xolobot_arm_server xolobot_arm_server
      --ros-args -p use_sim_time:=true
```

**Principio de invalidación de cache:** Solo la Capa 4 se reconstruye cuando cambia el código de los nodos cognitivos. Las Capas 1–3 permanecen en cache, reduciendo el tiempo de rebuild de ~15 minutos a ~2 minutos.

---

## Semana 10

### Corteza Premotora — Implementación del nodo SimulationController

Correspondiente a la actividad de los Meses 1 y 2: *"Apoyo en la implementación del nodo Corteza Premotora y enviar a la memoria procedural: robot simulado"*.

La **Corteza Premotora** es la región cortical responsable de la planificación y selección de acciones motoras complejas antes de que sean ejecutadas. En sistemas con neuronas espejo, esta región se activa tanto al planificar una acción propia como al observar a otro agente ejecutarla — siendo la base computacional de la imitación y el aprendizaje por observación. Para el Xolobot, la Corteza Premotora se implementa como el nodo C++ `SimulationController`: recibe señales de la Corteza Somatosensorial (tópicos de bumpers), planifica una secuencia de acciones (aproximación, agarre, elevación) y envía el plan resultante a la Corteza Motora Primaria (JTC) para su ejecución. El estado final del brazo tras el agarre constituye la salida dirigida a la **memoria procedural** del robot simulado.

#### Arquitectura de la máquina de estados reactiva

```
[INICIO] → warm-up (6 ticks × 2.5s = 15s para estabilizar /clock)
    ↓
[PLANIFICACIÓN] → Corteza Premotora genera trayectoria de 2 puntos
    ↓
[EJECUCIÓN] → Corteza Motora Primaria interpola hacia la lata
    ↓
[CONTACTO] → Corteza Somatosensorial detecta colisión palma/dedo
    ↓
[AGARRE] → Cierre de dedos + activación de articulación magnética
    ↓
[MEMORIA PROCEDURAL] → Corteza Premotora envía trayectoria de elevación
```

#### Valores de la pose de agarre (calibrados con Teach Pendant en RQT)

| Joint | Valor (rad) | Descripción biomecánica |
|---|---|---|
| `jnt_pecho_hombro` | 0.159 | Rotación del hombro hacia el objeto |
| `jnt_hombro_hombro` | 0.0 | Alineación lateral |
| `jnt_hombro_biceps` | -1.2874 | Flexión del hombro (valor calibrado) |
| `jnt_biceps_codo` | 1.5010 | Extensión del codo |
| `jnt_codo_antebrazo` | -1.068 | Pronación del antebrazo |
| `jnt_antebrazo_palma` | -1.1309 | Flexión de muñeca (-64.8°, perpendicular al objeto) |

#### Implementación del nodo de Corteza Premotora (fragmento principal)

```cpp
void SimulationController::generaAleatorios(){
    // Warm-up: esperar 6 ciclos del timer para que /clock fluya
    static int contador_inicio = 0;
    static bool pose_acercamiento_enviada = false;
    if (contador_inicio < 6) { contador_inicio++; return; }

    if (levantando) return;
    if (!colisionDetectada && pose_acercamiento_enviada) return;

    if (!colisionDetectada) {
        pose_acercamiento_enviada = true;
        RCLCPP_INFO(this->get_logger(),
            "Corteza Premotora: enviando plan motor a la lata...");
    }

    trajectory_msgs::msg::JointTrajectory jointTrajectoryMsg;
    // Sello temporal +0.5s sobre reloj de simulación (evita rechazo del JTC)
    jointTrajectoryMsg.header.stamp =
        this->now() + rclcpp::Duration::from_seconds(0.5);
    jointTrajectoryMsg.joint_names = { /* 21 joints */ };

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {0.159, 0.0, -1.2874, 1.5010, -1.068, -1.1309,
        /* dedos abiertos o cerrados según estado */};
    point.velocities.assign(TOTAL_JOINTS, 0.0);
    point.accelerations.assign(TOTAL_JOINTS, 0.0);

    if (!colisionDetectada) {
        // Punto 1: waypoint alto — volar sobre el pedestal
        trajectory_msgs::msg::JointTrajectoryPoint waypoint = point;
        waypoint.positions[2] = -0.5;
        waypoint.time_from_start.sec = 2;
        waypoint.time_from_start.nanosec = 500000000; // t = 2.5s
        jointTrajectoryMsg.points.push_back(waypoint);

        // Punto 2: pose final — descender sobre la lata
        point.positions[2] = -1.2874;
        point.time_from_start.sec = 5; // t = 5.0s
    } else {
        point.time_from_start.sec = 2;
    }

    jointTrajectoryMsg.points.push_back(point);
    jointTrajectoryPub->publish(jointTrajectoryMsg);
}
```

#### Detección de contacto — señal de la Corteza Somatosensorial al nodo Premotora

```cpp
// Filtro estricto: solo contacto en palma/dedos con el objeto objetivo
bool toca_robot = (
    col1.find("palma")   != std::string::npos ||
    col2.find("palma")   != std::string::npos ||
    col1.find("pulgar")  != std::string::npos ||
    col2.find("indice")  != std::string::npos ||
    col1.find("cordial") != std::string::npos ||
    col2.find("anular")  != std::string::npos ||
    col1.find("menique") != std::string::npos
);
bool toca_lata = (
    col1.find("objeto")   != std::string::npos ||
    col1.find("coke_can") != std::string::npos ||
    col2.find("objeto")   != std::string::npos ||
    col2.find("coke_can") != std::string::npos
);
if (toca_robot && toca_lata) {
    colisionDetectada = true;
    agarre_objeto();  // activa magnet_on — respuesta motora al tacto
}
```

---

## Semana 11

### Pruebas de integración en Máquina Virtual — Resolviendo la coubicación de brain y sim

Correspondiente a la actividad del Mes 3: *"Apoyo para poner en el mismo espacio el brazo y el robot simulados"*.

Al replicar el entorno en una máquina virtual para validar la portabilidad, se presentaron cuatro problemas de infraestructura. Los primeros tres afectan directamente la capacidad de los contenedores `brain` y `sim` de cohabitar en el mismo espacio computacional.

---

#### Problema 1: Colapso gráfico de Gazebo — Display no accesible

**Síntoma:**
```
qt.qpa.xcb: could not connect to display :0
qt.qpa.plugin: Could not load the Qt platform plugin "xcb"
[ERROR] [gz-5]: process has died [pid 8423, exit code 1]
```

**Causa raíz:** El `docker-compose.yml` original montaba el archivo de autoridad X11 con una ruta hardcodeada del host nativo (`/run/user/1000/gdm/Xauthority`). En la VM, GDM no estaba en ejecución y esta ruta no existía.

**Solución:** Cambiar el mapeo a la ruta estándar portable `~/.Xauthority`:

```yaml
# docker-compose.yml — corrección
volumes:
  - /tmp/.X11-unix:/tmp/.X11-unix:ro
  - ~/.Xauthority:/root/.Xauthority:ro

environment:
  - XAUTHORITY=/root/.Xauthority
  - DISPLAY=${DISPLAY}
```

**Resultado:** Gazebo renderiza su escena 3D desde el contenedor usando la sesión X11 activa de la VM.

---

#### Problema 2: Bloqueo de red DDS — El nodo `brain` no recibe `/clock`

**Síntoma:**
```
[WARN] [simulation_controller]: Usando wall time (177947...) — use_sim_time no sincronizado
[ERROR] [joint_trajectory_controller]: Trayectoria rechazada — timestamp en el pasado
```

**Causa raíz:** La red emulada de la VM bloqueaba el tráfico multicast utilizado por DDS. El contenedor `brain` (Corteza Premotora) no podía localizar el tópico `/clock` publicado por `sim`, por lo que su tiempo de simulación nunca se sincronizaba y las trayectorias llegaban al JTC con timestamps inválidos.

**Diagnóstico:**
```bash
ros2 topic echo /clock
# (sin salida — tópico no accesible desde el contenedor brain)
```

**Solución:** Forzar comunicación exclusivamente por loopback:

```yaml
environment:
  - ROS_LOCALHOST_ONLY=1
  - ROS_DOMAIN_ID=0
```

**Resultado:**
```bash
ros2 topic echo /clock
# header:
#   stamp:
#     sec: 12
#     nanosec: 450000000
# — Corteza Premotora sincronizada con tiempo de simulación ✓
```

---

#### Problema 3: Aislamiento de memoria IPC — Comandos del `brain` llegan pero no se ejecutan

**Síntoma:** El JTC en `sim` aparecía activo pero ignoraba silenciosamente las trayectorias publicadas por `brain`.

**Causa raíz:** Docker aísla el segmento de memoria compartida (IPC) entre contenedores por defecto. El middleware DDS utiliza `/dev/shm` para la comunicación intra-host de alta velocidad. Al estar en segmentos IPC distintos, los mensajes de la Corteza Premotora no llegaban a la Corteza Motora a través de este canal.

**Solución:** Compartir el espacio IPC del host entre ambos contenedores — esta directiva es la que efectivamente pone al `brain` y al `sim` en el **mismo espacio** de memoria:

```yaml
services:
  sim:
    ipc: host
    network_mode: host

  brain:
    ipc: host
    network_mode: host
```

**Resultado:** Los comandos de la Corteza Premotora comenzaron a ejecutarse en la Corteza Motora. Confirmación:

```bash
ros2 topic echo /joint_trajectory_controller/joint_trajectory --once
# joint_names: [jnt_pecho_hombro, jnt_hombro_hombro, ...]
# points: [{positions: [0.159, 0.0, -0.5, ...], time_from_start: {sec: 2}}]
```

---

## Semana 12

### Memoria Procedural — Adaptación del plan motor por retroalimentación del entorno

Correspondiente al cierre del Mes 3 e Informe Trimestral.

#### Problema 4: Colisión cinemática — El antebrazo choca con el pedestal durante la aproximación

**Síntoma:**
```
[INFO] [simulation_controller]: Choque detectado:
    xolobot_arm::link_palma_izq::collision <--> soporte::link::collision
```

**Causa raíz:** El `JointTrajectoryController` interpola todos los joints simultáneamente mediante una curva cúbica. Al enviar un único punto de destino, el brazo trazaba un arco descendente mientras aún giraba horizontalmente, y el antebrazo colisionaba con el pedestal antes de llegar al objeto.

```
Sin waypoint — interpolación en arco (incorrecto):

   [inicio] ─────────────────────── [pose_final]
                 ↓ arco cruza pedestal ↓
                       [PEDESTAL] ← colisión del antebrazo

Con waypoint — trayectoria controlada (correcto):

   [inicio] ─── [waypoint_alto] ─── [pose_final]
                 (sobre pedestal)       ↓ descenso vertical
                                    [LATA]
```

**Solución — Memoria Procedural:** La corrección refleja el principio de **aprendizaje por retroalimentación** que define a la memoria procedural: el plan motor inicial produjo un fallo (colisión), y la secuencia fue modificada para evitarlo en ejecuciones futuras mediante un waypoint intermedio. El sistema ahora almacena el plan de dos pasos como el programa motor estable del agarre.

```cpp
// Memoria procedural: plan motor corregido tras colisión con pedestal
if (!colisionDetectada) {
    // Paso 1 (t=2.5s): elevar el codo — volar sobre el espacio del pedestal
    trajectory_msgs::msg::JointTrajectoryPoint waypoint = point;
    waypoint.positions[2] = -0.5;
    waypoint.time_from_start.sec = 2;
    waypoint.time_from_start.nanosec = 500000000;

    // Paso 2 (t=5.0s): descender verticalmente — llegar a la lata desde arriba
    point.positions[2] = -1.2874;
    point.time_from_start.sec = 5;

    jointTrajectoryMsg.points.push_back(waypoint);
}
jointTrajectoryMsg.points.push_back(point);
```

#### Elevación post-agarre — envío del estado final a la Memoria Procedural

Una vez completado el agarre, la Corteza Premotora genera el plan de elevación que constituye la acción terminal de la secuencia almacenada:

```cpp
void SimulationController::moverHombro(){
    if(!colisionDetectada) return;
    temporizadorHombro->cancel();
    levantando = true;

    trajectory_msgs::msg::JointTrajectory liftMsg;
    liftMsg.header.stamp = this->now() + rclcpp::Duration::from_seconds(0.5);

    trajectory_msgs::msg::JointTrajectoryPoint point;
    // Posición de elevación: codo alto, dedos cerrados sosteniendo la lata
    point.positions = {0.159, 0.0, -0.5, 1.5010, -1.068, -1.1309,
        1.5708, 0.2094, 0.5236,   // pulgar
        0.80,   0.6109, 0.6981,   // índice
        1.1345, 0.6109, 0.6109,   // cordial
        1.1345, 0.6109, 0.6109,   // anular
        0.80,   0.6109, 0.6109};  // meñique
    point.velocities.assign(TOTAL_JOINTS, 0.0);
    point.accelerations.assign(TOTAL_JOINTS, 0.0);
    point.time_from_start.sec = 5; // elevación suave en 5 segundos
    liftMsg.points.push_back(point);
    jointTrajectoryPub->publish(liftMsg);
}
```

#### Log de ejecución completa del sistema integrado

```
[t=0s]   Contenedor brain iniciado — Corteza Premotora en warm-up (6 ticks)
[t=15s]  Corteza Premotora: enviando plan motor a la lata...
[t=15s]  Trayectoria publicada: 2 puntos (t=2.5s waypoint, t=5.0s pose final)
[t=17.5s] Corteza Motora Primaria: waypoint alcanzado — brazo sobre pedestal
[t=20s]  Corteza Motora Primaria: pose de agarre alcanzada
[t=20.3s] Corteza Somatosensorial: ¡Contacto confirmado!
              [link_palma_izq::collision <--> objeto::link::collision]
[t=20.3s] Corteza Premotora: cerrando dedos — secuencia de agarre activa
[t=20.3s] Articulación magnética activada (magnet_on)
[t=28.3s] Corteza Premotora → Memoria Procedural: elevando con la lata
```

---

# Observaciones y Conclusiones

## Observaciones técnicas

1. **Los contenedores Docker resuelven el requisito de "mismo espacio".** Las directivas `network_mode: host` e `ipc: host` son suficientes para que los nodos cognitivos (`brain`) y el simulador (`sim`) se comuniquen con latencias equivalentes a las de procesos locales. El `ROS_DOMAIN_ID` compartido garantiza que el descubrimiento DDS funcione sin configuración adicional, haciendo transparente la separación entre contenedores.

2. **La sincronización temporal es crítica para la Corteza Motora.** El JTC rechaza silenciosamente cualquier trayectoria cuyo `header.stamp` sea anterior al tiempo actual del controlador. La solución `this->now() + rclcpp::Duration::from_seconds(0.5)` es un patrón requerido en cualquier nodo de Corteza Premotora que interactúe con el JTC bajo tiempo de simulación.

3. **La calibración con el Teach Pendant es la única fuente de verdad para la cinemática.** Los valores calculados analíticamente divergen de los reales debido a offsets mecánicos del modelo SDF. El procedimiento con `rqt_joint_trajectory_controller` produjo los valores definitivos que se almacenaron en la memoria procedural del sistema.

4. **Los cuatro problemas de VM son sistémicos en entornos ROS 2 + Docker.** Son consecuencias predecibles de ejecutar DDS dentro de contenedores en redes virtualizadas. Las soluciones documentadas (`~/.Xauthority`, `ROS_LOCALHOST_ONLY=1`, `ipc: host`, waypoint cinemático) son aplicables a cualquier módulo de la arquitectura cognitiva en este contexto de despliegue.

## Conclusiones

Al término del Trimestre 26-P se cumplieron los objetivos establecidos para los **Meses 1, 2 y 3** del calendario oficial. El entorno de simulación del Xolobot fue migrado exitosamente a ROS 2 Jazzy/Gazebo Harmonic y encapsulado en una imagen Docker reproducible.

Los módulos computacionales de la **Arquitectura Cognitiva Inspirada en Neuronas Espejo** implementados en este trimestre son:

| Módulo cognitivo | Implementación técnica | Semana |
|---|---|---|
| Corteza Premotora | `SimulationController` C++ — planificación y trayectorias reactivas | 4 (referencia), 10 (implementación) |
| Corteza Motora Primaria | `JointTrajectoryController` — ejecución articular en Gazebo | 7 |
| Corteza Somatosensorial y Parietal | 7 bumpers + bridge YAML — retroalimentación táctil | 8 |
| Mismo espacio brain+sim | `network_mode: host` + `ipc: host` — coubicación computacional | 9, 11, 12 |
| Memoria Procedural | Ajuste del plan motor por retroalimentación — waypoint cinemático | 12 |

El sistema es capaz de ejecutar la secuencia completa de agarre autónomo — aproximación, contacto, cierre de dedos, elevación — en cualquier equipo del laboratorio mediante `docker compose up sim` seguido de `docker compose run brain`, cumpliendo el objetivo de reproducibilidad y portabilidad que la arquitectura cognitiva exige para su desarrollo colaborativo.
