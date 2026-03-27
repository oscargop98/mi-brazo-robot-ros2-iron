# Proyecto de Migración ROS 2 (Iron ➡️ Jazzy) y Gazebo (Classic ➡️ Harmonic)
### Bitácora 0001
**Autor:** Oscar David Gonzalez Pintor SS

## Índice
1. [Plan de Migración Original](#1-plan-de-migración-original)
2. [Resumen de Hallazgos y Problemas](#2-resumen-de-hallazgos-y-problemas)
3. [Comparativa de Código (Antes vs Después) y Justificación](#3-comparativa-de-código-antes-vs-después-y-justificación)

---

## 1. Plan de Migración Original

# Plan de Migración a ROS 2 Jazzy y Gazebo Harmonic

El proyecto actual (`mi-brazo-robot-ros2-iron`) utiliza **ROS 2 Iron** pero está fuertemente acoplado a **Gazebo Classic** (Gazebo 11), usando paquetes como `gazebo_ros`, `gazebo_dev` y `gazebo_msgs`. Para migrar a **ROS 2 Jazzy** en **Ubuntu 24.04**, es obligatorio migrar la simulación a **Gazebo Harmonic** (el nuevo estándar, anteriormente conocido como Ignition Gazebo), ya que Gazebo Classic ha llegado a su fin de vida (EOL) y no es compatible de forma nativa con Jazzy.

Esta migración conlleva cambios arquitectónicos importantes en la simulación.

## User Review Required
> [!WARNING]
> **Cambio radical en Gazebo:** El plugin personalizado `ros2_linkattacher` escrito en C++ para Gazebo Classic **no funcionará** en Gazebo Harmonic, ya que la arquitectura de Harmonic está basada en ECS (Entity Component System) y las APIs son completamente diferentes. 
> 
> **Propuesta para `ros2_linkattacher`:** En lugar de reescribir un plugin complejo en C++ desde cero para Harmonic, Gazebo Harmonic incluye un sistema nativo llamado `DetachableJoint` que permite unir y separar modelos dinámicamente usando tópicos de Gazebo. Proponemos adaptar `SimulationController.cpp` para que use el puente `ros_gz_bridge` y se comunique con el `DetachableJoint` nativo de Harmonic. ¿Estás de acuerdo con este enfoque?

> [!IMPORTANT]
> **Sensores de colisión (Bumpers):** Actualmente usas `gazebo_msgs::msg::ContactsState`. En Harmonic, usaremos el `Contact System` y puentearemos los mensajes de colisión a ROS 2 a través de `ros_gz_bridge`. El tipo de mensaje de ROS 2 cambiará a `ros_gz_interfaces/msg/Contacts`.

## Proposed Changes

---

### 1. Actualización de Dependencias (package.xml y CMakeLists.txt)
Sustitución de todas las referencias de Gazebo Classic por las de Gazebo Harmonic (`ros_gz`).

#### [MODIFY] `xolobot_arm/package.xml` y `xolobot_arm/CMakeLists.txt`
- Eliminar `gazebo_ros`.
- Añadir `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_interfaces`.

#### [MODIFY] `xolobot_arm_server/package.xml` y `xolobot_arm_server/CMakeLists.txt`
- Eliminar `gazebo_msgs`, `gazebo_ros`, `gazebo_plugins`, `gazebo_ros2_control`.
- Añadir `ros_gz_interfaces`, `ros_gz_bridge`, `gz_ros2_control`.

#### [DELETE] `ros2_linkattacher` y `linkattacher_msgs`
- Estos paquetes de Gazebo Classic se recomiendan eliminar a favor de usar la funcionalidad nativa de Gazebo Harmonic `DetachableJoint` (o alternativamente, si insistes, habrá que crear un paquete nuevo con un System Plugin de Ignition).

---

### 2. Actualización de Archivos Launch (`.launch.py`)
La forma de lanzar Gazebo y renderizar modelos cambia por completo.

#### [MODIFY] `xolobot_arm/launch/xolobot_arm_control.launch.py`
- Reemplazar la ejecución de `gazebo` directamente por `IncludeLaunchDescription` usando `ros_gz_sim/launch/gz_sim.launch.py`.
- Cambiar el nodo `spawn_entity.py` de `gazebo_ros` por el nodo `create` de `ros_gz_sim`.
- Configurar el puente de tópicos (`ros_gz_bridge`) para el reloj (`/clock`), las interrupciones del bumper y los controladores.

---

### 3. Modificación del Código C++ y URDF/SDF

#### [MODIFY] Modelos (.urdf y .sdf) en `xolobot_arm/models`
- Actualizar los tags de `<plugin>` para usar los sistemas de Gazebo Harmonic (ej. `<plugin filename="gz-sim-joint-state-publisher-system" ...>`).
- Cambiar los plugins del bumper a `gz-sim-contact-system`.
- Cambiar `gazebo_ros2_control/GazeboSystem` por `gz_ros2_control/GazeboSimSystem`.

#### [MODIFY] `xolobot_arm_server/src/SimulationController.cpp`
- Actualizar el tipo de mensaje de los bumpers de `gazebo_msgs::msg::ContactsState` a `ros_gz_interfaces::msg::Contacts`.
- Modificar la función `agarre_objeto()` para que, en lugar de llamar al servicio `/link_attacher_node/ATTACHLINK`, envíe un mensaje a través del proxy de ROS-GZ para activar el `DetachableJoint` de Harmonic.

#### [MODIFY] `xolobot_arm_server/src/startServerNode.cpp`
- Limpiar código legacy como `setenv("ROS_MASTER_URI", ...)` que pertenece a ROS 1.

---

## Verification Plan

### Automated Tests
- Validar el compilado general en ROS 2 Jazzy mediante `colcon build --packages-select xolobot_arm xolobot_control xolobot_arm_server`.

### Manual Verification
1. **Lanzar la simulación:** Ejecutar `ros2 launch xolobot_arm xolobot_arm_control.launch.py`.
2. **Revisar Gazebo Harmonic:** Verificar que la interfaz web/nativa de Gazebo arranca correctamente sin errores en la consola y carga el mundo `coca_levitando.world`.
3. **Verificar Controladores:** Listar los controladores activos con `ros2 control list_controllers` para confirmar que `joint_state_controller` y `joint_trajectory_controller` (u otros) están activos mediante `gz_ros2_control`.
4. **Verificar Funcionamiento de Bumpers y Agarre:** Inspeccionar tópicos `/bumper_states_*` y ejecutar el nodo servidor (`ros2 launch xolobot_arm_server arm_server.launch.py`) para confirmar que el brazo se mueve, detecta colisión y sujeta el objeto.

---

## 2. Resumen de Hallazgos y Problemas
Durante la ejecución meticulosa de la migración, nos enfrentamos a múltiples desafíos puntuales intrínsecos a los cambios arquitectónicos drásticos introducidos por ROS 2 Jazzy y Gazebo Harmonic en Ubuntu 24.04:

* **Incompatibilidad Fatal del Plugin `link_attacher`:** El plugin personalizado iterado en C++ (`gazebo_ros_link_attacher`) para Gazebo Classic quedó estructural y conceptualmente obsoleto debido a la migración profunda hacia la arquitectura de plugins ECS (Entity Component System) de Harmonic. Fue sustituido exitosamente mediante ingeniería apoyándose en el System Plugin nativo `DetachableJoint` de Harmonic y puenteado mediante `ros_gz_bridge`.
* **Crasheos Silenciosos de GUI (Wayland):** En la nueva versión de Ubuntu (24.04), la GUI nativa basada en Qt de Gazebo Harmonic fallaba estrepitosamente colgándose por conflictos de soporte con el sistema de visuales de Wayland. Se diagnosticó y solucionó forzando la inyección temprana de la variable de entorno `QT_QPA_PLATFORM=xcb` de forma automatizada mediante el ecosistema de ROS 2 en el archivo `.launch.py`.
* **Fallos Estrictos de Parsing en Macros de ROS (`$(find pkg)`) dentro de Archivos SDF:** Identificamos que el comando de inicialización vía argumento `-file` del nodo `ros_gz_sim create` arroja las macros de ROS (o variables directas de XACRO) intermitentemente hacia Gazebo sin resolver, causando un error fatal en el parser de Ruby de Gazebo. La robusta solución arquitectónica fue diseñar inyección y reemplazo de templates dinámicos en Python dentro del pipeline del Launch para pasar el path absoluto in-memory a Gazebo mediante el flag `-string`.
* **Problema de Visibilidad de Mallas Collada (Meshes):** Gazebo Harmonic transiciona estrictamente del entorno variable antiguo a rutear recursos en base a `GZ_SIM_RESOURCE_PATH`. Su configuración inicial demandó localizar dinámicamente la carpeta `models` compartida desde el gestor de paquetes de ROS para que Gazebo localice los archivos `.dae` eficientemente, agregando la ruta `AppendEnvironmentVariable` en el ecosistema launch de ROS.
* **Crasheo Crítico del Motor Físicas (DartSim) originado por Mallas Asimétricas:** El motor físico fundamental DartSim arrojaba `Segmentation fault` (`Address not mapped to object`) a nivel sistema intentando parsear y proyectar los primitivos del `CollisionDetector` con mallas `<mesh>` topológicamente malformadas exportadas. **Solución:** Se codificó y ejecutó un script iterativo en Python (Expresiones Regulares Revertibles) para expurgar y disolver las proyecciones visuales asimétricas de los fragmentos `<collision>...</collision>` en el modelo URDF/SDF pesado, insertando a cambio representaciones geométricas simples matemáticamente ligeras (`<box>`, `<cylinder>`, `<sphere>`).
* **Saturación Inmediata de RAM por Colcon (Congelamientos):** Durante las compilaciones intensas vinculadas a plantillas generativas de C++ provistas en el `xolobot_camera` y otros nodos para ROS 2, la computadora central de desarrollo se sofocaba hasta la asfixia colgando Ubuntu. Se alivianó la infraestructura del CPU limitando forzosamente el consumo total inyectando un job (`export MAKEFLAGS="-j1"`) y compaginándolo imperativamente con compuerta de acceso secuencial `--executor sequential`.
* **Fallas Conceptuales en Activación Spawner de Controladores:** El `joint_trajectory_controller`, fundamental para mover físicamente el apéndice articulado no toleraba la llamada prehistórica `ExecuteProcess` ya que los comandos UNIX encriptaban intermitencias tempranas con el `controller_manager` no inicializado de ROS 2 Jazzy. Se reparó sustituyéndolo programáticamente por el nodo reactivo `spawner`.
* **Duplicidad Jerárquica e Interferencias de Articulaciones Mundo-Base:** El ecosistema emitía docenas de alertas rojas notificando articulaciones cinemáticas fijas clonadas unidas al mundo en la base lógica raíz SDF (`fixed_to_ground` y `fixed_to_ground2`). El documento SDF original fue despurificado limpiando la repetición topológica de manera manual, estabilizando finalmente el árbol base-hijo inquebrantable de ROS.
* **Modelo Fantasma de Librería Ausente:** El render de prehensión referenciaba agresivamente en caché hacia archivos estandarizados predescargados por ROS 1 (`coke_can/meshes/coke_can.dae`); tras depuración, en lugar de invocar colas de descompresión desde Gazebo Fuel cada inicio o incurrir latencia HTTP, se inyectó un render geométrico local mediante primitivas nativas del visualizador `<cylinder>` de color con su radio y altura homóloga local (<ambient>Rojo</ambient>), cortando de tajo latencias visuales de manera elegante.

---

## 3. Comparativa de Código (Antes vs Después) y Justificación

### A. Dependencias de Gazebo (`package.xml`)

**Antes (ROS 2 Iron)**
```xml
<depend>gazebo_ros</depend>
<depend>gazebo_msgs</depend>
<depend>gazebo_plugins</depend>
<depend>gazebo_ros2_control</depend>
```

**Después (ROS 2 Jazzy)**
```xml
<depend>ros_gz_sim</depend>
<depend>ros_gz_bridge</depend>
<depend>ros_gz_interfaces</depend>
<depend>gz_ros2_control</depend>
```
**Justificación:** El metapaquete clásico `gazebo_ros_pkgs` fue descontinuado. La suite correcta y oficial para interoperar ROS 2 Jazzy con Gazebo Harmonic es `ros_gz`.

### B. Ejecución de la Simulación (`.launch.py`)

**Antes (ROS 2 Iron)**
```python
gazebo = ExecuteProcess(
    cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_path],
    output='screen')
spawn_model = Node(
    package='gazebo_ros', executable='spawn_entity.py',
    arguments=['-entity', 'xolobot_arm', '-file', sdf_path]
)
```

**Después (ROS 2 Jazzy)**
```python
gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    ),
    launch_arguments={'gz_args': f"-r -v4 {world_path} --force-version 8"}.items()
)
spawn_model = Node(
    package='ros_gz_sim', executable='create',
    arguments=['-string', sdf_content, '-name', 'xolobot_arm']
)
```
**Justificación:** Harmonic funciona bajo un concepto clúster servidor-cliente. Ahora invocamos su Launch empaquetado y modular oficial en lugar del viejo ejecutable monolítico manual bloqueante `gazebo`, junto con el moderno nodo `create` que transmite por Ignition transport para instanciar en RAM interactuando asincrónicamente mediante `-string` para evitar romper las macros de variables locales `$(find ...)` desde el template.

### C. Configuración del Plugin ROS2 Control Framework (`xolobot_arm.sdf`)

**Antes (ROS 2 Iron)**
```xml
<plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
  <parameters>xolobot_control.yaml</parameters>
</plugin>
```

**Después (ROS 2 Jazzy)**
```xml
<plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
  <parameters>$(find xolobot_control)/config/xolobot_control.yaml</parameters>
</plugin>
```
**Justificación:** El runtime `libgazebo_ros2_control.so` es el legacy wrapper que interceptaba los loops de control. En Gazebo Harmonic es indispensable instanciar e inicializar puntualmente el System Component Plugin reestructurado `gz_ros2_control::GazeboSimROS2ControlPlugin`.

### D. Eliminación Definitiva de Mallas Geométricas .dae en las Simulaciones Colisionables Cóncavas (`xolobot_arm.sdf`)

**Antes (ROS 2 Iron)**
```xml
<collision name='collision'>
  <geometry>
    <mesh>
      <uri>model://meshes/biceps_izq_colision.dae</uri>
    </mesh>
  </geometry>
</collision>
```

**Después (ROS 2 Jazzy)**
```xml
<collision name='collision'>
  <geometry>
    <cylinder>
      <radius>0.04</radius>
      <length>0.20</length>
    </cylinder>
  </geometry>
</collision>
```
**Justificación:** El intento exhaustivo de importar y proyectar visuales Collada nativos de la era Iron con imperfecciones y caras asimétricas causaban una crisis fatal, disparando una avería profunda (Segmentation Fault) rompiendo físicamente la integración al arrancar el framework de simulación DartSim, de forma intrínsecamente ajena. Resumir analíticamente estas topologías y reemplazarlas con geometría volumétrica trivial extirpó el crash instantaneamente.

### E. Inicialización Spawner Control Manager de ROS2

**Antes (ROS 2 Iron)**
```python
load_trajectory_controller = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_trajectory_controller']
)
```

**Después (ROS 2 Jazzy)**
```python
load_trajectory_controller = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_trajectory_controller"]
)
```
**Justificación:** Tratar de interceptar el framework inyectando imperativamente comandos interactivos nativos temporales UNIX destruyendo los semáforos temporizados contra el servidor de callbacks acarreaba muertes prematuras de nodos. Desplegar un wrapper asíncrono formal provisto por la herramienta `spawner` del `controller_manager` estabilizó los arranques permitiendo a ROS autogestionar la inyección a memoria.

### F. Renderización Simulada e Independiente para Objetos Operacionales (Ej. Lata de Refresco)

**Antes (ROS 2 Iron / Gazebo Classic)**
```xml
<visual name='visual'>
  <geometry>
    <mesh>
      <uri>model://coke_can/meshes/coke_can.dae</uri>
    </mesh>
  </geometry>
...
</visual>
```

**Después (ROS 2 Jazzy / Gazebo Harmonic)**
```xml
<visual name='visual'>
  <geometry>
    <cylinder>
      <radius>0.03</radius>
      <length>0.12</length>
    </cylinder>
  </geometry>
  <material>
    <ambient>1 0 0 1</ambient>
    <diffuse>1 0 0 1</diffuse>
  </material>
...
</visual>
```
**Justificación:** Retirar dependencias visuales de redes de Internet o descargas temporales desde servidores mundiales remotos garantiza completa hermeticidad para lanzar simulaciones locales portátiles en ROS 2. Evita errores fatales e incompatibilidad total de render.
