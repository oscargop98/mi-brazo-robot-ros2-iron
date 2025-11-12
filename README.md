# Brazo robótico antropomórfico 🦾
## src
Carpeta principal de este repositorio.
---
# INICAR EL PROYECTO

## TERMINAL 1 - PROYECTO CON LAUNCH
```cd ~/ros2_ws/
colcon build
source install/setup.bash
ros2 launch xolobot_arm xolobot_arm_control.launch.py # Inicia la simulación. Este comando mantendrá esta terminal ocupada
```
## TERMINAL 2 - EL SERIVIDOR
```
cd ~/ros2_ws/
colcon build
source install/setup.bash
ros2 run xolobot_arm_server xolobot_arm_server # Como la simulación ya está corriendo, el controller_manager estará activo y te responderá
```

## Paso 3
```
cd ~/ros2_ws/
colcon build
source install/setup.bash
ros2 control list_controllers
```
## Nuevos comandos
```
ros2 control list_controllers

ros2 pkg list | grep ros2_control
    #gazebo_ros2_control
    #ros2_control
    #ros2_control_test_assets
    #ros2_controllers'' 
ros2 pkg list | grep libgazebo_ros2_control.so 

ros2 pkg list | grep joint_trajectory_controller
    # joint_trajectory_controller

# Matar los procesos
pkill -9 gzserver
pkill -9 gzclient
pkill -9 gazebo
```
---
## Comandos Anteriores
### Para lanzar el mundo - ejecutar 
1. colcon build
2. source install/setup.bash
3. ros2 launch xolobot_arm xolobot_arm_control.launch.py
4. ros2 run xolobot_arm_server xolobot_arm_server

### Para escuchar tópicos
1. ros2 topic echo /bumper_states

### Para ver los controladores disponibles
1. ros2 control list_controllers

### Para matar procesos
1. pkill -9 gzserver
2. pkill -9 gzclient
3. pkill -9 gazebo

### Paquete de Python
ros2 pkg create --build-type ament_python --license Apache-2.0 modulos

## src_lata
Esta es una carpeta que puede servir de prueba. Tiene el proyecto del brazo robótico antropomórfico que carga la lata y simula el agarre.
