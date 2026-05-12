# Contexto Global - Proyecto Brazo Robótico Xolobot

Este archivo contiene las reglas y contextos mandatorios para el desarrollo y mantenimiento del proyecto de migración hacia ROS 2 Jazzy.

## Reglas Obligatorias

1. **Entorno de Simulación:** 
   - **ROS 2:** Jazzy Jalisco.
   - **Simulador:** Gazebo Harmonic (gz-sim8). Ya NO se usa Gazebo Classic.

2. **C++ y Control de Nodos:**
   - Todo nodo basado en temporizadores (`rclcpp::TimerBase`) debe utilizar la API `create_timer` sincronizada usando el tiempo del entorno.
   - Es mandatorio ejecutar el entorno y los nodos con el parámetro y flag **`use_sim_time:=true`**. De lo contrario, `joint_trajectory_controller` dropeará (descartará) los comandos.

3. **Inyección de Trayectorias y Posiciones:**
   - Las trayectorias no deben ir apuntadas estáticamente ("hardcodeadas") al cero absoluto (`0.0`).
   - Todos los alcances del end-effector deben llevar un factor matemático o una compensación trigonométrica (ej. `jnt_hombro_hombro = 0.758 rads`) calculada basada en las coordenadas `X`, `Y` posicionales del objeto a interactuar.
