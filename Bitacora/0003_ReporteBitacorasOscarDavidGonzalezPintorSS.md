# Reporte de Bitácora - Xolobot ROS 2
**Fecha:** 30 Mar 2026 (Reporte de avances del día anterior)
**Autor:** Oscar David González Pintor (SS)
**Entorno:** ROS 2 Jazzy, Gazebo Harmonic

## Avances Críticos Logrados

Durante la sesión anterior, logramos dos avances fundamentales para el proyecto de migración y la estabilidad del movimiento autónomo:

### 1. Sincronización de Reloj (Tick Rate Update)
- **Problema:** En Jazzy/Harmonic, las trayectorias con el `joint_trajectory_controller` eran ignoradas o llegaban con retraso porque el reloj del nodo de simulación no estaba alineado con el tiempo de Gazebo.
- **Solución:** Reemplazamos `create_wall_timer` por `create_timer` en el nodo de control C++ (`SimulationController.cpp`). Además, aseguramos la ejecución con `use_sim_time:=true` en todos los nodos (Controlador, Node, y Bridge) como un requisito mandatorio en arquitecturas ROS 2 control - Gazebo Harmonic. Esto garantiza la fluidez de las trayectorias sin descartar comandos.

### 2. Parche Trigonométrico (Compensación de Desfase)
- **Problema:** Al enviar el brazo robot a atrapar la lata de refresco, existía un desfase lateral en el eje Y (Y = 0.25), haciendo que el manipulador pasara de largo respecto a la posición física del objeto en Gain.
- **Solución:** Inyectamos una compensación en radianes, modificando estáticamente un offset de **0.758 radianes (43.4 grados)** en la articulación `jnt_hombro_hombro`. Este valor fue calculado trigonométricamente y permite al brazo centralizar su aproximación exacta sobre la lata.
