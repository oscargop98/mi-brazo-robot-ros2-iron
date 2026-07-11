#!/bin/bash
# setup_aliases.sh — Inyecta los aliases de desarrollo xolobot en ~/.bashrc
# Idempotente: solo escribe si el bloque no existe ya.

MARKER="ALIAS PARA MIGRACION XOLOBOT (ROS 2 JAZZY)"

if grep -qF "$MARKER" ~/.bashrc; then
    echo "[xolobot] El bloque de aliases ya existe en ~/.bashrc. No se hizo ningún cambio."
    exit 0
fi

cat >> ~/.bashrc << 'EOF'

# ==========================================
# ALIAS PARA MIGRACION XOLOBOT (ROS 2 JAZZY)
# ==========================================

# T1: Compilar el servidor y lanzar el entorno de Gazebo
alias xolo_sim="xolo_kill && source /opt/ros/jazzy/setup.bash && cd ~/migration_ws && colcon build --packages-select xolobot_arm_server && source install/setup.bash && ros2 launch xolobot_arm xolobot_arm_control.launch.py"

# T2: Lanzar el cerebro/controlador con la pausa de 3 segundos
alias xolo_brain="source /opt/ros/jazzy/setup.bash && cd ~/migration_ws && source install/setup.bash && ros2 run xolobot_arm_server xolobot_arm_server --ros-args -p use_sim_time:=true"

# Boton de panico para matar todos los procesos de la simulacion
alias xolo_kill="pkill -9 -f ros2; pkill -9 -f gz; pkill -9 -f xolobot_arm_server; echo 'Procesos de ROS 2 y Gazebo aniquilados.'"

# Saber donde esta la camara posicionada
alias xolo_cam="gz topic -e -t /gui/camera/pose"

# Mover Camara
alias xolo_view='xolo_float && gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 2000 --req "pose: {position: {x: 0.212, y: 1.133, z: 0.890} orientation: {x: 0.073, y: 0.085, z: -0.645, w: 0.755}}"'

# Controles Manuales - Sirve para poder calibrar manualmente cada parte del brazo
alias xolo_jtc='source /opt/ros/jazzy/setup.bash; ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller'

# Matar Controles Manuales
alias xolo_kill_jtc='pkill -9 -f rqt_joint_trajectory_controller'

# Depuracion
alias xolo_dep="source /opt/ros/jazzy/setup.bash"
EOF

echo "[xolobot] Aliases inyectados correctamente."
echo "[xolobot] Ejecuta: source ~/.bashrc"
