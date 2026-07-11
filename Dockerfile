# ==============================================================
# Xolobot Arm — ROS 2 Jazzy + Gazebo Harmonic
# Imagen de producción con workspace pre-compilado
# Ejecutar desde la raíz del repositorio: docker compose build
# ==============================================================

# ── CAPA 1: Base oficial ROS 2 Jazzy con herramientas de escritorio ──────────
FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

# ── CAPA 2: Repositorio de Gazebo Harmonic + paquetes del sistema ────────────
RUN apt-get update && apt-get install -y curl && \
    curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
         -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
         http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
         > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y \
        # Gazebo Harmonic
        gz-harmonic \
        # Puentes ROS 2 ↔ Gazebo
        ros-jazzy-ros-gz-sim \
        ros-jazzy-ros-gz-bridge \
        ros-jazzy-ros-gz-interfaces \
        # Control
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-gz-ros2-control \
        ros-jazzy-controller-manager \
        ros-jazzy-hardware-interface \
        ros-jazzy-control-msgs \
        # Robot description
        ros-jazzy-robot-state-publisher \
        ros-jazzy-joint-state-publisher \
        ros-jazzy-xacro \
        # Calibración manual
        ros-jazzy-rqt-joint-trajectory-controller \
        # Herramientas de build
        python3-colcon-common-extensions \
        python3-rosdep \
        # Utilidades de diagnóstico
        ros-jazzy-rqt-robot-monitor \
        gz-tools2 \
    && rm -rf /var/lib/apt/lists/*

# ── CAPA 3: rosdep (inicializar si no está ya) ───────────────────────────────
RUN rosdep init 2>/dev/null || true && rosdep update

# ── CAPA 4: Workspace — copia, rosdep install y colcon build ─────────────────
WORKDIR /ros2_ws

COPY src/ src/

RUN . /opt/ros/jazzy/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    # Limpiar directorios de build para reducir tamaño de imagen
    rm -rf build/ log/

# ── CAPA 5: Entorno de ejecución ──────────────────────────────────────────────
# Sources automáticos en shells interactivos
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash"  >> /root/.bashrc

# Aliases del proyecto (idempotente via setup_aliases.sh)
COPY setup_aliases.sh /setup_aliases.sh
RUN bash /setup_aliases.sh

# Punto de entrada: sourcea ROS antes de cualquier comando
COPY docker-entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
