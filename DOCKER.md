# Xolobot Arm — Entorno Containerizado

Entorno de simulación completo para el brazo robótico Xolobot encapsulado en Docker.  
Incluye ROS 2 Jazzy, Gazebo Harmonic, el bridge de comunicación y el nodo de control autónomo,
pre-compilados dentro de la imagen para que el arranque sea inmediato.

---

## Requisitos del host

| Requisito | Versión mínima |
|---|---|
| Ubuntu | 22.04 / 24.04 (nativo o VM con X11) |
| Docker Engine | 20.x o superior |
| Docker Compose | v2.x o superior |
| GPU | Cualquier tarjeta con soporte OpenGL (NVIDIA o AMD/Intel via DRI) |

### Instalar Docker en Ubuntu 24.04

```bash
sudo apt-get update
sudo apt-get install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
    | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
    https://download.docker.com/linux/ubuntu noble stable" \
    | sudo tee /etc/apt/sources.list.d/docker.list
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin

# Agregar tu usuario al grupo docker (requiere reiniciar sesión)
sudo usermod -aG docker $USER
newgrp docker
```

---

## Pasos de despliegue

### 1. Clonar el repositorio

```bash
git clone https://github.com/oscargop98/mi-brazo-robot-ros2-iron.git
cd mi-brazo-robot-ros2-iron
```

### 2. Construir la imagen (primera vez — ~10-15 min)

```bash
docker compose build
```

La imagen resultante (`xolobot:jazzy`) contiene el workspace ya compilado.
Las ejecuciones posteriores son instantáneas — no recompila a menos que cambie el código fuente.

### 3. Habilitar el forwarding gráfico

Ejecutar **una vez por sesión** de trabajo antes de levantar cualquier contenedor:

```bash
xhost +local:docker
```

Esto permite que la ventana de Gazebo aparezca en el escritorio del host.

---

## Lanzar la simulación

La simulación requiere **dos terminales en orden**:

### Terminal 1 — Gazebo + entorno completo

```bash
docker compose up sim
```

Espera a ver el mensaje del JointTrajectoryController activo antes de continuar.

### Terminal 2 — Controlador autónomo

```bash
docker compose run brain
```

El nodo espera ~15 segundos (warm-up) antes de enviar la trayectoria inicial.  
Verás `¡Conexión lista! Enviando pose maestra a la lata...` cuando arranque.

---

## Flujo de trabajo completo

```
[host]$ xhost +local:docker

[T1]$ docker compose up sim
      → Gazebo abre ventana 3D
      → JTC activo

[T2]$ docker compose run brain
      → Brazo se aproxima a la lata con waypoint
      → CONTACTO CONFIRMADO → imán activado → elevación suave
```

---

## Comandos de diagnóstico dentro del contenedor

```bash
# Abrir shell interactivo en el contenedor de simulación
docker compose exec sim bash

# Dentro del contenedor:
ros2 control list_controllers          # verificar JTC activo
ros2 topic echo /clock                 # verificar sincronía de reloj
ros2 topic echo /bumper_states_palma   # verificar sensores de contacto
gz topic -l                            # listar tópicos de Gazebo
```

---

## Detener todo

```bash
# Detener ambos contenedores
docker compose down

# Matar procesos colgados de forma inmediata
docker compose kill
```

---

## Reconstruir la imagen tras cambios de código

```bash
# Solo si modificaste src/ o los archivos de configuración
docker compose build --no-cache
docker compose up sim
```

---

## Estructura de archivos relevantes

```
mi-brazo-robot-ros2-iron/
├── Dockerfile              ← Imagen de 5 capas (deps + build)
├── docker-compose.yml      ← Servicios sim y brain
├── docker-entrypoint.sh    ← Source automático de ROS al arrancar
├── setup_aliases.sh        ← Aliases de desarrollo inyectados en la imagen
├── DOCKER.md               ← Este archivo
└── src/
    ├── xolobot_arm/        ← Modelo, mundo Gazebo, launch, bridge
    ├── xolobot_arm_server/ ← Nodo C++ de control autónomo
    └── xolobot_control/    ← Configuración JointTrajectoryController
```
