# Proyecto de Migración ROS 2 (Iron ➡️ Jazzy) y Gazebo (Classic ➡️ Harmonic)
### Bitácora 0002
**Autor:** Oscar David Gonzalez Pintor SS

## Índice
1. [Respaldo del Código en GitHub](#1-respaldo-del-código-en-github)
2. [Gestión de la Bitácora con Enlaces Simbólicos](#2-gestión-de-la-bitácora-con-enlaces-simbólicos)
3. [Autenticación con PAT en GitHub](#3-autenticación-con-pat-en-github)
4. [Resumen de Contexto para la IA (Context Restoration)](#4-resumen-de-contexto-para-la-ia-context-restoration)

---

## 1. Respaldo del Código en GitHub

Tras concluir la estabilización del entorno, los modelos (URDF/SDF), archivos Launch y de configuración (`CMakeLists.txt`), se procedió a proteger y versionar todos estos avances estructurales que demuestran una simulación libre de crasheos en Gazebo Harmonic.

Para esto, validamos nuestra ubicación en la rama activa correcta y documentamos nuestro progreso en un _commit_ de trabajo en progreso (**WIP**) ya que, aunque la arquitectura y los modelos levantaban correctamente, aún faltaba la validación cinemática completa y el accionar del `DetachableJoint`.

**Comandos utilizados:**
```bash
# Validar el estado del árbol de trabajo y el repositorio remoto
git status
git remote -v

# Agregar todos los cambios (URDF, CMakeLists, Launch, etc.) al área de preparación
git add -A

# Confirmación parcial de seguridad
git commit -m "WIP: Entorno visual, modelos y servidor migrados a Jazzy/Harmonic. Entorno estable sin crasheos. Pendiente validación de cinemática y agarre."
```

---

## 2. Gestión de la Bitácora con Enlaces Simbólicos

Anteriormente, los reportes de progreso y las bitácoras se almacenaban aislados en el `Escritorio`. Esto supone un riesgo de pérdida de documentación valiosa, ya que Git no rastrea carpetas externas al repositorio. 

Para acoplarnos a las mejores prácticas de la ingeniería de software, se tomó la decisión técnica de **mover físicamente** el directorio de la bitácora hacia el interior del repositorio local del proyecto. Al mismo tiempo, para no afectar la ergonomía ni el flujo de trabajo del desarrollador (que prefiere acceder rápidamente a los logs desde el Escritorio), se instauró un **Enlace Simbólico** (`symlink`).

Esta es una técnica de sistema de archivos robusta en Linux que instruye al sistema operativo a reflejar todas las lecturas/escrituras de una ruta (Escritorio) hacia otra (Workspace). De esta manera, el repositorio de Git puede rastrear y subir al servidor la documentación de forma orgánica, mientras el ingeniero conserva un acceso directo y transparente.

**Comandos utilizados:**
```bash
# Mover físicamente el directorio original al workspace rastreado por Git
mv ~/Desktop/Bitacora/ ~/migration_ws/src/mi-brazo-robot-ros2-iron/Bitacora/

# Crear el enlace simbólico blando ('soft link') en el Escritorio apuntando a la nueva ubicación
ln -s ~/migration_ws/src/mi-brazo-robot-ros2-iron/Bitacora ~/Desktop/Bitacora
```

---

## 3. Autenticación con PAT en GitHub

Al pretender ejecutar la sincronización del punto de control hacia la nube (`git push`), se superó la restricción moderna de autenticación tradicional usando contraseña (descontinuada por GitHub por razones de seguridad). Para establecer la integración del repositorio desde la línea de comandos, se generó y empleó un **Personal Access Token (PAT) Clásico**.

Este estándar es más seguro ya que restringe el acceso limitando los permisos a operaciones específicas, en este caso, se otorgó el control sobre el alcance `repo` (Full control of private/public repositories).

**Comandos utilizados y de entorno:**
```bash
# Empuje de la rama local hacia el repositorio remoto origin
git push origin jazzy-migration

# (El sistema solicita el 'Username' y 'Password'. Se provee el usuario de Git y en la contraseña se ingresa el Token PAT recién generado)
```

---

## 4. Resumen de Contexto para la IA (Context Restoration)

**Estado Técnico Ultracompacto:**
- **Nivel Arquitectura:** Migración de ROS 2 Iron a Jazzy completada orgánicamente.
- **Simulador:** Gazebo Harmonic ejecuta de forma estable utilizando DartSim y proyecciones de mallas simplificadas a primitivas puras (ej. cilindro rojo nativo para emular la lata sin latencia).
- **Plugins:** El obsoleto plugin encadenable `ros2_linkattacher` fue erradicado y delegado completamente al sistema nativo `DetachableJoint`.
- **Paso Inmediato Actual:** Lanzar el nodo C++ `xolobot_arm_server` (`ros2 launch xolobot_arm_server arm_server.launch.py`) en la tercera terminal para probar la generación de la cinemática; esto desencadenará el *timer* de 700ms enviando el brazo hacia la colisión para detonar así la intercepción automática y probar el agarre.
