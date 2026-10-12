# Thaiquiri Webots

Este README describe la forma de ejecutar y depurar el paquete `thaiquiri_webots` dentro del repositorio `webots_drone`.

No se agregan funcionalidades de otros paquetes; esta guía está enfocada exclusivamente en Thaiquiri.

## Requisitos

- Docker y Docker Compose funcionando.
- Entorno ROS 2 Jazzy disponible dentro del contenedor.
- Webots y la simulación del dron `thaiquiri` configurados en el workspace.

## 1) Iniciar el entorno

Desde la raíz del repositorio:

```bash
cd webots_drone/
docker compose up -d
```

Abre una terminal dentro del contenedor de desarrollo:

```bash
docker compose exec padoru_webots_jazzy bash
```

## 2) Compilar el paquete Thaiquiri

Dentro del contenedor:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/ubuntu/ros_ws
colcon build --packages-select thaiquiri_webots
source install/setup.bash
```

Si quieres compilar el paquete y dejarlo listo para ejecutar inmediatamente, también puedes usar:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## 3) Lanzar la simulación

Ejecuta la simulación del dron Thaiquiri:

```bash
ros2 launch thaiquiri_webots drone_launch
```

Esto levanta el mundo Webots y el driver del dron asociado al paquete `thaiquiri_webots`.

## 4) Controlar el dron con teclado

En otra terminal, dentro del mismo contenedor:

```bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ros_ws/install/setup.bash
ros2 run thaiquiri_webots teleop_drone
```

Esto permite controlar el dron manualmente desde teclado.

## 5) Archivo principal a modificar

Si necesitas ajustar la lógica del controlador del dron, el archivo principal es:

```text
webots_drone/src/thaiquiri_webots/thaiquiri_webots/drone_driver.py
```

También puedes revisar otros nodos del paquete en la misma carpeta si necesitas depurar sensores, odometría o cámara.

## 6) Flujo recomendado

1. Levantar Docker.
2. Entrar al contenedor.
3. Compilar `thaiquiri_webots`.
4. Ejecutar `ros2 launch thaiquiri_webots drone_launch`.
5. Si hace falta, usar `ros2 run thaiquiri_webots teleop_drone`.
6. Modificar el driver del dron en `drone_driver.py` y volver a compilar.

## Consejo de depuración

Tras hacer cambios en el nodo del dron, repite la compilación:

```bash
colcon build --package-select thaiquiri_webots
source install/setup.bash
```

Y luego lanza nuevamente la simulación.
