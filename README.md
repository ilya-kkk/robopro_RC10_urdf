# RC10 Robot ROS Package

ROS 2 Humble пакет для робота RC10, экспортированного из SolidWorks.

## Запуск в Docker

### Быстрый старт

1. **Сборка и запуск контейнера (ROS 2 Humble):**
```bash
./run.sh
```

2. **Войти в контейнер:**
```bash
docker exec -it rc10-robot bash
```

3. **В контейнере активировать ROS 2 workspace:**
```bash
source /catkin_ws/devel/setup.bash
```

4. **Запустить модель (ROS 2 Humble):**

   **Вариант 1: Отображение в RViz2 (с GUI для управления суставами)**
   ```bash
   ros2 launch RC10 display.launch.py
   ```

   **Вариант 2: Запуск в Gazebo симуляторе**
   ```bash
   ros2 launch RC10 gazebo.launch.py
   ```

### Ручной запуск

Если хотите запустить вручную:

```bash
# Создать X11 auth
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
chmod 666 $XAUTH

# Собрать образ
docker-compose build

# Запустить контейнер
docker-compose up -d

# Войти в контейнер
docker exec -it rc10-robot bash
```

### Остановка

```bash
docker-compose down
```

## Структура пакета

- `RC10/urdf/RC10.urdf` - URDF описание робота
- `RC10/meshes/` - 3D модели (STL файлы)
- `RC10/launch/display.launch.py` - Launch файл для RViz2 (ROS 2)
- `RC10/launch/gazebo.launch.py` - Launch файл для Gazebo (ROS 2)
- `RC10/config/` - Конфигурационные файлы

## Требования

- Docker
- Docker Compose
- X11 сервер (для GUI приложений)

## Примечания

- Для работы GUI приложений (RViz, Gazebo) необходим доступ к X11 серверу
- Контейнер использует `network_mode: host` для удобной работы с ROS 2 (DDS)
- Пакет автоматически собирается при создании образа с помощью `colcon build` (ROS 2)

# robopro_RC10_urdf
