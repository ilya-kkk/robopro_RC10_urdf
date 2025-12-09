#!/bin/bash

# Скрипт для запуска ROS модели RC10 в Docker

# Создание X11 auth файла для GUI приложений
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]; then
    touch $XAUTH
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
    chmod 666 $XAUTH
fi

# Запуск контейнера
docker-compose up -d

# Ожидание запуска контейнера
sleep 2

echo "=========================================="
echo "ROS 2 Humble контейнер запущен!"
echo "=========================================="
echo ""
echo "Для запуска модели используйте:"
echo ""
echo "1. Открыть терминал в контейнере:"
echo "   docker exec -it rc10-robot bash"
echo ""
echo "2. В контейнере запустить:"
echo "   source /opt/ros/humble/setup.bash"
echo "   source /ros2_ws/install/setup.bash"
echo ""
echo "3. Для отображения в RViz2:"
echo "   ros2 launch RC10 display.launch.py"
echo ""
echo "4. Для запуска в Gazebo:"
echo "   ros2 launch RC10 gazebo.launch.py"
echo ""
echo "=========================================="

