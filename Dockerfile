FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# Установка необходимых зависимостей для сборки colcon и gazebo_ros
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    build-essential \
    ros-humble-rviz2 \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-tf2-ros \
    ros-humble-gazebo-ros \
    && rm -rf /var/lib/apt/lists/*

# # Инициализация rosdep: очищаем старый sources.list, чтобы не было конфликта
# RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list && rosdep init
# # rosdep update с одним повтором; если оба раза упали — провалить сборку
# RUN rosdep update || (sleep 3 && rosdep update)

# Создание ROS 2 workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Копирование пакета RC10
COPY ./RC10 /ros2_ws/src/RC10

# Установка зависимостей пакета (обновляем apt кэш перед rosdep install)
RUN apt-get update && \
    . /opt/ros/humble/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y

# Сборка workspace через colcon
RUN /bin/bash -c ". /opt/ros/humble/setup.bash && \
    colcon build"

# Подстраховка: кладём ресурс пакета в ament index вручную, если CMake не положил
RUN mkdir -p /ros2_ws/install/share/ament_index/resource_index/packages && \
    cp /ros2_ws/src/RC10/resource/RC10 /ros2_ws/install/share/ament_index/resource_index/packages/RC10 || true

# Проверяем, что пакет RC10 установлен и зарегистрирован в ament index.
# AMENT_TRACE_SETUP_FILES может быть не задан, поэтому не используем "set -u".
RUN /bin/bash -lc "set -eo pipefail; \
    export AMENT_TRACE_SETUP_FILES=; \
    source /opt/ros/humble/setup.bash; \
    source /ros2_ws/install/setup.bash; \
    echo '--- RC10 share contents ---'; \
    ls -la /ros2_ws/install/RC10/share/RC10; \
    echo '--- ament index packages ---'; \
    ls -la /ros2_ws/install/share/ament_index/resource_index/packages; \
    echo '--- ros2 pkg prefix RC10 ---'; \
    ros2 pkg prefix RC10"

# Добавление setup.bash в .bashrc (без лишних кавычек)
RUN echo 'if [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; fi' >> /root/.bashrc && \
    echo 'if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi' >> /root/.bashrc

WORKDIR /ros2_ws

