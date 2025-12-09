FROM ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Установка необходимых зависимостей для сборки colcon и gazebo_ros
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Инициализация rosdep
RUN rosdep init || true
RUN rosdep update

# Создание ROS 2 workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Копирование пакета RC10
COPY RC10 /ros2_ws/src/RC10

# Установка зависимостей пакета
RUN . /opt/ros/humble/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y || true

# Сборка workspace через colcon
RUN /bin/bash -c ". /opt/ros/humble/setup.bash && \
    colcon build"

# Добавление setup.bash в .bashrc
RUN echo \"source /opt/ros/humble/setup.bash\" >> ~/.bashrc && \
    echo \"source /ros2_ws/install/setup.bash\" >> ~/.bashrc

WORKDIR /ros2_ws

