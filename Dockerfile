FROM osrf/ros:noetic-desktop-full

# Установка необходимых зависимостей
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Инициализация rosdep
RUN rosdep init || true
RUN rosdep update

# Создание catkin workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# Копирование пакета RC10
COPY RC10 /catkin_ws/src/RC10

# Установка зависимостей пакета
RUN rosdep install --from-paths src --ignore-src -r -y || true

# Сборка workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    catkin_make"

# Добавление setup.bash в .bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

WORKDIR /catkin_ws

