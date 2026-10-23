# syntax=docker/dockerfile:1
# Слои: base (ROS 2 Jazzy + Gazebo Harmonic) → px4 (PX4 SITL на коммите) →
# deps (px4_msgs + Micro-XRCE-DDS Agent) → app (проект).
# Зафиксированные версии — только здесь. PX4 и px4_msgs меняются вместе:
# сообщения должны совпадать с прошивкой. Изменение версии пересобирает её
# слой и всё выше, нижние слои берутся из кэша.
ARG PX4_COMMIT=878877ad022109cc4af7fea566efa0692c43813c
ARG PX4_MSGS_COMMIT=2e892300d8c20d3c29e198e324a10854b40ab8d3
ARG XRCE_AGENT_VERSION=v3.0.2

FROM osrf/ros:jazzy-desktop AS base
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
# Gazebo Harmonic из репозитория OSRF — как на рабочей машине разработки.
RUN apt-get update && apt-get install -y --no-install-recommends curl gnupg lsb-release sudo git \
 && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
    > /etc/apt/sources.list.d/gazebo-stable.list \
 && apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
    gz-harmonic python3-gz-transport13 python3-gz-msgs10 \
    ros-jazzy-ros-gz ros-jazzy-rosbag2-storage-mcap \
    python3-scipy python3-numpy python3-yaml python3-matplotlib python3-pip \
    libgl1-mesa-dri libegl1 mesa-utils \
 && pip3 install --no-cache-dir --break-system-packages pymavlink==2.4.50 \
 && rm -rf /var/lib/apt/lists/*

FROM base AS px4
ARG PX4_COMMIT
# Зависимости PX4 из pip не должны заменять NumPy из apt: SciPy, Matplotlib и
# ROS собраны под 1.26 (на машине разработки pip также оставил 1.26.4).
RUN echo "numpy==1.26.4" > /etc/pip-constraints.txt
ENV PIP_CONSTRAINT=/etc/pip-constraints.txt
# Установщик PX4 скачивает ключ Gazebo через wget и проверяет систему dmidecode.
RUN apt-get update && apt-get install -y --no-install-recommends wget dmidecode \
 && rm -rf /var/lib/apt/lists/*
# Три шага вместо одного: при сбое повтор начинается с упавшего, а не с клонирования.
RUN test -n "$PX4_COMMIT" \
 && git clone https://github.com/PX4/PX4-Autopilot.git /opt/PX4-Autopilot \
 && cd /opt/PX4-Autopilot && git checkout "$PX4_COMMIT" \
 && git submodule update --init --recursive --jobs 8
# Только SITL: инструменты NuttX для прошивки плат не нужны.
RUN cd /opt/PX4-Autopilot && bash Tools/setup/ubuntu.sh --no-nuttx \
 && rm -rf /var/lib/apt/lists/* /root/.cache
RUN cd /opt/PX4-Autopilot && make px4_sitl

FROM px4 AS deps
ARG PX4_MSGS_COMMIT
ARG XRCE_AGENT_VERSION
# px4_msgs строго под коммит PX4; отдельное рабочее пространство /opt/px4_ws.
RUN test -n "$PX4_MSGS_COMMIT" \
 && git clone https://github.com/PX4/px4_msgs.git /opt/px4_ws/src/px4_msgs \
 && git -C /opt/px4_ws/src/px4_msgs checkout "$PX4_MSGS_COMMIT" \
 && source /opt/ros/jazzy/setup.bash && cd /opt/px4_ws \
 && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && rm -rf build log
RUN test -n "$XRCE_AGENT_VERSION" \
 && git clone --depth 1 --branch "$XRCE_AGENT_VERSION" \
    https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/agent \
 && cmake -S /tmp/agent -B /tmp/agent/build -DCMAKE_BUILD_TYPE=Release \
 && cmake --build /tmp/agent/build -j"$(nproc)" && cmake --install /tmp/agent/build \
 && ldconfig && rm -rf /tmp/agent

FROM deps AS app
RUN apt-get update && apt-get install -y --no-install-recommends python3-pytest \
 && rm -rf /var/lib/apt/lists/*
# Пользователь ubuntu (UID 1000) образа совпадает с типичным пользователем
# WSL: файлы в смонтированном log/ не становятся собственностью root.
COPY --chown=ubuntu:ubuntu src /ws/src
COPY --chown=ubuntu:ubuntu scripts /ws/scripts
COPY --chown=ubuntu:ubuntu docker/entrypoint.sh /ws/docker/entrypoint.sh
USER ubuntu
WORKDIR /ws
RUN source /opt/ros/jazzy/setup.bash && source /opt/px4_ws/install/setup.bash \
 && colcon build && rm -rf build/*/test_results
ENV PX4_DIR=/opt/PX4-Autopilot
ENTRYPOINT ["/ws/docker/entrypoint.sh"]
CMD ["ros2", "launch", "swarm_lidar", "swarm_demo.launch.py", "gui:=false", \
     "web_host:=0.0.0.0", "open_browser:=false"]
