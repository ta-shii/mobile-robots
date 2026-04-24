FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive
ENV QT_X11_NO_MITSHM=1
ARG ARIACODA_REF=master

# -------------------------
# System dependencies
# -------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    git \
    nano \
    wget \
    build-essential \
    cmake \
    make \
    pkg-config \
    zip \
    unzip \
    iproute2 \
    iputils-ping \
    joystick \
    usbutils \
    libusb-1.0-0-dev \
    python3-colcon-common-extensions \
    python3-pip \
    python3-venv \
    python3-opencv \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# -------------------------
# Add ROS testing repo (needed for depthai ROS v3)
# -------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros2-testing-apt-source \
    && rm -rf /var/lib/apt/lists/*

# -------------------------
# ROS packages
# -------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-joy \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-tf2 \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-sensor-msgs-py \
    ros-jazzy-rclcpp \
    ros-jazzy-rclpy \
    ros-jazzy-launch \
    ros-jazzy-launch-ros \
    ros-jazzy-std-msgs \
    ros-jazzy-nmea-navsat-driver \
    ros-jazzy-robot-localization \
    ros-jazzy-tf-transformations \
    ros-jazzy-phidgets-spatial \
    ros-jazzy-cv-bridge \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rviz2 \
    ros-jazzy-depthai-ros-v3 \
    && rm -rf /var/lib/apt/lists/*

# -------------------------
# Python packages for project nodes
# -------------------------
RUN pip3 install --break-system-packages \
    matplotlib \
    pyyaml \
    numpy \
    depthai

# -------------------------
# rosdep init (safe in Docker)
# -------------------------
RUN rosdep init || true && rosdep update || true

# -------------------------
# AriaCoda (Pioneer robot library)
# -------------------------
RUN set -e; \
    for i in 1 2 3; do \
        rm -rf /tmp/AriaCoda && \
        git clone --depth 1 --single-branch --branch ${ARIACODA_REF} \
            https://github.com/reedhedges/AriaCoda.git /tmp/AriaCoda \
        && break || { echo "Clone attempt $i/3 failed"; [ $i -lt 3 ] && sleep 10; }; \
    done \
    && make -C /tmp/AriaCoda -j"$(nproc)" lib/libAria.so lib/libAria.a params \
    && make -C /tmp/AriaCoda install-default \
    && ldconfig \
    && test -f /usr/local/lib/libAria.so || { echo "AriaCoda install failed"; exit 1; } \
    && rm -rf /tmp/AriaCoda

# -------------------------
# Create project workspace path
# -------------------------
RUN mkdir -p /workspace/autonomous_nav

# -------------------------
# Environment defaults
# -------------------------
ENV ROS_DOMAIN_ID=9
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV LD_LIBRARY_PATH=/usr/local/lib
ENV RCUTILS_COLORIZED_OUTPUT=1

# -------------------------
# Bash environment for interactive shells
# -------------------------
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=9" >> /root/.bashrc && \
    echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /root/.bashrc && \
    echo "cd /workspace/autonomous_nav" >> /root/.bashrc && \
    echo "if [ -f /workspace/autonomous_nav/install/setup.bash ]; then source /workspace/autonomous_nav/install/setup.bash; fi" >> /root/.bashrc

# -------------------------
# Entrypoint
# -------------------------
COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

WORKDIR /workspace/autonomous_nav
ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]