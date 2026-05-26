FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive
ENV QT_X11_NO_MITSHM=1
ARG ARIACODA_REF=master

# System dependencies
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
    usbutils \
    libusb-1.0-0-dev \
    python3-colcon-common-extensions \
    python3-pip \
    python3-venv \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Add ROS testing repo (needed for depthai ROS v3)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros2-testing-apt-source \
    && rm -rf /var/lib/apt/lists/*

# ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-joy \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-tf2 \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-tf2-sensor-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-sensor-msgs-py \
    ros-jazzy-rclcpp \
    ros-jazzy-rclpy \
    ros-jazzy-foxglove-bridge \
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
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-rosbag2 \
    ros-jazzy-rosbag2-storage-mcap \
    ros-jazzy-pcl-conversions \
    ros-jazzy-pcl-ros \
    ros-jazzy-pcl-msgs \
    ros-jazzy-vision-opencv \
    ros-jazzy-image-transport \
    ros-jazzy-depth-image-proc \
    && rm -rf /var/lib/apt/lists/*

# Tesseract OCR with Greek language pack
# 
RUN apt-get update && apt-get install -y --no-install-recommends \
    tesseract-ocr \
    tesseract-ocr-ell \
    python3-pyqt5 \
    && rm -rf /var/lib/apt/lists/*

# Python packages
RUN pip3 install --break-system-packages --ignore-installed \
    "numpy<2" \
    matplotlib \
    "opencv-python<4.10" \
    depthai \
    pytesseract \
    transformers \
    accelerate \
    Pillow

# PyTorch (CPU-only build — separate index URL required)
RUN pip3 install --break-system-packages \
    torch torchvision \
    --index-url https://download.pytorch.org/whl/cpu

# Lakibeam ROS2 driver
RUN mkdir -p /workspace/autonomous_nav/ros2_ws/src && \
    git clone --depth 1 \
        https://github.com/RichbeamTechnology/Lakibeam_ROS2_Driver.git \
        /workspace/autonomous_nav/ros2_ws/src/lakibeam1_ros2 || \
    echo "WARNING: Lakibeam clone failed – add manually if needed"

# m-explore ROS2 frontier exploration
RUN mkdir -p /workspace/autonomous_nav/ros2_ws/src && \
    git clone --depth 1 \
        https://github.com/robo-friends/m-explore-ros2.git \
        /workspace/autonomous_nav/ros2_ws/src/m-explore-ros2 || \
    echo "WARNING: m-explore clone failed – add manually if needed"
    
# rosdep init
RUN rosdep init || true && rosdep update || true

# AriaCoda (Pioneer robot library) - stable build version
RUN set -e; \
    rm -rf /tmp/AriaCoda && \
    git clone --depth 1 https://github.com/reedhedges/AriaCoda.git /tmp/AriaCoda || \
    (echo "WARNING: AriaCoda clone failed - build will continue but Aria may be missing" && exit 0)

RUN if [ -d /tmp/AriaCoda ]; then \
        make -C /tmp/AriaCoda -j$(nproc) lib/libAria.so lib/libAria.a params && \
        make -C /tmp/AriaCoda install-default && \
        ldconfig && \
        test -f /usr/local/lib/libAria.so || echo "WARNING: AriaCoda build incomplete"; \
    else \
        echo "WARNING: AriaCoda not present - skipping build"; \
    fi

# Create workspace path
RUN mkdir -p /workspace/autonomous_nav

# Environment defaults
ENV ROS_DOMAIN_ID=9
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV LD_LIBRARY_PATH=/usr/local/lib
ENV RCUTILS_COLORIZED_OUTPUT=1

# Bash environment
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=9" >> /root/.bashrc && \
    echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /root/.bashrc && \
    echo "cd /workspace/autonomous_nav" >> /root/.bashrc && \
    echo "if [ -f /workspace/autonomous_nav/install/setup.bash ]; then source /workspace/autonomous_nav/install/setup.bash; fi" >> /root/.bashrc

# Entrypoint
COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

WORKDIR /workspace/autonomous_nav
ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]