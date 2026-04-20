FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# System dependencies
RUN apt-get update && apt-get install -y \
    build-essential cmake git wget curl \
    python3-pip python3-colcon-common-extensions \
    python3-rosdep python3-vcstool \
    ros-jazzy-rclcpp ros-jazzy-geometry-msgs \
    ros-jazzy-sensor-msgs ros-jazzy-std-msgs \
    ros-jazzy-nav-msgs ros-jazzy-cv-bridge \
    ros-jazzy-joy ros-jazzy-teleop-twist-joy \
    ros-jazzy-nmea-navsat-driver \
    libopencv-dev python3-opencv \
    libusb-1.0-0-dev libudev-dev \
    && rm -rf /var/lib/apt/lists/*

# Python packages
RUN pip3 install --break-system-packages \
    numpy \
    pyyaml \
    opencv-python-headless

# Build AriaCoda from source
WORKDIR /opt
RUN git clone https://github.com/reedhedges/AriaCoda.git
WORKDIR /opt/AriaCoda
RUN make -j2
RUN make install
# Ensure libAria.so is findable
RUN ldconfig

# ROS workspace
ENV ROS_WS=/ros_ws
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

# Copy packages into workspace
COPY ariaNode/ src/ariaNode/
COPY part2_nav/ src/part2_nav/

# Fix ariaNode CMakeLists to find AriaCoda headers + lib
RUN sed -i 's|target_link_libraries(\n  ariaNode\n  libAria.so\n)|target_link_libraries(ariaNode /usr/local/lib/libAria.so)|' \
    src/ariaNode/CMakeLists.txt || true

# Build workspace
RUN source /opt/ros/jazzy/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_CXX_FLAGS="-I/usr/local/include" \
            -DCMAKE_EXE_LINKER_FLAGS="-L/usr/local/lib"

# Entrypoint
COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

ENV ROS_DOMAIN_ID=9
ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]
