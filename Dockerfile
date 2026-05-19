FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# System + ROS apt dependencies
# apt-get upgrade keeps the pinned ros-jazzy-desktop base image aligned
# with any newer ROS packages pulled in below (e.g. nav2-msgs), preventing
# ABI skew between packages built at different snapshots of the apt repo.
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-diagnostic-aggregator \
    cmake \
    libopencv-dev \
    liblua5.1-0-dev \
    libboost-filesystem-dev \
    libboost-date-time-dev \
    libboost-system-dev \
    python3-colcon-common-extensions \
    git \
    wget \
    qt6-wayland \
    qtwayland5 \
    && rm -rf /var/lib/apt/lists/*

# Build Box2D 2.3.1 from source (Ubuntu 24.04's libbox2d-dev lacks CMake config)
RUN cd /tmp && \
    git clone --branch v2.3.1 --depth 1 https://github.com/erincatto/box2d.git && \
    cd box2d/Box2D/Build && \
    cmake -DBOX2D_BUILD_EXAMPLES=OFF -DBOX2D_BUILD_TESTS=OFF \
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
      -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j"$(nproc)" && make install && \
    rm -rf /tmp/box2d

# Copy upstream flatland source (from git submodule at third_party/flatland)
# and apply our patch stack. See patches/flatland/ for individual patches and
# README ("Updating the flatland patches") for the regeneration workflow.
COPY third_party/flatland/ /ros2_ws/src/flatland/
COPY patches/flatland/ /tmp/patches/flatland/
RUN cd /ros2_ws/src/flatland && \
    rm -rf .git && \
    for p in /tmp/patches/flatland/*.patch; do \
      echo "Applying $(basename "$p")"; \
      git apply --whitespace=nowarn "$p"; \
    done && \
    rm -rf /tmp/patches

# Copy project-owned plugin sources (the patches only register them)
COPY plugins/battery.h /ros2_ws/src/flatland/flatland_plugins/include/flatland_plugins/
COPY plugins/battery.cpp /ros2_ws/src/flatland/flatland_plugins/src/
COPY plugins/camera_math.h /ros2_ws/src/flatland/flatland_plugins/include/flatland_plugins/
COPY plugins/camera.h /ros2_ws/src/flatland/flatland_plugins/include/flatland_plugins/
COPY plugins/camera.cpp /ros2_ws/src/flatland/flatland_plugins/src/

# Copy project-owned republisher package
COPY republisher/ /ros2_ws/src/republisher/

# Copy project-owned diagnostics_watcher package
COPY diagnostics_watcher/ /ros2_ws/src/diagnostics_watcher/

# Install rosdep dependencies and build flatland
# Skip flatland_viz (rviz2 alone handles visualization)
RUN cd /ros2_ws && \
    . /opt/ros/jazzy/setup.sh && \
    (rosdep install --from-paths src --ignore-src -r -y || true) && \
    colcon build --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
      --packages-select flatland_msgs flatland_server flatland_plugins republisher diagnostics_watcher

# Copy project files
COPY launch/ /ros2_ws/src/flatland_nav2_bringup/launch/
COPY config/ /ros2_ws/src/flatland_nav2_bringup/config/
COPY maps/ /ros2_ws/maps/
COPY worlds/ /ros2_ws/worlds/
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /ros2_ws

# Display environment for X11/Wayland forwarding
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

# Github artifacts
LABEL org.opencontainers.image.source=https://github.com/OpenRobOps/sim-flatland
LABEL org.opencontainers.image.description="ROS2 Flatland simulation"
LABEL org.opencontainers.image.licenses=Apache-V2

ENTRYPOINT ["/entrypoint.sh"]
CMD ["--with-rviz"]
