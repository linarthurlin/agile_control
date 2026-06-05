# ==========================================
# Stage 1: Build Stage
# ==========================================
FROM ros:humble-ros-base AS builder

SHELL ["/bin/bash", "-c"]

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Add AutonomouStuff public APT repository for pacmod2 binary dependency
RUN echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ jammy main" > /etc/apt/sources.list.d/autonomoustuff-public.list

# Install compilation toolchains, rosdep, git, PCL, Eigen development headers, and pacmod2 driver binaries
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    libpcl-dev \
    libeigen3-dev \
    ros-humble-pacmod2 \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-tf2-eigen \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-visualization-msgs \
    ros-humble-diagnostic-updater \
    ros-humble-can-msgs \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-spatio-temporal-voxel-layer \
    && rm -rf /var/lib/apt/lists/*

# Install Livox-SDK2 from source
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2 && \
    cd /tmp/Livox-SDK2 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/Livox-SDK2

# Set up the build workspace
WORKDIR /workspace/ros_ws

# Copy workspace source code from the host
COPY ros_ws/src /workspace/ros_ws/src

# Install workspace dependencies using rosdep
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro humble && \
    rm -rf /var/lib/apt/lists/*

# Compile the ROS 2 Humble workspace (standard build, no symlinks for clean copying)
# NOTE: We pass -DROS_EDITION=ROS2 -DDISTRO_ROS=humble so livox_ros_driver2 compiles for Humble
RUN source /opt/ros/humble/setup.bash && \
    colcon build --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DCMAKE_BUILD_TYPE=Release


# ==========================================
# Stage 2: Runtime Stage
# ==========================================
FROM ros:humble-ros-base AS runtime

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Add AutonomouStuff public APT repository for pacmod2 binary dependency
RUN echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ jammy main" > /etc/apt/sources.list.d/autonomoustuff-public.list

# Install only runtime ROS 2 dependencies, pacmod2 driver, Navigation2, and network/CAN utilities
RUN apt-get update && apt-get install -y \
    iproute2 \
    can-utils \
    ros-humble-pacmod2 \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-tf2-eigen \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-visualization-msgs \
    ros-humble-diagnostic-updater \
    ros-humble-can-msgs \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-spatio-temporal-voxel-layer \
    ros-humble-rviz2 \
    ros-humble-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

# Copy built Livox-SDK2 shared libraries and headers from builder
COPY --from=builder /usr/local/lib /usr/local/lib
COPY --from=builder /usr/local/include /usr/local/include
RUN ldconfig

# Set up runtime workspace directory
WORKDIR /workspace

# Copy built workspace install overlay directly from the builder stage
COPY --from=builder /workspace/ros_ws/install /workspace/ros_ws/install

# Create placeholder directories for fast_lio debug logs and PCD files to prevent warnings/crashes
RUN mkdir -p /workspace/ros_ws/src/fast_lio/Log /workspace/ros_ws/src/fast_lio/PCD

# Configure CycloneDDS
COPY configs/cyclonedds.xml /etc/cyclonedds.xml
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml

# Copy entrypoint script
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]