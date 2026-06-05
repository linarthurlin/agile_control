#!/usr/bin/env bash
set -e

IMAGE_NAME="agile_control_ros2"
COMPOSE_FILE="compose.yaml"

echo "==============================="
echo " ROS2 Agile Control Setup"
echo "==============================="

# -----------------------------
# 0. Initialize Host Workspace
# -----------------------------
init_host_workspace() {
    echo "[0/3] Initializing host workspace..."
    mkdir -p ros_ws/src
    
    if [ ! -d "ros_ws/src/fast_lio" ]; then
        echo "Cloning FAST_LIO_ROS2..."
        git clone --recursive https://github.com/Ericsii/FAST_LIO_ROS2.git ros_ws/src/fast_lio
    fi
    
    if [ ! -d "ros_ws/src/livox_ros_driver2" ]; then
        echo "Cloning livox_ros_driver2..."
        git clone https://github.com/Livox-SDK/livox_ros_driver2.git ros_ws/src/livox_ros_driver2
        cp ros_ws/src/livox_ros_driver2/package_ROS2.xml ros_ws/src/livox_ros_driver2/package.xml
    fi
    
    if [ ! -d "ros_ws/src/pacmod2_msgs" ]; then
        echo "Cloning pacmod2_msgs..."
        git clone https://github.com/astuff/pacmod2_msgs.git ros_ws/src/pacmod2_msgs
    fi
    
    if [ ! -d "ros_ws/src/Nav2" ]; then
        echo "Integrating Nav2 custom package from agile_planning-nav2-stvl..."
        mkdir -p ros_ws/src/Nav2/launch ros_ws/src/Nav2/config ros_ws/src/Nav2/src
        if [ -d "agile_planning-nav2-stvl/agile_planning-nav2-stvl/ros2_ws/nav2" ]; then
            cp -r agile_planning-nav2-stvl/agile_planning-nav2-stvl/ros2_ws/nav2/*.py ros_ws/src/Nav2/src/ 2>/dev/null || true
            cp -r agile_planning-nav2-stvl/agile_planning-nav2-stvl/ros2_ws/nav2/waypoints.txt ros_ws/src/Nav2/src/ 2>/dev/null || true
            cp -r agile_planning-nav2-stvl/agile_planning-nav2-stvl/ros2_ws/nav2/agile_nav2_launch.launch.py ros_ws/src/Nav2/launch/ 2>/dev/null || true
            cp -r agile_planning-nav2-stvl/agile_planning-nav2-stvl/ros2_ws/nav2/nav2_params.yaml ros_ws/src/Nav2/config/ 2>/dev/null || true
            cp -r agile_planning-nav2-stvl/agile_planning-nav2-stvl/ros2_ws/nav2/archive ros_ws/src/Nav2/ 2>/dev/null || true
        fi
    fi
}

# -----------------------------
# 1. Build Docker Image
# -----------------------------
build_image() {
    init_host_workspace
    echo "[1/3] Building Docker image: $IMAGE_NAME"
    docker build -t $IMAGE_NAME -f docker/Dockerfile .
}

# -----------------------------
# 2. Clean old containers (optional safe)
# -----------------------------
clean_containers() {
    echo "[2/3] Cleaning stopped containers..."
    docker container prune -f
}

# -----------------------------
# 3. Start system
# -----------------------------
start_system() {
    # Ensure multicast is enabled on loopback for CycloneDDS
    if ! ip link show lo | head -n 1 | grep -q "MULTICAST"; then
        echo "Enabling multicast on loopback interface..."
        sudo -n ip link set lo multicast on || true
    fi
    echo "[3/3] Starting ROS2 system..."
    docker compose -f $COMPOSE_FILE up
}

# -----------------------------
# Full pipeline
# -----------------------------
full_setup() {
    build_image
    clean_containers
    start_system
}

# -----------------------------
# CLI options
# -----------------------------
case "$1" in
    build)
        build_image
        ;;
    clean)
        clean_containers
        ;;
    up)
        start_system
        ;;
    all|"")
        full_setup
        ;;
    *)
        echo "Usage: ./setup.sh [build|clean|up|all]"
        exit 1
        ;;
esac