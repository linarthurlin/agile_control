#!/bin/bash
set -e

# Source ROS 2 Humble base setup
source /opt/ros/humble/setup.bash

# Source the unified workspace build setup if it exists
if [ -f "/workspace/ros_ws/install/setup.bash" ]; then
    source /workspace/ros_ws/install/setup.bash
fi

# Set DDS environment variables for optimized CycloneDDS configuration
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///etc/cyclonedds.xml

# Execute the command passed to the container
exec "$@"
