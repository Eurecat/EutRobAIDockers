#!/bin/bash
set -e

echo "=== ENTRYPOINT START $(date) PID=$$ ==="
# Source ROS 2 environment
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    echo "Sourcing ROS 2 environment..."
    source /opt/ros/${ROS_DISTRO}/setup.bash
    echo "Sourced ${ROS_DISTRO}"
fi
if [ -f "/opt/vulcanexus/${ROS_DISTRO}/setup.bash" ]; then
    echo "Sourcing ROS 2 Vulcanexus environment..."
    source /opt/vulcanexus/${ROS_DISTRO}/setup.bash
    echo "Sourced ${ROS_DISTRO}"
fi

# Use configurable workspace root (ARM image uses /ros2_ws, x86 keeps /workspace)
ROS_WS="${ROS_WS:-/workspace}"

# Build person detection and skeleton detection packages
echo "Building ros2 packages in ${ROS_WS}..."
cd "${ROS_WS}"
colcon build --symlink-install --event-handlers console_direct+
    
# Source the updated workspace after building
if [ -f "${ROS_WS}/install/setup.bash" ]; then
    echo "Sourcing updated workspace environment..."
    source "${ROS_WS}/install/setup.bash"
fi

source /opt/ros_python_env/bin/activate
echo "=== ENTRYPOINT END $(date) PID=$$ ==="

# Execute the command passed to the container
exec "$@"
