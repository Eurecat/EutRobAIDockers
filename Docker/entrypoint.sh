#!/bin/bash
set -e

echo "=== ENTRYPOINT START $(date) PID=$$ ==="
# Source ROS 2 environment
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    echo "Sourcing ROS 2 environment..."
    source /opt/ros/jazzy/setup.bash
    echo "Sourced ${ROS_DISTRO}"
fi
if [ -f "/opt/vulcanexus/jazzy/setup.bash" ]; then
    echo "Sourcing ROS 2 Vulcanexus environment..."
    source /opt/vulcanexus/jazzy/setup.bash
    echo "Sourced ${ROS_DISTRO}"
fi

# Build person detection and skeleton detection packages
echo "Building ros2 packages of this repo..."
cd /workspace
colcon build --symlink-install --event-handlers console_direct+ 

# Source the updated workspace after building
if [ -f "/workspace/install/setup.bash" ]; then
    echo "Sourcing updated workspace environment..."
    source /workspace/install/setup.bash
fi

source /opt/ros_python_env/bin/activate
echo "Python version: $(python3 --version)"
echo "Torch version: $(python3 -c 'import torch; print(torch.__version__)')"
echo "=== ENTRYPOINT END $(date) PID=$$ ==="

# Execute the command passed to the container
exec "$@"
