#!/usr/bin/env bash
#
# EutRobAI Docker Build Script
# Build configurable ROS2 Docker images with PyTorch support

# Function to display help
show_help() {
    cat << EOF
Usage: ./build_container.sh [OPTIONS]

Build EutRobAI Docker images with configurable ROS2 distributions and hardware support.

OPTIONS:
    --humble            Build for ROS2 Humble (Ubuntu 22.04, Python 3.10)
                        Default: ROS2 Jazzy (Ubuntu 24.04, Python 3.12)
    
    --vulcanexus        Use Vulcanexus Jazzy base image instead of standard ROS2
                        Note: Only compatible with Jazzy (cannot be used with --humble)
    
    --cpu               Build CPU-only version (no GPU support)
                        Default: GPU-enabled build with CUDA support
    
    --clean-rebuild     Force a clean rebuild without using Docker cache
    
    --help, -h          Display this help message

EXAMPLES:
    # Standard ROS2 Jazzy with GPU support (default)
    ./build_container.sh

    # ROS2 Humble with GPU support
    ./build_container.sh --humble

    # ROS2 Jazzy CPU-only
    ./build_container.sh --cpu

    # Vulcanexus Jazzy with GPU support
    ./build_container.sh --vulcanexus

    # ROS2 Humble CPU-only with clean rebuild
    ./build_container.sh --humble --cpu --clean-rebuild

GENERATED IMAGES:
    Jazzy GPU:           eut_ros_torch:jazzy
    Jazzy CPU:           eut_ros_torch_cpu:jazzy
    Humble GPU:          eut_ros_torch:humble
    Humble CPU:          eut_ros_torch_cpu:humble
    Vulcanexus Jazzy GPU: eut_ros_vulcanexus_torch:jazzy
    Vulcanexus Jazzy CPU: eut_ros_vulcanexus_torch_cpu:jazzy

ENVIRONMENT:
    The script creates/updates a .env file with:
    - USERNAME, USER_ID, GROUP_ID
    - ROS_DOMAIN_ID
    - TARGET_DISTRO, BUILT_IMAGE
    - DOCKER_RUNTIME (nvidia for GPU, runc for CPU)

EOF
    exit 0
}

# Check for help flag first
for arg in "$@"; do
    if [ "$arg" == "--help" ] || [ "$arg" == "-h" ]; then
        show_help
    fi
done

export DOCKER_BUILDKIT=1

set -e

# --- BEGIN: Manage .env file ---
ENV_FILE="./.env" # Assuming .env is in the same directory as this script
# Create .env if it doesn't exist
if [ ! -f "$ENV_FILE" ]; then
    echo "Creating $ENV_FILE..."
    touch "$ENV_FILE"
fi

DEFAULT_USERNAME="coghri"
DEFAULT_ROS_DOMAIN_ID="12"
CURRENT_USER_ID=$(id -u)
CURRENT_GROUP_ID=$(id -g)

# Set USERNAME if not present, otherwise leave existing value
if ! grep -q -E "^USERNAME=" "$ENV_FILE"; then
    echo "USERNAME=$DEFAULT_USERNAME" >> "$ENV_FILE"
fi

# Set ROS_DOMAIN_ID if not present, otherwise leave existing value
if ! grep -q -E "^ROS_DOMAIN_ID=" "$ENV_FILE"; then
    echo "ROS_DOMAIN_ID=$DEFAULT_ROS_DOMAIN_ID" >> "$ENV_FILE"
fi

# Set or Update USER_ID with current host user ID
if grep -q -E "^USER_ID=" "$ENV_FILE"; then
    sed -i "s/^USER_ID=.*/USER_ID=$CURRENT_USER_ID/" "$ENV_FILE"
else
    echo "USER_ID=$CURRENT_USER_ID" >> "$ENV_FILE"
fi

# Set or Update GROUP_ID with current host group ID
if grep -q -E "^GROUP_ID=" "$ENV_FILE"; then
    sed -i "s/^GROUP_ID=.*/GROUP_ID=$CURRENT_GROUP_ID/" "$ENV_FILE"
else
    echo "GROUP_ID=$CURRENT_GROUP_ID" >> "$ENV_FILE"
fi
# --- END: Manage .env file ---

# Initialize flags and variables
BASE_IMAGE="osrf/ros:jazzy-desktop-full"
TARGET_DISTRO="jazzy"
REBUILD=false
USE_VULCANEXUS=false
USE_HUMBLE=false
CPU_ONLY="false"

# Parse command line arguments
for arg in "$@"; do
    if [ "$arg" == "--clean-rebuild" ]; then
        REBUILD=true
    fi
    if [ "$arg" == "--vulcanexus" ]; then
        USE_VULCANEXUS=true
        BASE_IMAGE="eprosima/vulcanexus:jazzy-desktop"
    fi
    if [ "$arg" == "--humble" ]; then
        USE_HUMBLE=true
        BASE_IMAGE="osrf/ros:humble-desktop-full"
        TARGET_DISTRO="humble"
    fi
    if [ "$arg" == "--cpu" ]; then
        CPU_ONLY="true"
    fi
done

# Validate that Vulcanexus and Humble are not used together
if $USE_VULCANEXUS && $USE_HUMBLE; then
    echo "ERROR: --vulcanexus and --humble cannot be used together."
    echo "Vulcanexus is only available for Jazzy."
    exit 1
fi

if $REBUILD; then # remove all files just in case some modifications have been made and git pull does not work
    echo "Rebuilding: cleaning up dependencies..."
fi

# Set Python version based on ROS distribution
if [ "${TARGET_DISTRO}" == "humble" ]; then
    PYTHON_VERSION="3.10"
else
    PYTHON_VERSION="3.12"
fi

# Set image name based on the base image choice and CPU flag
if [[ "${BASE_IMAGE}" == *"vulcanexus"* ]]; then
    if [ "$CPU_ONLY" = "true" ]; then
        IMAGE_NAME="eut_ros_vulcanexus_torch_cpu:${TARGET_DISTRO}"
        echo "Building with Vulcanexus ${TARGET_DISTRO} CPU-only base image..."
    else
        IMAGE_NAME="eut_ros_vulcanexus_torch:${TARGET_DISTRO}"
        echo "Building with Vulcanexus ${TARGET_DISTRO} base image..."
    fi
else
    if [ "$CPU_ONLY" = "true" ]; then
        IMAGE_NAME="eut_ros_torch_cpu:${TARGET_DISTRO}"
        echo "Building with standard ROS2 ${TARGET_DISTRO} CPU-only base image..."
    else
        IMAGE_NAME="eut_ros_torch:${TARGET_DISTRO}"
        echo "Building with standard ROS2 ${TARGET_DISTRO} base image..."
    fi
fi

echo "Base image: ${BASE_IMAGE}"
echo "CPU Only: ${CPU_ONLY}"
echo "Output image: ${IMAGE_NAME}"
echo "Python version: ${PYTHON_VERSION}"

if $REBUILD; then
    echo "Rebuilding the Docker image..."
    docker build --no-cache . --build-arg BASE_IMAGE="${BASE_IMAGE}" --build-arg PYTHON_VERSION="${PYTHON_VERSION}" --build-arg CPU_ONLY="${CPU_ONLY}" -t ${IMAGE_NAME} -f Dockerfile
else
    docker build . --build-arg BASE_IMAGE="${BASE_IMAGE}" --build-arg PYTHON_VERSION="${PYTHON_VERSION}" --build-arg CPU_ONLY="${CPU_ONLY}" -t ${IMAGE_NAME} -f Dockerfile
fi

# Set or Update TARGET_DISTRO 
if grep -q -E "^TARGET_DISTRO=" "$ENV_FILE"; then
    sed -i "s/^TARGET_DISTRO=.*/TARGET_DISTRO=$TARGET_DISTRO/" "$ENV_FILE"
else
    echo "TARGET_DISTRO=$TARGET_DISTRO" >> "$ENV_FILE"
fi

# Set or Update BUILT_IMAGE 
if grep -q -E "^BUILT_IMAGE=" "$ENV_FILE"; then
    sed -i "s/^BUILT_IMAGE=.*/BUILT_IMAGE=$IMAGE_NAME/" "$ENV_FILE"
else
    echo "BUILT_IMAGE=$IMAGE_NAME" >> "$ENV_FILE"
fi

# Set or Update DOCKER_RUNTIME based on CPU_ONLY flag
if [ "$CPU_ONLY" = "true" ]; then
    DOCKER_RUNTIME="runc"
else
    DOCKER_RUNTIME="nvidia"
fi

if grep -q -E "^DOCKER_RUNTIME=" "$ENV_FILE"; then
    sed -i "s/^DOCKER_RUNTIME=.*/DOCKER_RUNTIME=$DOCKER_RUNTIME/" "$ENV_FILE"
else
    echo "DOCKER_RUNTIME=$DOCKER_RUNTIME" >> "$ENV_FILE"
fi

echo "Docker image $IMAGE_NAME built successfully!"
echo "Build process completed!"


