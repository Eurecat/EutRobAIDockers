#!/usr/bin/env bash
#
# Usage: 
# - Standard ROS2 Jazzy: ./build_container.sh
# - Vulcanexus Jazzy: ./build_container.sh --vulcanexus
# - CPU-only version: ./build_container.sh --cpu
# - CPU-only Vulcanexus: ./build_container.sh --cpu --vulcanexus
# - Clean rebuild: ./build_container.sh --clean-rebuild [--vulcanexus] [--cpu]

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

# Check if --clean-rebuild is among the arguments
BASE_IMAGE="osrf/ros:jazzy-desktop-full"
REBUILD=false
CPU_ONLY="false"
for arg in "$@"; do
    if [ "$arg" == "--clean-rebuild" ]; then
        REBUILD=true
    fi
    if [ "$arg" == "--vulcanexus" ]; then
        BASE_IMAGE="eprosima/vulcanexus:jazzy-desktop"
    fi
    if [ "$arg" == "--cpu" ]; then
        CPU_ONLY="true"
    fi
done

if $REBUILD; then # remove all files just in case some modifications have been made and git pull does not work
    echo "Rebuilding: cleaning up dependencies..."
fi

# Set image name based on the base image choice and CPU flag
TARGET_DISTRO="jazzy"
if [[ "${BASE_IMAGE}" == *"vulcanexus"* ]]; then
    if [ "$CPU_ONLY" = "true" ]; then
        IMAGE_NAME="eut_ros_vulcanexus_torch_cpu:${TARGET_DISTRO}"
        echo "Building with Vulcanexus Jazzy CPU-only base image..."
    else
        IMAGE_NAME="eut_ros_vulcanexus_torch:${TARGET_DISTRO}"
        echo "Building with Vulcanexus Jazzy base image..."
    fi
else
    if [ "$CPU_ONLY" = "true" ]; then
        IMAGE_NAME="eut_ros_torch_cpu:${TARGET_DISTRO}"
        echo "Building with standard ROS2 Jazzy CPU-only base image..."
    else
        IMAGE_NAME="eut_ros_torch:${TARGET_DISTRO}"
        echo "Building with standard ROS2 Jazzy base image..."
    fi
fi

echo "Base image: ${BASE_IMAGE}"
echo "CPU Only: ${CPU_ONLY}"
echo "Output image: ${IMAGE_NAME}"

if $REBUILD; then
    echo "Rebuilding the Docker image..."
    docker build --no-cache . --build-arg BASE_IMAGE="${BASE_IMAGE}" --build-arg CPU_ONLY="${CPU_ONLY}" -t ${IMAGE_NAME} -f Dockerfile
else
    docker build . --build-arg BASE_IMAGE="${BASE_IMAGE}" --build-arg CPU_ONLY="${CPU_ONLY}" -t ${IMAGE_NAME} -f Dockerfile
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

echo "Docker image $IMAGE_NAME built successfully!"
echo "Build process completed!"


