#!/usr/bin/env bash
#
# Usage: 
# - Standard ROS2 Jazzy: ./build_container.sh
# - Vulcanexus Jazzy: ./build_container.sh --vulcanexus
# - Clean rebuild: ./build_container.sh --clean-rebuild [--vulcanexus]

export DOCKER_BUILDKIT=1

set -e

# --- BEGIN: Manage .env file ---
ENV_FILE="./.env" # Assuming .env is in the same directory as this script
# Create .env if it doesn't exist
if [ ! -f "$ENV_FILE" ]; then
    echo "Creating $ENV_FILE..."
    touch "$ENV_FILE"
fi

# Check if --clean-rebuild is among the arguments
BASE_IMAGE="osrf/ros:jazzy-desktop-full"
REBUILD=false
for arg in "$@"; do
    if [ "$arg" == "--clean-rebuild" ]; then
        REBUILD=true
    fi
    if [ "$arg" == "--vulcanexus" ]; then
        BASE_IMAGE="eprosima/vulcanexus:jazzy-desktop"
    fi
done

if $REBUILD; then # remove all files just in case some modifications have been made and git pull does not work
    echo "Rebuilding: cleaning up dependencies..."
fi

# Set image name based on the base image choice
if [[ "${BASE_IMAGE}" == *"vulcanexus"* ]]; then
    IMAGE_NAME="eut_ros_vulcanexus_torch:jazzy"
    echo "Building with Vulcanexus Jazzy base image..."
else
    IMAGE_NAME="eut_ros_torch:jazzy"
    echo "Building with standard ROS2 Jazzy base image..."
fi

echo "Base image: ${BASE_IMAGE}"
echo "Output image: ${IMAGE_NAME}"

if $REBUILD; then
    echo "Rebuilding the Docker image..."
    docker build --ssh default --no-cache . --build-arg BASE_IMAGE="${BASE_IMAGE}" -t ${IMAGE_NAME} -f Dockerfile
else
    docker build --ssh default . --build-arg BASE_IMAGE="${BASE_IMAGE}" -t ${IMAGE_NAME} -f Dockerfile
fi

# Set or Update BUILT_IMAGE 
if grep -q -E "^BUILT_IMAGE=" "$ENV_FILE"; then
    sed -i "s/^BUILT_IMAGE=.*/BUILT_IMAGE=$IMAGE_NAME/" "$ENV_FILE"
else
    echo "BUILT_IMAGE=$IMAGE_NAME" >> "$ENV_FILE"
fi

echo "Docker image $IMAGE_NAME built successfully!"
echo "Build process completed!"


