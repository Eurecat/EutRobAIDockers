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
    --platform ARCH     Target platform architecture: 'amd' (default) or 'arm'
                        amd: Standard x86_64 build using Dockerfile
                        arm: Jetson ARM64 build using Dockerfile.arm
    
    --humble            Build for ROS2 Humble (Ubuntu 22.04, Python 3.10)
                        Default: ROS2 Jazzy (Ubuntu 24.04, Python 3.12)
                        Not supported when --platform arm is used
    
    --vulcanexus        Use Vulcanexus base image instead of standard ROS2
                        Default: Vulcanexus Jazzy; with --humble uses Vulcanexus Humble
                        Not supported when --platform arm is used
    
    --cpu               Build CPU-only version (no GPU support)
                        Default: GPU-enabled build with CUDA support
                        Not supported when --platform arm is used
    
    --clean-rebuild     Force a clean rebuild without using Docker cache
    
    --help, -h          Display this help message

EXAMPLES:
    # Standard ROS2 Jazzy with GPU support (default, x86_64)
    ./build_container.sh

    # Jetson Thor / ARM64 ROS2 Jazzy build
    ./build_container.sh --platform arm

    # ROS2 Humble with GPU support on x86_64
    ./build_container.sh --humble --platform amd

    # ROS2 Jazzy CPU-only
    ./build_container.sh --cpu

    # Vulcanexus Jazzy with GPU support
    ./build_container.sh --vulcanexus

    # Vulcanexus Humble with GPU support
    ./build_container.sh --humble --vulcanexus

    # ROS2 Humble CPU-only with clean rebuild
    ./build_container.sh --humble --cpu --clean-rebuild

GENERATED IMAGES:
    Jazzy GPU (amd64):       eut_ros_torch:jazzy
    Jazzy CPU (amd64):       eut_ros_torch_cpu:jazzy
    Jazzy GPU (arm64):       eut_ros_torch:jazzy (using Dockerfile.arm + NVIDIA Jetson PyTorch base)
    Humble GPU (amd64):      eut_ros_torch:humble
    Humble CPU (amd64):      eut_ros_torch_cpu:humble
    Vulcanexus Jazzy GPU:    eut_ros_vulcanexus_torch:jazzy
    Vulcanexus Jazzy CPU:    eut_ros_vulcanexus_torch_cpu:jazzy
    Vulcanexus Humble GPU: eut_ros_vulcanexus_torch:humble
    Vulcanexus Humble CPU: eut_ros_vulcanexus_torch_cpu:humble

ENVIRONMENT:
    The script creates/updates a .env file with:
    - USERNAME, USER_ID, GROUP_ID
    - ROS_DOMAIN_ID
    - TARGET_DISTRO, BUILT_IMAGE
    - DOCKER_RUNTIME (nvidia for GPU, runc for CPU)

ARM OVERRIDES:
    When --platform arm is selected:
    - Dockerfile.arm is used
    - ROS 2 Jazzy is forced
    - The build starts from JETSON_BASE_IMAGE
    - Default JETSON_BASE_IMAGE: nvcr.io/nvidia/pytorch:25.08-py3-igpu

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
DOCKERFILE="Dockerfile"
TARGET_DISTRO="jazzy"
REBUILD=false
USE_VULCANEXUS=false
USE_HUMBLE=false
CPU_ONLY="false"
PLATFORM_ARCH="amd"
JETSON_BASE_IMAGE="${JETSON_BASE_IMAGE:-nvcr.io/nvidia/pytorch:25.08-py3}"

# Parse command line arguments
while [ "$#" -gt 0 ]; do
    case "$1" in
        --clean-rebuild)
            REBUILD=true
            ;;
        --vulcanexus)
            USE_VULCANEXUS=true
            ;;
        --humble)
            USE_HUMBLE=true
            TARGET_DISTRO="humble"
            ;;
        --cpu)
            CPU_ONLY="true"
            ;;
        --platform)
            shift
            if [ -z "$1" ]; then
                echo "Error: --platform requires a value: amd or arm"
                exit 1
            fi
            PLATFORM_ARCH="$1"
            ;;
        --platform=*)
            PLATFORM_ARCH="${1#*=}"
            ;;
        *)
            echo "Error: Unknown option '$1'"
            echo "Use --help to see available options."
            exit 1
            ;;
    esac
    shift
done

if [ "$PLATFORM_ARCH" != "amd" ] && [ "$PLATFORM_ARCH" != "arm" ]; then
    echo "Error: Invalid platform '$PLATFORM_ARCH'. Expected 'amd' or 'arm'."
    exit 1
fi

if [ "$PLATFORM_ARCH" = "arm" ]; then
    DOCKERFILE="Dockerfile.arm"
    BASE_IMAGE="$JETSON_BASE_IMAGE"
    TARGET_DISTRO="jazzy"

    if $USE_VULCANEXUS; then
        echo "Error: --vulcanexus is not supported with --platform arm. Dockerfile.arm installs standard ROS 2 Jazzy on top of a Jetson-compatible NVIDIA PyTorch base."
        exit 1
    fi

    if [ "$CPU_ONLY" = "true" ]; then
        echo "Error: --cpu is not supported with --platform arm. Dockerfile.arm requires the Jetson GPU-enabled base image."
        exit 1
    fi

    if $USE_HUMBLE; then
        echo "Warning: --humble is ignored for --platform arm. Forcing ROS 2 Jazzy in Dockerfile.arm."
    fi

    USE_HUMBLE=false
else
    if $USE_VULCANEXUS; then
        if $USE_HUMBLE; then
            BASE_IMAGE="eprosima/vulcanexus:humble-desktop"
        else
            BASE_IMAGE="eprosima/vulcanexus:jazzy-desktop"
        fi
    else
        if $USE_HUMBLE; then
            BASE_IMAGE="osrf/ros:humble-desktop-full"
        else
            BASE_IMAGE="osrf/ros:jazzy-desktop-full"
        fi
    fi
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

# Set Docker platform based on architecture
if [ "$PLATFORM_ARCH" = "arm" ]; then
    DOCKER_PLATFORM="linux/arm64"
else
    DOCKER_PLATFORM="linux/amd64"
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
echo "Dockerfile: ${DOCKERFILE}"
echo "Platform: ${PLATFORM_ARCH} (${DOCKER_PLATFORM})"
echo "CPU Only: ${CPU_ONLY}"
echo "Output image: ${IMAGE_NAME}"
echo "Python version: ${PYTHON_VERSION}"

if $REBUILD; then
    echo "Rebuilding the Docker image..."
    docker build --platform ${DOCKER_PLATFORM} --no-cache . --build-arg BASE_IMAGE="${BASE_IMAGE}" --build-arg PYTHON_VERSION="${PYTHON_VERSION}" --build-arg CPU_ONLY="${CPU_ONLY}" --build-arg PLATFORM_ARCH="${PLATFORM_ARCH}" -t ${IMAGE_NAME} -f ${DOCKERFILE}
else
    docker build --platform ${DOCKER_PLATFORM} . --build-arg BASE_IMAGE="${BASE_IMAGE}" --build-arg PYTHON_VERSION="${PYTHON_VERSION}" --build-arg CPU_ONLY="${CPU_ONLY}" --build-arg PLATFORM_ARCH="${PLATFORM_ARCH}" -t ${IMAGE_NAME} -f ${DOCKERFILE}
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

# Set or Update ROS_WS based on platform
if [ "$PLATFORM_ARCH" = "arm" ]; then
    ROS_WS="/ros2_ws"
else
    ROS_WS="/workspace"
fi

if grep -q -E "^ROS_WS=" "$ENV_FILE"; then
    sed -i "s|^ROS_WS=.*|ROS_WS=$ROS_WS|" "$ENV_FILE"
else
    echo "ROS_WS=$ROS_WS" >> "$ENV_FILE"
fi

echo "Docker image $IMAGE_NAME built successfully!"
echo "Build process completed!"


