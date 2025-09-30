# EutRobAI Base Image: Configurable ROS2 + PyTorch
# 
# This Dockerfile creates a base image with:
# - Configurable ROS 2 base (standard Jazzy or Vulcanexus Jazzy)
# - PyTorch for deep learning capabilities
# - Development tools and utilities for robotics AI projects
#
# Usage: 
# - Standard ROS2 Jazzy: ./build.sh
# - Vulcanexus Jazzy: ./build.sh --vulcanexus
# - Clean rebuild: ./build.sh --clean-rebuild [--vulcanexus]

# Choose the base image at build time. Defaults to the second option.
ARG BASE_IMAGE="osrf/ros:jazzy-desktop-full"

# You can refer to ARGs in FROM (supported by modern Docker)
FROM ${BASE_IMAGE} AS base

# (Optional) re-declare the arg if you want to use it in later instructions
ARG BASE_IMAGE

# Set default user/group variables
# ARG USER_NAME=coghri
# ARG GROUP_NAME=coghri-devs
# ARG USER_ID=1001
# ARG GROUP_ID=1001

ENV DEBIAN_FRONTEND=noninteractive

# ####################################################################################################
# # Install Python dependencies including PyTorch
# ####################################################################################################

# Install pip, uv and Python development packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Install uv (fast Python package installer) via pip with system packages override
RUN pip3 install --break-system-packages uv

# Install PyTorch and torchvision using uv with system packages override
RUN uv pip install --system --no-cache --break-system-packages \
    "torch>=1.7.0" \
    "torchvision>=0.8.1"

# ####################################################################################################
# # Binaries (ros2 utilities, etc.)
# ####################################################################################################

# ####################################################################################################
# # Binaries (general, utilities, BT/groot/rosxintrospection/... deps etc.)
# ####################################################################################################


# ####################################################################################################
# # Binaries for ....
# ####################################################################################################

# ####################################################################################################
# # Clone private repositories that do not need to be remapped as volumes
# ####################################################################################################
RUN mkdir -p /workspace

RUN echo "Setting up SSH for git access"
RUN mkdir -p ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

# Set working directory
WORKDIR /workspace/src

#Clone repos
# RUN --mount=type=ssh git clone ....

# ####################################################################################################
# # Prepare local ros2 workspace (build deps and compile as root)
# ####################################################################################################

WORKDIR /workspace

# Optional: fix rosdep init issue gracefully
RUN [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ] /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash;sudo rosdep init" || echo "rosdep already initialized"

RUN apt-get update && \ 
    /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash;\
      rosdep update; \
      rosdep install -y -r -q --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}; "

# ####################################################################################################
# Moving src in the workspace and compiling...
# ####################################################################################################
# '
# Copy all BTs-related folders (for cache efficiency in building, maybe we want to move the COPY operations)
# COPY deps/...

WORKDIR /workspace
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; colcon build --event-handlers console_direct+ --symlink-install;"

# ####################################################################################################
# Setting up local user space (create user, set permissions, etc.) during image build
# ####################################################################################################

# # Install gosu for privilege dropping
# RUN apt-get update && apt-get install -y gosu

# ####################################################################################################
# User created during build - switch to non-root user
# ####################################################################################################

# Copy and run original entrypoint script during build
# COPY entrypoint.sh /workspace/entrypoint.sh
# RUN chmod +x /workspace/entrypoint.sh
# ENTRYPOINT ["/workspace/entrypoint.sh"]

# # USER coghri
# CMD ["/bin/bash"]
