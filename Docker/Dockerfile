# EutRobAI Base Image: Configurable ROS2 + PyTorch
# 
# This Dockerfile creates a base image with:
# - Configurable ROS 2 base (standard Jazzy or Vulcanexus Jazzy)
# - PyTorch for deep learning capabilities
# - Development tools and utilities for robotics AI projects

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
    python3-venv \
    build-essential \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    git \
    terminator \
    && rm -rf /var/lib/apt/lists/*

# Install uv (fast Python package installer) via pip with system packages override
RUN pip3 install --break-system-packages uv

# # Install PyTorch and torchvision using uv with system packages override
# Create a virtual environment for Python packages
RUN python3 -m venv /opt/ros_python_env

# Install requirements in the virtual environment
COPY requirements.txt /tmp/requirements.txt
RUN /bin/bash -c "source /opt/ros_python_env/bin/activate && uv pip install --no-cache -r /tmp/requirements.txt"

# ####################################################################################################
# # Binaries for .... (ros2 utilities, etc.)
# ####################################################################################################

# ####################################################################################################
# # Prepare workspace: clone private repositories that do not need to be COPIED from deps nor remapped as volumes
# ####################################################################################################
RUN mkdir -p /workspace/src

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
# Copy all src deps
# COPY deps/...

WORKDIR /workspace
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; colcon build --event-handlers console_direct+ --symlink-install;"

# ####################################################################################################
# [NOT DONE atm] Setting up local user space (create user, set permissions, etc.) during image build
# ####################################################################################################

# ...

# # Install gosu for privilege dropping
# RUN apt-get update && apt-get install -y gosu

# ####################################################################################################
# [NOT DONE atm] User created during build - switch to non-root user
# ####################################################################################################

# Copy and set up the entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# # USER coghri
# CMD ["/bin/bash"]
