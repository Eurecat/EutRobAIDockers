# EutRobAI Base Image: Configurable ROS2 + PyTorch for x86_64
# 
# This Dockerfile creates a base image with:
# - Configurable ROS 2 base (standard Jazzy or Vulcanexus Jazzy)
# - PyTorch for deep learning capabilities (GPU or CPU)
# - Development tools and utilities for robotics AI projects

# Choose the base image at build time. Defaults to Jazzy.
ARG BASE_IMAGE="osrf/ros:jazzy-desktop-full"
ARG CPU_ONLY="false"
ARG PLATFORM_ARCH="amd"

FROM ${BASE_IMAGE} AS base

# (Optional) re-declare the args if you want to use them in later instructions
ARG BASE_IMAGE
ARG CPU_ONLY
ARG PLATFORM_ARCH

# Python version to use (3.10 for Humble/Ubuntu 22.04, 3.12 for Jazzy/Ubuntu 24.04)
ARG PYTHON_VERSION="3.12"

# Set default user/group variables
# ARG USER_NAME=coghri
# ARG GROUP_NAME=coghri-devs
# ARG USER_ID=1001
# ARG GROUP_ID=1001

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS=/workspace

# ####################################################################################################
# # Install Python dependencies including PyTorch
# ####################################################################################################

# Install pip, uv and Python development packages
RUN apt-get update && apt-get install -y \
    lcov \
    python3-pip \
    python3-dev \
    python3-venv \
    python3-pytest python3-pytest-cov \
    build-essential \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    git \
    wget \
    terminator \
    gosu \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip to ensure --break-system-packages flag is supported (only needed for Humble)
ARG PYTHON_VERSION
RUN if [ "${PYTHON_VERSION}" = "3.10" ]; then \
        pip3 install --upgrade pip; \
    fi

# Install uv (fast Python package installer) via pip with system packages override
RUN pip3 install --break-system-packages uv

# # Install PyTorch and torchvision using uv with system packages override
# Create a virtual environment for Python packages
RUN uv venv /opt/ros_python_env --python ${PYTHON_VERSION}

# Install requirements in the virtual environment
COPY requirements.txt /tmp/requirements.txt

# Use CPU or GPU requirements based on build arg.
RUN if [ "$CPU_ONLY" = "true" ]; then \
        /bin/bash -c "source /opt/ros_python_env/bin/activate && uv pip install --no-cache -r /tmp/requirements.txt \
          --index-url https://download.pytorch.org/whl/cpu \
          --extra-index-url https://pypi.org/simple \
          --index-strategy unsafe-best-match"; \
    else \
        # For x86_64 with CUDA support, use the versions from requirements.txt \
        /bin/bash -c "source /opt/ros_python_env/bin/activate && uv pip install --no-cache -r /tmp/requirements.txt \
          --index-url https://download.pytorch.org/whl/cu121 \
          --extra-index-url https://pypi.org/simple \
          --index-strategy unsafe-best-match"; \
    fi


# ####################################################################################################
# # Binaries for .... (ros2 utilities, etc.)
# ####################################################################################################

# ####################################################################################################
# # Prepare workspace: clone private repositories that do not need to be COPIED from deps nor remapped as volumes
# ####################################################################################################
RUN mkdir -p ${ROS_WS}/src

# Set working directory
WORKDIR ${ROS_WS}/src

#Clone repos
# RUN --mount=type=ssh git clone ....

# ####################################################################################################
# # Prepare local ros2 workspace (build deps and compile as root)
# ####################################################################################################

WORKDIR ${ROS_WS}

# Optional: fix rosdep init issue gracefully
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
      /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep init"; \
    else \
      echo "rosdep already initialized"; \
    fi

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

WORKDIR ${ROS_WS}
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

# Copy coverage script for automated testing within CI/CD
COPY ci_cd_coverage.sh /ci_cd_coverage.sh
RUN chmod +x /ci_cd_coverage.sh

# Copy and set up the entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["/bin/bash"]
