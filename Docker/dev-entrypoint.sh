#!/bin/bash
set -e

echo "=== ENTRYPOINT START $(date) PID=$$ ==="

# Default USER_ID and GROUP_ID if not set
USER_ID=${USER_ID:-1000}
GROUP_ID=${GROUP_ID:-1000}
USERNAME="coghri" # Default username
WORKSPACE="/workspace"

# Check if user with USER_ID already exists
EXISTING_USERNAME=$(getent passwd "$USER_ID" | cut -d: -f1)

# If the user exists and the username is different, remove the existing user
if [ -n "$EXISTING_USERNAME" ] && [ "$EXISTING_USERNAME" != "$USERNAME" ]; then
  echo "Removing user $EXISTING_USERNAME"
  deluser --remove-home $EXISTING_USERNAME
fi

# Create group if it doesn't exist
if ! getent group "$USERNAME" >/dev/null && ! getent group "$GROUP_ID" >/dev/null; then
    groupadd --gid "$GROUP_ID" "$USERNAME"
elif ! getent group "$USERNAME" >/dev/null && getent group "$GROUP_ID" >/dev/null; then
    EXISTING_GROUP_NAME=$(getent group "$GROUP_ID" | cut -d: -f1)
    echo "Warning: Group with GID $GROUP_ID already exists with name $EXISTING_GROUP_NAME."
fi

# Create user if it doesn't exist
if ! id -u "$USERNAME" >/dev/null 2>&1 && ! getent passwd "$USER_ID" >/dev/null; then
    if ! getent group "$GROUP_ID" >/dev/null; then
        groupadd --gid "$GROUP_ID" "$USERNAME"
    fi
    useradd --shell /bin/bash --uid "$USER_ID" --gid "$GROUP_ID" --create-home "$USERNAME"
elif ! id -u "$USERNAME" >/dev/null 2>&1 && getent passwd "$USER_ID" >/dev/null; then
    EXISTING_USER_NAME_WITH_UID=$(getent passwd "$USER_ID" | cut -d: -f1)
    echo "Error: UID $USER_ID is already in use by user '$EXISTING_USER_NAME_WITH_UID'," \
         "but the desired username '$USERNAME' does not exist." >&2
    exit 1
fi

# Add user to sudoers with NOPASSWD
echo "Adding user $USERNAME to sudoers with NOPASSWD..."
echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" > "/etc/sudoers.d/$USERNAME"
chmod 0440 "/etc/sudoers.d/$USERNAME"

# Change ownership of workspace
if [ -d "${WORKSPACE}" ]; then
    echo "Changing ownership of ${WORKSPACE} to $USERNAME ($USER_ID:$GROUP_ID)..."
    chown -R "$USER_ID:$GROUP_ID" ${WORKSPACE}
fi

# Ensure home directory permissions and setup .bashrc
USER_HOME="/home/$USERNAME"
if [ -d "$USER_HOME" ]; then
    chown -R "$USER_ID:$GROUP_ID" "$USER_HOME"
    BASHRC_FILE="$USER_HOME/.bashrc"
    if [ ! -f "$BASHRC_FILE" ]; then
        touch "$BASHRC_FILE"
        chown "$USER_ID:$GROUP_ID" "$BASHRC_FILE"
    fi

    echo "Updating $BASHRC_FILE for user $USERNAME..."

    # Add ROS 2 environment setup lines if they don't exist
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        LINE_ROS_SETUP="source /opt/ros/jazzy/setup.bash"
        grep -qxF "$LINE_ROS_SETUP" "$BASHRC_FILE" || echo "$LINE_ROS_SETUP" >> "$BASHRC_FILE"
    fi

    if [ -f "/opt/vulcanexus/jazzy/setup.bash" ]; then
        LINE_VULCANEXUS_SETUP="source /opt/vulcanexus/jazzy/setup.bash"
        grep -qxF "$LINE_VULCANEXUS_SETUP" "$BASHRC_FILE" || echo "$LINE_VULCANEXUS_SETUP" >> "$BASHRC_FILE"
    fi

    LINE_WS_SETUP="source ${WORKSPACE}/install/setup.bash"
    grep -qxF "$LINE_WS_SETUP" "$BASHRC_FILE" || echo "$LINE_WS_SETUP" >> "$BASHRC_FILE"

    if [ -f "/usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" ]; then
        LINE_COLCON_ARGCOMPLETE="source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash"
        grep -qxF "$LINE_COLCON_ARGCOMPLETE" "$BASHRC_FILE" || echo "$LINE_COLCON_ARGCOMPLETE" >> "$BASHRC_FILE"
    fi

    # Add Python virtual environment activation
    # if [ -d "/opt/ros_python_env" ]; then
    #     LINE_PYTHON_ENV="source /opt/ros_python_env/bin/activate"
    #     grep -qxF "$LINE_PYTHON_ENV" "$BASHRC_FILE" || echo "$LINE_PYTHON_ENV" >> "$BASHRC_FILE"
    # fi
fi

# Source ROS 2 environment for build
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

# Build ros2 packages
echo "Building ros2 packages of this repo..."
cd /workspace
colcon build --symlink-install --event-handlers console_direct+ 

# Source the updated workspace after building
if [ -f "/workspace/install/setup.bash" ]; then
    echo "Sourcing updated workspace environment..."
    source /workspace/install/setup.bash
fi

# Activate Python virtual environment
# if [ -d "/opt/ros_python_env" ]; then
#     source /opt/ros_python_env/bin/activate
# fi

# Change ownership of built artifacts
if [ -d "${WORKSPACE}/build" ]; then
    chown -R "$USER_ID:$GROUP_ID" "${WORKSPACE}/build"
fi
if [ -d "${WORKSPACE}/install" ]; then
    chown -R "$USER_ID:$GROUP_ID" "${WORKSPACE}/install"
fi
if [ -d "${WORKSPACE}/log" ]; then
    chown -R "$USER_ID:$GROUP_ID" "${WORKSPACE}/log"
fi

echo "=== ENTRYPOINT END $(date) PID=$$ ==="

# Execute the command as the created user
echo "Executing command as user $USERNAME ($USER_ID:$GROUP_ID): $@"
exec gosu "$USERNAME" "$@"
