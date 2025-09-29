#!/usr/bin/env bash

export DOCKER_BUILDKIT=1

set -e

# --- BEGIN: Manage .env file ---
# ENV_FILE="./.env" # Assuming .env is in the same directory as this script
# DEFAULT_USERNAME="coghri"
# DEFAULT_ROS_DOMAIN_ID="1"
# CURRENT_USER_ID=$(id -u)
# CURRENT_GROUP_ID=$(id -g)

# # Create .env if it doesn't exist
# if [ ! -f "$ENV_FILE" ]; then
#     echo "Creating $ENV_FILE..."
#     touch "$ENV_FILE"
# fi

# # Set USERNAME if not present, otherwise leave existing value
# if ! grep -q -E "^USERNAME=" "$ENV_FILE"; then
#     echo "USERNAME=$DEFAULT_USERNAME" >> "$ENV_FILE"
# fi

# # Set ROS_DOMAIN_ID if not present, otherwise leave existing value
# if ! grep -q -E "^ROS_DOMAIN_ID=" "$ENV_FILE"; then
#     echo "ROS_DOMAIN_ID=$DEFAULT_ROS_DOMAIN_ID" >> "$ENV_FILE"
# fi

# # Set or Update USER_ID with current host user ID
# if grep -q -E "^USER_ID=" "$ENV_FILE"; then
#     sed -i "s/^USER_ID=.*/USER_ID=$CURRENT_USER_ID/" "$ENV_FILE"
# else
#     echo "USER_ID=$CURRENT_USER_ID" >> "$ENV_FILE"
# fi

# # Set or Update GROUP_ID with current host group ID
# if grep -q -E "^GROUP_ID=" "$ENV_FILE"; then
#     sed -i "s/^GROUP_ID=.*/GROUP_ID=$CURRENT_GROUP_ID/" "$ENV_FILE"
# else
#     echo "GROUP_ID=$CURRENT_GROUP_ID" >> "$ENV_FILE"
# fi
# --- END: Manage .env file ---

# DEPS_DIR="./deps"


# Check if --clean-rebuild is among the arguments
REBUILD=false
for arg in "$@"; do
    if [ "$arg" == "--clean-rebuild" ]; then
        REBUILD=true
    fi
done

if $REBUILD; then # remove all files just in case some modifications have been made and git pull does not work
    echo "Rebuilding: cleaning up dependencies..."
fi

# vcs import ${BT_DEPS_DIR} < deps.repos
# vcs pull ${BT_DEPS_DIR}

if $REBUILD; then
    echo "Rebuilding the Docker image..."
    docker build --ssh default --no-cache . -t eut_ros_vulcanexus_torch:jazzy -f Dockerfile
else
    docker build --ssh default . -t eut_ros_vulcanexus_torch:jazzy -f Dockerfile
fi