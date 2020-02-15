#!/usr/bin/env bash

DOCKER_IMAGE="sheaffej/roboclaw_driver"

# [ -z "$ROS_MASTER_URI" ] && echo "Please set ROS_MASTER_URI env" && exit 1

# MYDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
# REPOS_DIR=$MYDIR/../..  # Directory containing the cloned git repos
# CODE_MOUNT="/workspaces"

docker run -it --rm \
--privileged \
--net host \
--env DISPLAY \
${DOCKER_IMAGE} $@

# --env ROS_MASTER_URI \