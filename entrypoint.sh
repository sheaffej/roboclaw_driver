#!/bin/bash

source "/opt/ros/$ROS_DISTRO/setup.bash"
source "${ROS_WS}/devel/setup.bash"
exec "$@"