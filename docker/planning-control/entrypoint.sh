#!/usr/bin/env bash
# shellcheck disable=SC1090,SC1091

source "/opt/ros/$ROS_DISTRO/setup.bash"
source /opt/autoware/setup.bash
exec "$@"
