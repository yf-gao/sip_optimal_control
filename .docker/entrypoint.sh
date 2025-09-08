#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
# setup workspace environment
cd /_ws
source "/_ws/install/setup.bash"
exec "$@"
