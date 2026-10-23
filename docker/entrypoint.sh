#!/bin/bash
# Окружения ROS, px4_msgs и проекта; затем команда контейнера.
set -e
source /opt/ros/jazzy/setup.bash
source /opt/px4_ws/install/setup.bash
source /ws/install/setup.bash
exec "$@"
