#!/bin/bash
set -e

source /opt/ros/humble/install/setup.bash
source /ros_deep_learning/install/setup.bash
ros2 launch /app/src/big_brother/launch/ros_deep_learning.launch

exec "$@"