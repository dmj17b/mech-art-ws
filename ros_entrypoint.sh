#!/bin/bash

source /opt/ros/humble/install/setup.bash && ros2 launch /app/src/big_brother/launch/ros_deep_learning.launch

exec "$@"