#!/bin/bash
set -e

ros2 launch /app/src/big_brother/launch/ros_deep_learning.launch

exec "$@"