#!/bin/bash

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

cd ${SCRIPT_DIR}/../jetson-inference && \
    ./docker/run.sh \
    --ros=humble \
    -v ${SCRIPT_DIR}:/app/src \
    --run "/bin/bash -c \"source /opt/ros/humble/setup.bash && \
                          ros2 launch /app/src/big_brother/launch/ros_deep_learning.launch\""