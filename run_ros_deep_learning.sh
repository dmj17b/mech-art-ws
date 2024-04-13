#!/bin/bash

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

cd ${SCRIPT_DIR}/../jetson-inference && \
    ./docker/run.sh \
    --ros=humble \
    -v ${SCRIPT_DIR}:/app/src \
    --run "./ros_entrypoint.sh && \
            ros2 launch /app/src/big_brother/launch/ros_deep_learning.launch"