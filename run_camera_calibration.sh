#!/bin/bash

xhost +local:root

# Get the absolute path to the directory containing this script
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Build the Docker image
docker build -t big-brother:latest -f docker/Dockerfile.big-brother "${SCRIPT_DIR}"

# Start the Docker container
docker run -it \
    --rm \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "/${SCRIPT_DIR}:/app/src" \
    big-brother:latest \
    /bin/bash -c "colcon build --packages-select big_brother && source install/setup.bash && ros2 run big_brother camera_calibration_node"
