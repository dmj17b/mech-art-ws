#!/bin/bash

xhost +local:root

# Get the absolute path to the directory containing this script
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Build the Docker image
docker build -t big-brother-jetson:latest -f docker/Dockerfile.big-brother-jetson "${SCRIPT_DIR}"

# Start the Docker container
docker run -it --rm \
    --privileged \
    --runtime nvidia \
    --network host \
    --pid host \
    --ipc host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "/${SCRIPT_DIR}:/app/src" \
    big-brother-jetson:latest \
    /bin/bash -c "source /opt/ros/humble/install/setup.bash && \
                    colcon build --packages-select big_brother && \
                    source /app/install/setup.bash && \
                    ros2 launch big_brother big_brother_launch.py"
