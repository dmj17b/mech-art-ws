#!/bin/bash

# Get the absolute path to the directory containing this script
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Build the Docker image
docker build -t wishing-well:latest -f docker/Dockerfile.wishing-well "${SCRIPT_DIR}"

# Start the Docker container
docker run -it --rm \
    --privileged \
    --user root \
    --network host \
    --pid host \
    --ipc host \
    -v "/${SCRIPT_DIR}:/app/src" \
    -e "OPENAI_API_KEY=$(cat openai-key.txt)" \
    wishing-well:latest \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
                    colcon build --packages-select wishing_well && \
                    source /app/install/setup.bash && \
                    ros2 launch wishing_well wishing_well_launch.py"