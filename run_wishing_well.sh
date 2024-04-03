#!/bin/bash

# Get the absolute path to the directory containing this script
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Build the Docker image
docker build -t wishing-well:latest -f Dockerfile.wishing-well "${SCRIPT_DIR}"

# Start the Docker container
docker run -it \
    --rm \
    -p 5000:5000 \
    --privileged \
    -e "OPENAI_API_KEY=$(cat openai-key.txt)" \
    -v "/${SCRIPT_DIR}:/app" \
    wishing-well:latest \
    /bin/bash -c "colcon build --packages-select wishing_well && source install/setup.bash && ros2 launch wishing_well wishing_well_launch.py"
