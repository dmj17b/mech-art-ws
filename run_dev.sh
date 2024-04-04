#!/bin/bash

xhost +local:root

# Get the absolute path to the directory containing this script
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Build the Docker image
docker build -t mechart-dev:latest -f docker/Dockerfile.dev "${SCRIPT_DIR}"

# Start the Docker container
docker run -it --rm \
    --privileged \
    --network host \
    --pid host \
    --ipc host \
    --env UID=$(id -u) \
    --env GID=$(id -g) \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e ROS_DOMAIN_ID \
    --runtime nvidia \
    -v /dev/*:/dev/* \
    -v "/${SCRIPT_DIR}:/app/src" \
    -e "OPENAI_API_KEY=$(cat openai-key.txt)" \
    mechart-dev:latest
