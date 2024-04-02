#!/bin/bash

xhost +local:root

# Get the absolute path to the directory containing this script
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Build the Docker image
docker build -t mechart-cv:latest "${SCRIPT_DIR}"

# Start the Docker container
docker run -it \
    --rm \
    -p 5000:5000 \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "/${SCRIPT_DIR}:/app" \
    --device /dev/video0:/dev/video0 \
    mechart-cv:latest \
    bash
