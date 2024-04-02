#!/bin/bash

xhost +local:root

# Get the absolute path to the directory containing this script
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Build the Docker image
docker build -t mechart:latest "${SCRIPT_DIR}"

# Start the Docker container
docker run -it \
    --rm \
    --net host \
    -e "OPENAI_API_KEY=$(cat openai-key.txt)" \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "/${SCRIPT_DIR}:/app" \
    mechart:latest \
    bash
