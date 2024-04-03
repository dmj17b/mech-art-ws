#!/bin/bash

# Get the absolute path to the directory containing this script
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Build the Docker image
docker build -t wishing-well:latest -f Dockerfile.wishing-well "${SCRIPT_DIR}" > /dev/null 2>&1

# Start the Docker container
docker run -it \
    --rm \
    --privileged \
    wishing-well:latest \
    python3 -c "import sounddevice as sd; print(sd.query_devices())"