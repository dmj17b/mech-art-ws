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
    --volume /tmp/argus_socket:/tmp/argus_socket \
    --volume /etc/enctune.conf:/etc/enctune.conf \
    --volume /etc/nv_tegra_release:/etc/nv_tegra_release \
    --volume /tmp/nv_jetson_model:/tmp/nv_jetson_model \
    --volume /var/run/dbus:/var/run/dbus \
    --volume /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket \
    --volume /var/run/docker.sock:/var/run/docker.sock \
    --volume $ROOT/data:/data \
    big-brother-jetson:latest \
    /bin/bash -c "source /opt/ros/humble/install/setup.bash && \
                    colcon build --packages-select big_brother && \
                    source /app/install/setup.bash && \
                    ros2 launch big_brother big_brother_launch.py"
