# Start from the official Ubuntu image
FROM ros:humble

# Install necessary packages
RUN apt-get update && \
    apt-get install -y \
        python3-pip \
        libgtk2.0-dev \
        pkg-config \
        libgl1-mesa-glx \
        libglib2.0-0 \
        libqt5x11extras5 \
        libx11-xcb1

RUN pip3 install \
        numpy  \
        opencv-python-headless  \
        flask \
        openai \
        requests \
        sounddevice \
        scipy

# Create a new user with a specific UID and GID, and set up the workspace
RUN useradd -m -u 1000 -s /bin/bash user && \
    mkdir -p /app && \
    chown -R user:user /app
WORKDIR /app

# Add the user to the video group
RUN usermod -aG video user

# Switch to the new non-root user
USER user

# Set the default command to execute when creating a new container
CMD ["bash"]
