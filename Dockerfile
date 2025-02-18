# Use the official ROS 2 Galactic base image
FROM osrf/ros:galactic-desktop

# Install additional dependencies (colcon build tool, etc.)
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Copy your ROS2 workspace into the container.
# This assumes your package (and the launch folder) is in the current directory.

# Set the working directory
RUN mkdir -p ~/ros2_ws \
    && cd ~/ros2_ws \
    && git clone https://github.com/uwrobotics/UWRT_Controller_StateMachine.git -b y29lin/device-config \
    && cd UWRT_Controller_StateMachine \
    && git submodule init \
    && git submodule update --recursive \
    && cd ~/ros2_ws


# Build the workspace
RUN . /opt/ros/galactic/setup.sh \
    && apt-get update \
    && apt-get install nlohmann-json3-dev \