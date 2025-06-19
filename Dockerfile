FROM ros:noetic

# Use bash as a default shell
CMD ["/bin/bash"]

# Install dependencies
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    bash git curl wget sudo build-essential cmake \
    python3-pip \
    ros-noetic-apriltag-ros \
    ros-noetic-turtlebot3-gazebo \
    ros-noetic-turtlebot3-simulations \
    ros-noetic-rviz \
    ros-noetic-tf2-ros \
    ros-noetic-geometry-msgs \
    ros-noetic-cv-bridge \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# Create user
RUN useradd -m dev

# Set environment
ENV ROS_WORKSPACE=/home/dev/catkin_ws

# Switch to user and workspace
USER dev
WORKDIR /home/dev/catkin_ws