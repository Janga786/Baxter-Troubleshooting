# Use Ubuntu 18.04 as base to support both ROS1 Melodic and ROS2 Humble bridge
FROM ubuntu:18.04

# Prevent interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Install basic dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    python3-pip \
    git \
    && rm -rf /var/lib/apt/lists/*

# Add ROS1 repository
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Add ROS2 repository  
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS1 Melodic (compatible with Indigo messages) and setup for ROS2
RUN apt-get update && apt-get install -y \
    ros-melodic-ros-base \
    ros-melodic-std-msgs \
    ros-melodic-geometry-msgs \
    ros-melodic-sensor-msgs \
    ros-melodic-trajectory-msgs \
    ros-melodic-control-msgs \
    ros-melodic-actionlib-msgs \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 Humble via binary (since we need specific bridge compatibility)
RUN pip3 install --upgrade pip && \
    pip3 install catkin_pkg lark

# Initialize rosdep
RUN rosdep init && rosdep update

# Create a launch script
RUN echo '#!/bin/bash\n\
source /opt/ros/noetic/setup.bash\n\
source /opt/ros/humble/setup.bash\n\
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics' > /start_bridge.sh && \
chmod +x /start_bridge.sh

CMD ["/start_bridge.sh"]