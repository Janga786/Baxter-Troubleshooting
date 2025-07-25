# Use ROS 2 Humble as the base
FROM ros:humble

# Install MoveIt 2, ros2_control, and build dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-moveit \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-controller-manager \
    ros-humble-rviz2 \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p /ros/ws_moveit2/src
WORKDIR /ros/ws_moveit2

# Clone only the ros1_bridge (we'll build MoveIt config for Baxter separately)
RUN cd src && \
    git clone https://github.com/ros2/ros1_bridge.git

# Install Python dependencies for the bridge
RUN pip3 install --upgrade pip && \
    pip3 install catkin_pkg lark

# Initialize rosdep if needed
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi && \
    rosdep update

# Note: Building ros1_bridge requires both ROS1 and ROS2 environments
# We'll skip the full build for now as it needs ROS1 message definitions

# Source the environment automatically
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Add a script to build the bridge when both ROS environments are available
RUN echo '#!/bin/bash\n\
echo "To build the ROS1-ROS2 bridge, you need:"\n\
echo "1. Mount the ROS1 workspace from the other container"\n\
echo "2. Source both ROS1 and ROS2 environments"\n\
echo "3. Run: colcon build --packages-select ros1_bridge"\n\
' > /ros/build_bridge_info.sh && chmod +x /ros/build_bridge_info.sh

CMD ["/bin/bash"]