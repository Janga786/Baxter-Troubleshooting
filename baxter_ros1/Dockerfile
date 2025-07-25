# Use ROS 1 Indigo as the base
FROM ros:indigo

# Install system dependencies and the Baxter SDK
RUN apt-get update && apt-get install -y \
    git \
    python-catkin-tools \
    python-rosdep \
    python-wstool \
    ros-indigo-actionlib \
    ros-indigo-cv-bridge \
    ros-indigo-control-msgs \
    ros-indigo-trajectory-msgs \
    ros-indigo-rospy-message-converter \
    ros-indigo-xacro \
    ros-indigo-tf2-ros \
    ros-indigo-gazebo-ros \
    ros-indigo-gazebo-ros-control \
    && rm -rf /var/lib/apt/lists/*

# Set up the workspace
RUN mkdir -p /ros/ws_baxter/src
WORKDIR /ros/ws_baxter

# Clone Baxter repositories directly (without wstool to avoid interaction)
RUN cd src && \
    git clone https://github.com/RethinkRobotics/baxter.git && \
    git clone https://github.com/RethinkRobotics/baxter_interface.git && \
    git clone https://github.com/RethinkRobotics/baxter_tools.git && \
    git clone https://github.com/RethinkRobotics/baxter_common.git && \
    git clone https://github.com/RethinkRobotics/baxter_examples.git

# Initialize rosdep if not already done
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi && \
    rosdep update

# Install all dependencies
RUN rosdep install --from-paths src --ignore-src -y -r

# Build the workspace
RUN /bin/bash -c "source /opt/ros/indigo/setup.bash && catkin_make"

# Source the environment automatically
RUN echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc && \
    echo "source /ros/ws_baxter/devel/setup.bash" >> ~/.bashrc

# Set the ROS_MASTER_URI (you'll need to update this with your Baxter's IP)
ENV ROS_MASTER_URI=http://baxter.local:11311

# Set the ROS_MASTER_URI
ENV ROS_MASTER_URI=http://192.168.42.2:11311

CMD ["/bin/bash"]