FROM ros:humble

# Use bash shell
SHELL ["bash", "-c"]

# Set working directory
WORKDIR /root/nav2_ws

# Copy source files into the workspace
COPY ./src /root/nav2_ws/src

# Set non-interactive frontend for apt-get
ARG DEBIAN_FRONTEND=noninteractive

# Install dependencies and required ROS 2 packages
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r --default-yes && \
    apt-get install -y \
    ros-humble-gazebo-ros \
    ros-humble-navigation2 \
    ros-humble-gazebo-msgs \
    ros-humble-gazebo-plugins \
    ros-humble-nav2-bringup

# Source ROS 2 and build nav2_cpp_simple_commander
RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-select nav2_common nav2_msgs nav2_cpp_simple_commander && \
    echo "source /root/nav2_ws/install/setup.bash" >> ~/.bashrc

# Modify the entrypoint to source the workspace setup file automatically
RUN sed -i '/^exec "$@"/i source /root/nav2_ws/install/setup.bash' /ros_entrypoint.sh

# Default command to keep the container running
ENTRYPOINT ["tail", "-f", "/dev/null"]
