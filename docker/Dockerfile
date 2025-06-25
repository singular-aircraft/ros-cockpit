# Use Ubuntu 22.04 as base (for ROS 2 Humble)
FROM ubuntu:22.04

# Set non-interactive mode
ENV DEBIAN_FRONTEND=noninteractive

# Install ROS 2 Humble and rosbridge_suite
RUN apt-get update && \
    apt-get install -y curl gnupg2 lsb-release locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    echo "deb http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list && \
    apt-get update && \
    apt-get install -y ros-humble-ros-base ros-humble-rosbridge-suite && \
    rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Setup ROS environment
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
ENV ROS_DISTRO=humble


# Start rosbridge
CMD source /opt/ros/humble/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml 

# Copy entrypoint and graph introspector
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
COPY ros2_utils/ros2_graph_introspector.py /ros2_ws/ros2_graph_introspector.py

ENTRYPOINT ["/entrypoint.sh"] 