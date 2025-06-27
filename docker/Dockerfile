# Use Ubuntu 22.04 as base (for ROS 2 Humble)
FROM ubuntu:22.04

# Set non-interactive mode
ENV DEBIAN_FRONTEND=noninteractive

# Install ROS 2 Humble, rosbridge_suite, and Node.js
RUN apt-get update && \
    apt-get install -y curl gnupg2 lsb-release locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    echo "deb http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list && \
    apt-get update && \
    apt-get install -y ros-humble-ros-base ros-humble-rosbridge-suite && \
    rm -rf /var/lib/apt/lists/*

# Install Node.js and npm
RUN curl -fsSL https://deb.nodesource.com/setup_18.x | bash - && \
    apt-get install -y nodejs

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Setup ROS environment
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
ENV ROS_DISTRO=humble

# Copy the entire project
COPY . /app
WORKDIR /app

# Install frontend dependencies
RUN npm install

# No build ni dist, ejecuta Vite en modo dev
EXPOSE 5173 9090 8000

# Copy entrypoint and ROS2 utilities
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
COPY ros2_utils/ros2_graph_introspector.py /ros2_ws/ros2_graph_introspector.py
COPY ros2_utils/topic_info_service.py /ros2_ws/topic_info_service.py
RUN chmod +x /ros2_ws/topic_info_service.py

# Instala colcon y build-essential para poder compilar el workspace ROS2
RUN apt-get update && \
    apt-get install -y python3-colcon-common-extensions build-essential

# Copy the custom ROS2 service package
COPY ros2_utils/ros2_monitor_srvs /ros2_ws/src/ros2_monitor_srvs

# Build the ROS2 workspace (for custom services)
RUN source /opt/ros/humble/setup.bash && \
    cd /ros2_ws && \
    colcon build --symlink-install

# Source the workspace in the entrypoint
RUN echo 'source /ros2_ws/install/setup.bash' >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"] 