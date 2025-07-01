FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y \
    software-properties-common \
    curl \
    git && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-ros-base \
    ros-humble-rosbridge-server && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y ca-certificates curl gnupg && \
    mkdir -p /etc/apt/keyrings && \
    curl -fsSL https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key | gpg --dearmor -o /etc/apt/keyrings/nodesource.gpg && \
    NODE_MAJOR=20 && \
    echo "deb [signed-by=/etc/apt/keyrings/nodesource.gpg] https://deb.nodesource.com/node_$NODE_MAJOR.x nodistro main" | tee /etc/apt/sources.list.d/nodesource.list && \
    apt-get update && \
    apt-get install -y nodejs && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY package*.json ./
RUN npm install

COPY . .

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --packages-select ros2_monitor_srvs && \
    colcon build --symlink-install && \
    rm -rf /var/lib/apt/lists/*"

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 5173 9090

ENTRYPOINT ["/entrypoint.sh"]
