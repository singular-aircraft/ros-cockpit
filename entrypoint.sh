#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

echo "Launching ROS2 Graph Introspector..."
python3 /ros2_ws/ros2_graph_introspector.py &

echo "Launching rosbridge..."
exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml 