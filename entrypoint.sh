#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

echo "Lanzando Vite (frontend) en modo dev..."
npm run dev -- --host 0.0.0.0 --port 5173 &

echo "Launching ROS2 Graph Introspector..."
python3 /ros2_ws/ros2_graph_introspector.py &

echo "Launching Topic Info Service..."
python3 /ros2_ws/topic_info_service.py &

echo "Launching rosbridge..."
exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml 