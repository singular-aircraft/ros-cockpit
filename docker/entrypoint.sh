#!/bin/bash
set -m # Enable Job Control

# Function to handle exit signals gracefully
cleanup() {
    echo "Caught SIGINT/SIGTERM signal! Shutting down all processes..."
    # Kill all background jobs that are children of this script
    kill $(jobs -p)
    wait # Wait for all jobs to terminate
    echo "Shutdown complete."
    exit 0
}

# Trap SIGINT (Ctrl+C) and SIGTERM to call the cleanup function
trap cleanup SIGINT SIGTERM

# --- 1. Execute Custom Setup Command (Optional) ---
if [ -n "$SETUP_COMMAND" ]; then
  echo "--- Found SETUP_COMMAND. Executing... ---"
  apt-get update
  eval "$SETUP_COMMAND"
  echo "--- Custom setup command finished. ---"
fi

# --- 2. Sourcing of all ROS environments ---
source /opt/ros/humble/setup.bash
source /app/install/setup.bash
echo "ROS Humble environment sourced."

# No pre-existing workspace to source in this image.
# --- 3. Launch all services in the background ---
echo "--- Launching ROS services... ---"
python3 /app/ros2_utils/ros2_graph_introspector.py &
python3 /app/ros2_utils/topic_info_service.py &
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &

echo "--- Launching Frontend service... ---"
# Assumes the working directory is /app as set in the Dockerfile
(cd /app && npm run dev -- --host 0.0.0.0 --port 5173) &

# Wait for all background processes. The script will pause here.
# The trap will handle the exit.
echo "All services started. Monitoring..."
wait