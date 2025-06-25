# ROS2 Monitor

This project provides a web-based monitor for ROS2 systems, including a dynamic graph visualization of nodes, topics, and services.

## Running with Docker

You need to launch **two Docker containers**:

1. **rosbridge** (for WebSocket communication with the frontend)
2. **graph inspector** (to publish the ROS2 graph structure as JSON)

### 1. Launch rosbridge

By default, the container will launch rosbridge:

```bash
docker run \
  --rm \
  singularaircraft/ros2-rosbridge:latest
```

Or explicitly:

```bash
docker run \
  --rm \
  -e ROS2_MONITOR_NODE=bridge \
  singularaircraft/ros2-rosbridge:latest
```

### 2. Launch the graph inspector

This container publishes the ROS2 graph structure on the `/ros2_graph` topic as JSON:

```bash
docker run \
  --rm \
  -e ROS2_MONITOR_NODE=inspector \
  singularaircraft/ros2-rosbridge:latest
```

## Frontend

`npm run dev`
