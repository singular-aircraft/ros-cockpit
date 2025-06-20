# ros2-monitor

Frontend to monitor ROS2 data using rosbridge, React, and Three.js.

## Requirements
- rosbridge_server running on your ROS2 machine (default at ws://localhost:9090)
- Node.js and npm installed

## Installation

```bash
npm install
```

## Usage

```bash
npm run dev
```

The app will open at http://localhost:5173 (by default).

## rosbridge configuration
Make sure you have rosbridge_server running:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

---

Modify and expand the widgets as needed!
