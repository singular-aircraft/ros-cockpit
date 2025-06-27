# ROS2 Monitor

A web-based monitoring interface for ROS2 systems using React and rosbridge.

## Features

- **Real-time Topic Monitoring**: Subscribe to ROS2 topics and view messages in real-time
- **Message Visualization**: Display messages in JSON format with optional hexadecimal view
- **Topic Information**: Get detailed verbose information about topics including publishers and subscribers
- **Chart Visualization**: Plot numeric data from topic messages
- **Node Graph**: Visualize the ROS2 node graph with D3.js
- **Connection Status**: Real-time connection status indicator
- **Dynamic Host Configuration**: Configure rosbridge host via UI

## Topic Information Service

The application includes a custom ROS2 service that provides verbose topic information directly in the web interface. This service:

- Executes `ros2 topic info --verbose` for selected topics
- Parses the output to extract publisher and subscriber information
- Returns structured data that can be displayed in the web UI
- Runs automatically when the Docker container starts

### What you'll see:

- **Topic Type**: The message type (e.g., `std_msgs/String`)
- **Publishers**: List of nodes publishing to the topic with connection counts
- **Subscribers**: List of nodes subscribing to the topic with connection counts
- **Real-time Updates**: Refresh button to get updated information

## Quick Start

### Using Docker (Recommended)

1. **Build the Docker image**:
   ```bash
   docker build -f docker/Dockerfile -t ros2-monitor .
   ```

2. **Run the container**:
   ```bash
   docker run -p 8000:8000 -p 9090:9090 ros2-monitor
   ```

3. **Access the web interface**:
   - Open http://localhost:8000 in your browser
   - The frontend is served on port 8000
   - rosbridge is available on port 9090
   - Configure the rosbridge host as: ws://localhost:9090

### Manual Setup

1. **Install dependencies**:
   ```bash
   npm install
   ```

2. **Start the development server**:
   ```bash
   npm run dev
   ```

3. **Start rosbridge** (in a separate terminal):
   ```bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```

4. **Start the topic info service** (in another terminal):
   ```bash
   python3 ros2_utils/topic_info_service.py
   ```

## Usage

1. **Connect to ROS2**: Enter the rosbridge WebSocket URL (default: ws://127.0.0.1:9090)
2. **Select a Topic**: Choose from the list of available topics
3. **View Information**: 
   - **Messages**: See real-time message data
   - **Topic Info**: Get detailed publisher/subscriber information
   - **Chart**: Plot numeric fields from messages
   - **Nodes**: View the ROS2 node graph

## Architecture

- **Frontend**: React with Vite
- **ROS2 Communication**: rosbridge WebSocket interface
- **Topic Info Service**: Custom ROS2 service for verbose topic information
- **Visualization**: Chart.js for plots, D3.js for node graphs

## Development

### Project Structure

```
ros2-monitor/
├── src/
│   ├── components/
│   │   └── RosMonitorWidget.jsx    # Main monitoring component
│   ├── ros/
│   │   └── rosbridge.js            # ROS2 connection setup
│   └── App.jsx                     # Main application
├── ros2_utils/
│   ├── topic_info_service.py       # ROS2 service for topic info
│   └── ros2_graph_introspector.py  # Node graph data collection
├── docker/
│   └── Dockerfile                  # Container configuration
└── entrypoint.sh                   # Container startup script
```

### Building for Production

```bash
npm run build
```

The built files will be in the `dist/` directory.

## Troubleshooting

### Connection Issues

- Ensure rosbridge is running: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
- Check the WebSocket URL in the UI
- Verify ROS2 topics are being published

### Topic Information Not Available

- The topic info service runs automatically in Docker
- For manual setup, run: `python3 ros2_utils/topic_info_service.py`
- Check that the service is available: `ros2 service list | grep get_topic_info`

### Build Issues

- Clear node_modules and reinstall: `rm -rf node_modules && npm install`
- Check for missing dependencies in package.json

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

This project is licensed under the MIT License.
