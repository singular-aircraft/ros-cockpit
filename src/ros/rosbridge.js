import ROSLIB from 'roslib';

// Change the URL if your rosbridge is running on a different IP/port
const ros = new ROSLIB.Ros({
  url: 'ws://10.0.1.130:9090',
});

ros.on('connection', () => {
  console.log('Connected to rosbridge websocket server.');
});
ros.on('error', (error) => {
  console.error('Error connecting to rosbridge:', error);
});
ros.on('close', () => {
  console.log('Connection to rosbridge closed.');
});

export default ros; 