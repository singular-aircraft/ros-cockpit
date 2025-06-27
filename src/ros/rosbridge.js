import ROSLIB from 'roslib';

// Returns a new ROSLIB.Ros instance for the given host
export default function rosInit(url) {
  const ros = new ROSLIB.Ros({ url });
ros.on('connection', () => {
  console.log('Connected to rosbridge websocket server.');
});
ros.on('error', (error) => {
  console.error('Error connecting to rosbridge:', error);
});
ros.on('close', () => {
  console.log('Connection to rosbridge closed.');
});
  return ros;
} 