#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import json
import re
from ros2_monitor_srvs.srv import GetTopicInfo

class TopicInfoService(Node):
    def __init__(self):
        super().__init__('topic_info_service')
        self.srv = self.create_service(GetTopicInfo, 'get_topic_info', self.get_topic_info_callback)
        self.get_logger().info('Topic Info Service started')
        print("Topic Info Service started")

    def get_topic_info_callback(self, request, response):
        try:
            topic_name = request.topic_name  # Use the custom field
            if not topic_name:
                response.success = False
                response.message = "No topic name provided"
                return response
            info_result = subprocess.run(
                ['ros2', 'topic', 'info', '--verbose', topic_name], 
                capture_output=True, text=True, timeout=10
            )
            if info_result.returncode == 0:
                parsed_info = self.parse_topic_info(info_result.stdout)
                response.success = True
                response.message = json.dumps(parsed_info)
            else:
                response.success = False
                response.message = f"Error getting info for topic {topic_name}: {info_result.stderr}"
        except subprocess.TimeoutExpired:
            response.success = False
            response.message = "Timeout getting topic information"
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
        return response

    def parse_topic_info(self, info_text):
        """Parse the output of 'ros2 topic info --verbose' (nuevo formato)"""
        info = {
            'type': '',
            'publishers': [],
            'subscribers': [],
            'publisher_count': 0,
            'subscriber_count': 0
        }
        lines = info_text.strip().split('\n')
        # Extrae tipo y contadores
        for line in lines:
            if line.startswith('Type:'):
                info['type'] = line.replace('Type:', '').strip()
            elif line.startswith('Publisher count:'):
                info['publisher_count'] = int(line.replace('Publisher count:', '').strip())
            elif line.startswith('Subscriber count:'):
                info['subscriber_count'] = int(line.replace('Subscriber count:', '').strip())
        # Parsear endpoints
        endpoint = None
        qos = None
        for line in lines:
            line = line.strip()
            if line.startswith('Node name:'):
                if endpoint:
                    # Guarda el endpoint anterior
                    if endpoint.get('endpoint_type') == 'PUBLISHER':
                        info['publishers'].append(endpoint)
                    elif endpoint.get('endpoint_type') == 'SUBSCRIBER':
                        info['subscribers'].append(endpoint)
                endpoint = {
                    'node_name': line.replace('Node name:', '').strip(),
                    'node_namespace': '',
                    'topic_type': '',
                    'endpoint_type': '',
                    'gid': '',
                    'qos': {}
                }
                qos = None
            elif endpoint is not None:
                if line.startswith('Node namespace:'):
                    endpoint['node_namespace'] = line.replace('Node namespace:', '').strip()
                elif line.startswith('Topic type:'):
                    endpoint['topic_type'] = line.replace('Topic type:', '').strip()
                elif line.startswith('Endpoint type:'):
                    endpoint['endpoint_type'] = line.replace('Endpoint type:', '').strip()
                elif line.startswith('GID:'):
                    endpoint['gid'] = line.replace('GID:', '').strip()
                elif line.startswith('QoS profile:'):
                    qos = {}
                elif qos is not None and ':' in line:
                    k, v = line.split(':', 1)
                    qos[k.strip()] = v.strip()
                elif qos is not None and line == '':
                    endpoint['qos'] = qos
                    qos = None
        # Guarda el último endpoint
        if endpoint:
            if endpoint.get('endpoint_type') == 'PUBLISHER':
                info['publishers'].append(endpoint)
            elif endpoint.get('endpoint_type') == 'SUBSCRIBER':
                info['subscribers'].append(endpoint)
        return info

def main(args=None):
    rclpy.init(args=args)
    topic_info_service = TopicInfoService()
    rclpy.spin(topic_info_service)
    topic_info_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 