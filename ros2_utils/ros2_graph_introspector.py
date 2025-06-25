import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class GraphIntrospector(Node):
    def __init__(self):
        super().__init__('graph_introspector')
        print('[GraphIntrospector] Node initialized2')
        self.publisher_ = self.create_publisher(String, '/ros2_graph', 10)
        self.timer = self.create_timer(2.0, self.publish_graph)

    def publish_graph(self):
        print('[GraphIntrospector] Running publish_graph')
        # Get all topics and their types
        topics_and_types = self.get_topic_names_and_types()
        print(f'[GraphIntrospector] topics_and_types: {topics_and_types}')
        # Get all nodes
        node_names = self.get_node_names_and_namespaces()
        print(f'[GraphIntrospector] node_names: {node_names}')
        # Build a simple graph structure
        graph = {
            'topics': [],
            'nodes': [],
            'publishers': [],
            'subscribers': []
        }
        for topic, types in topics_and_types:
            graph['topics'].append({'name': topic, 'types': types})
        for name, ns in node_names:
            graph['nodes'].append({'name': name, 'namespace': ns})
        # Get publishers/subscribers info per topic
        for topic, _ in topics_and_types:
            pubs = self.get_publishers_info_by_topic(topic)
            subs = self.get_subscriptions_info_by_topic(topic)
            for pub in pubs:
                graph['publishers'].append({'node': pub.node_name, 'topic': topic})
            for sub in subs:
                graph['subscribers'].append({'node': sub.node_name, 'topic': topic})
        print(f'[GraphIntrospector] graph: {json.dumps(graph)}')
        # Publish as JSON
        msg = String()
        msg.data = json.dumps(graph)
        self.publisher_.publish(msg)
        print('[GraphIntrospector] Message published on /ros2_graph')

def main(args=None):
    rclpy.init(args=args)
    node = GraphIntrospector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 