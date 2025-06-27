import rclpy
from rclpy.node import Node
from ros2_monitor_srvs.srv import GetGraphInfo
import json

class GraphIntrospectorService(Node):
    def __init__(self):
        super().__init__('graph_introspector_service')
        self.srv = self.create_service(GetGraphInfo, 'get_graph_info', self.handle_get_graph_info)

    def handle_get_graph_info(self, request, response):
        graph = self.build_graph()
        response.graph_json = json.dumps(graph)
        return response

    def build_graph(self):
        # Enriquecido: añade publishers y subscribers
        topics_and_types = super().get_topic_names_and_types()
        node_names = super().get_node_names_and_namespaces()
        publishers = []
        subscribers = []
        get_pubs = getattr(self, 'get_publishers_info_by_topic', None)
        get_subs = getattr(self, 'get_subscriptions_info_by_topic', None)
        for topic, _ in topics_and_types:
            if get_pubs:
                for pub in get_pubs(topic):
                    publishers.append({"node": pub.node_name, "topic": topic})
            if get_subs:
                for sub in get_subs(topic):
                    subscribers.append({"node": sub.node_name, "topic": topic})
        graph = {
            "topics": [
                {"name": name, "types": types} for name, types in topics_and_types
            ],
            "nodes": [
                {"name": name, "namespace": ns} for name, ns in node_names
            ],
            "publishers": publishers,
            "subscribers": subscribers
        }
        return graph

    def get_topic_names_and_types(self):
        # Devuelve lista de (topic_name, [types])
        return self.get_topic_names_and_types()

    def get_node_names_and_namespaces(self):
        # Devuelve lista de (node_name, namespace)
        return self.get_node_names_and_namespaces()

def main(args=None):
    rclpy.init(args=args)
    node = GraphIntrospectorService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 