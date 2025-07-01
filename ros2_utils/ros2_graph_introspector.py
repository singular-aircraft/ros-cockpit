import rclpy
from rclpy.node import Node
from ros2_monitor_srvs.srv import GetGraphInfo
import json

class GraphIntrospectorService(Node):
    def __init__(self):
        super().__init__('graph_introspector_service')
        self.srv = self.create_service(GetGraphInfo, 'get_graph_info', self.handle_get_graph_info)
        self.get_logger().info('Graph Introspector Service started')
        print("Graph Introspector Service started")
        
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
        services_data = []
        # Define a filter for common parameter-related services
        SERVICE_FILTER_BASE_NAMES = [
            'set_parameters',
            'get_parameters',
            'list_parameters',
            'describe_parameters',
            'get_parameter_types',
            'set_parameters_atomically'
        ]

        def is_filtered_service(service_name):
            # Extract the base service name (the part after the last '/')
            base_service_name = service_name.split('/')[-1]
            for base_filter_name in SERVICE_FILTER_BASE_NAMES:
                if base_service_name == base_filter_name:
                    return True
            return False

        # Get all service names and types
        all_services_and_types = super().get_service_names_and_types()

        # Get all node names and namespaces
        node_names_and_namespaces = super().get_node_names_and_namespaces()

        # Create a mapping from service name to its types for quick lookup
        service_types_map = {name: types for name, types in all_services_and_types}

        # Iterate through each node to find services it provides
        for node_name, node_namespace in node_names_and_namespaces:
            try:
                services_by_node = self.get_service_names_and_types_by_node(node_name, node_namespace)
                for service_name, service_types in services_by_node:
                    if not is_filtered_service(service_name):
                        # Use the service_types from the direct query if available, otherwise from the general map
                        service_type = service_types[0] if service_types else service_types_map.get(service_name, [""])[0]
                        services_data.append({
                            "name": service_name,
                            "type": service_type,
                            "node": node_name # The node providing the service
                        })
            except Exception as e:
                self.get_logger().warn(f"Could not get services for node {node_name}/{node_namespace}: {e}")

        # Remove duplicate services (if a service is provided by multiple nodes, we'll just list one for now)
        # A more sophisticated approach might list all providers or handle this on the frontend
        unique_services = {}
        for service in services_data:
            if service["name"] not in unique_services:
                unique_services[service["name"]] = service
        services_data = list(unique_services.values())

        service_clients = []
        for node_name, node_namespace in node_names_and_namespaces:
            try:
                clients_by_node = self.get_client_names_and_types_by_node(node_name, node_namespace)
                for client_service_name, client_service_types in clients_by_node:
                    if not is_filtered_service(client_service_name):
                        service_clients.append({
                            "node": node_name,
                            "service": client_service_name,
                            "type": client_service_types[0] if client_service_types else ""
                        })
            except Exception as e:
                self.get_logger().warn(f"Could not get service clients for node {node_name}/{node_namespace}: {e}")

        graph = {
            "topics": [
                {"name": name, "types": types} for name, types in topics_and_types
            ],
            "nodes": [
                {"name": name, "namespace": ns} for name, ns in node_names
            ],
            "publishers": publishers,
            "subscribers": subscribers,
            "services": services_data,
            "service_clients": service_clients
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