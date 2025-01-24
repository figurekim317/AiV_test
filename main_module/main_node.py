import rclpy
from rclpy.node import Node
import yaml
from importlib import import_module

class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')
        self.declare_parameter('config_file', 'config/config1.yaml')
        config_path = self.get_parameter('config_file').get_parameter_value().string_value
        self.config = self.load_config(config_path)
        self.nodes = []
        self.load_nodes()
    
    def load_config(self, file_path):
        with open(file_path, 'r') as f:
            return yaml.safe_load(f)

    def load_nodes(self):
        for node_info in self.config['nodes']:
            module = import_module(f"{node_info['module']}.{node_info['name']}")
            node_class = getattr(module, node_info['name'].capitalize())
            self.nodes.append(node_class())

    def run(self):
        for node in self.nodes:
            node.run()

def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
