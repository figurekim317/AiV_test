import os
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml

CONFIG_FILE = os.path.join(os.getcwd(), 'config', 'config1.yaml')

def load_config():
    with open(CONFIG_FILE, 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():
    config = load_config()
    
    nodes = [
        Node(
            package=node["module"],
            executable=node["name"],
            name=node["name"]
        ) for node in config["nodes"]
    ]
    
    return LaunchDescription(nodes)
