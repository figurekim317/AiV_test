import os
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml

def load_config(config_file="config/config1.yaml"):
    with open(os.path.join(os.getcwd(), config_file), 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():
    config_file = os.getenv("ROS_CONFIG_FILE", "config/config1.yaml")  # 환경 변수로 선택 가능
    config = load_config(config_file)

    nodes = [
        Node(
            package=node["module"],
            executable=node["name"],
            name=node["name"],
            parameters=[{"id": node["id"]}]
        ) for node in config["nodes"]
    ]
    
    return LaunchDescription(nodes)
