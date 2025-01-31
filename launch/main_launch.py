import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node

def load_config(config_file="config/config1.yaml"):
    """
    Loads the configuration file containing node definitions.

    Parameters:
        config_file (str): Path to the YAML configuration file.

    Returns:
        dict: Parsed YAML configuration containing node information.
    """
    config_path = os.path.join(os.getcwd(), config_file)

    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Configuration file not found: {config_path}")

    with open(config_path, 'r') as f:
        try:
            return yaml.safe_load(f)
        except yaml.YAMLError as e:
            raise ValueError(f"Error parsing YAML file: {config_path}\n{e}")

def generate_launch_description():
    """
    Generates a launch description by dynamically creating nodes based on the configuration file.

    Uses the `ROS_CONFIG_FILE` environment variable if set, otherwise defaults to `config/config1.yaml`.

    Returns:
        LaunchDescription: A description of the nodes to be launched.
    """
    config_file = os.getenv("ROS_CONFIG_FILE", "config/config1.yaml")
    config = load_config(config_file)

    nodes = []

    for node in config["nodes"]:
        node_params = {"id": node["id"]}

        # If additional parameters are provided in the config, add them
        if "parameters" in node:
            node_params.update(node["parameters"])

        nodes.append(
            Node(
                package=node["module"],
                executable=node["name"],
                name=node["name"],
                parameters=[node_params]
            )
        )

    return LaunchDescription(nodes)
