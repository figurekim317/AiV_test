import rclpy
import sys
import yaml
import os
from robot_module.robot_type1 import RobotType1
from robot_module.robot_type2 import RobotType2
from robot_module.robot_type3 import RobotType3

CONFIG_FILE = os.getenv("ROS_CONFIG_FILE", "config/config1.yaml")

def load_robot_config():
    """Loads the robot node type from the configuration file."""
    try:
        with open(CONFIG_FILE, "r") as f:
            config = yaml.safe_load(f)
        for node in config["nodes"]:
            if node["module"] == "robot_module":
                return node["name"]
    except Exception as e:
        print(f"Error loading config file: {e}")
    return None

def main():
    """Initializes and runs the selected robot node."""
    rclpy.init()

    # Load the default robot type from config if no argument is provided
    robot_type = sys.argv[1] if len(sys.argv) > 1 else load_robot_config()

    if not robot_type:
        print("Error: No robot type provided and no valid configuration found.")
        return
    
    # Robot node mapping
    robot_nodes = {
        "robot_type1": RobotType1,
        "robot_type2": RobotType2,
        "robot_type3": RobotType3
    }

    if robot_type not in robot_nodes:
        print(f"Invalid robot type: {robot_type}. Choose from {list(robot_nodes.keys())}")
        return

    try:
        node = robot_nodes[robot_type]()
        node.get_logger().info(f"Starting {robot_type}")
        rclpy.spin(node)
    except Exception as e:
        print(f"Error starting robot node: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
