import rclpy
import sys
import yaml
import os
from camera_module.camera_type1 import CameraType1
from camera_module.camera_type2 import CameraType2
from camera_module.camera_type3 import CameraType3

CONFIG_FILE = os.getenv("ROS_CONFIG_FILE", "config/config1.yaml")

def load_camera_config():
    """Loads the camera node type from the configuration file."""
    try:
        with open(CONFIG_FILE, "r") as f:
            config = yaml.safe_load(f)
        for node in config["nodes"]:
            if node["module"] == "camera_module":
                return node["name"]
    except Exception as e:
        print(f"Error loading config file: {e}")
    return None

def main():
    """Initializes and runs the selected camera node."""
    rclpy.init()

    # Load the default camera type from config if no argument is provided
    camera_type = sys.argv[1] if len(sys.argv) > 1 else load_camera_config()

    if not camera_type:
        print("Error: No camera type provided and no valid configuration found.")
        return
    
    # Camera node mapping
    camera_nodes = {
        "camera_type1": CameraType1,
        "camera_type2": CameraType2,
        "camera_type3": CameraType3
    }

    if camera_type not in camera_nodes:
        print(f"Invalid camera type: {camera_type}. Choose from {list(camera_nodes.keys())}")
        return

    try:
        node = camera_nodes[camera_type]()
        node.get_logger().info(f"Starting {camera_type}")
        rclpy.spin(node)
    except Exception as e:
        print(f"Error starting camera node: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
