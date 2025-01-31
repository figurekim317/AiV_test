import rclpy
import sys
import yaml
import os
from image_processing_module.processor_type1 import ProcessorType1
from image_processing_module.processor_type2 import ProcessorType2
from image_processing_module.processor_type3 import ProcessorType3

CONFIG_FILE = "config/config1.yaml"

def load_processing_config():
    """설정 파일에서 이미지 프로세싱 노드의 타입을 동적으로 로드"""
    try:
        with open(CONFIG_FILE, "r") as f:
            config = yaml.safe_load(f)
        for node in config["nodes"]:
            if node["module"] == "image_processing_module":
                return node["name"]
    except Exception as e:
        print(f"Error loading config file: {e}")
    return None

def main():
    rclpy.init()

    # 설정 파일에서 기본 이미지 프로세싱 타입 로드 (명령줄 인수가 없을 경우)
    processor_type = sys.argv[1] if len(sys.argv) > 1 else load_processing_config()

    if not processor_type:
        print("Error: No processing type provided and no valid configuration found.")
        return
    
    # 프로세싱 노드 매핑
    processing_nodes = {
        "processor_type1": ProcessorType1,
        "processor_type2": ProcessorType2,
        "processor_type3": ProcessorType3
    }

    if processor_type not in processing_nodes:
        print(f"Invalid processor type: {processor_type}. Choose from {list(processing_nodes.keys())}")
        return

    try:
        node = processing_nodes[processor_type]()
        node.get_logger().info(f"Starting {processor_type}")
        rclpy.spin(node)
    except Exception as e:
        print(f"Error starting processing node: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
