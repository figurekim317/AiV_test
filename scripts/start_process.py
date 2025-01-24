import rclpy
import sys
from image_processing_module.processor_type1 import ProcessorType1
from image_processing_module.processor_type2 import ProcessorType2
from image_processing_module.processor_type3 import ProcessorType3

def main():
    if len(sys.argv) < 2:
        print("Usage: python scripts/start_processing.py <processor_type>")
        return
    
    processor_type = sys.argv[1]
    rclpy.init()

    if processor_type == "processor_type1":
        node = ProcessorType1()
    elif processor_type == "processor_type2":
        node = ProcessorType2()
    elif processor_type == "processor_type3":
        node = ProcessorType3()
    else:
        print("Invalid processor type")
        return

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
