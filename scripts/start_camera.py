import rclpy
import sys
from camera_module.camera_type1 import CameraType1
from camera_module.camera_type2 import CameraType2
from camera_module.camera_type3 import CameraType3

def main():
    if len(sys.argv) < 2:
        print("Usage: python scripts/start_camera.py <camera_type>")
        return
    
    camera_type = sys.argv[1]
    rclpy.init()

    if camera_type == "camera_type1":
        node = CameraType1()
    elif camera_type == "camera_type2":
        node = CameraType2()
    elif camera_type == "camera_type3":
        node = CameraType3()
    else:
        print("Invalid camera type")
        return
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
