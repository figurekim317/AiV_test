import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import random

class CameraType3(Node):
    def __init__(self):
        super().__init__('camera_type3')
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.capture_sub = self.create_subscription(String, 'camera/request', self.capture_callback, 10)

    def capture_callback(self, msg):
        if msg.data == "capture":
            self.get_logger().info("Camera Type 3: Capturing Image.")
            self.publish_image()

    def publish_image(self):
        msg = Image()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = list([random.randint(0, 255) for _ in range(100)])
        self.image_pub.publish(msg)
        self.get_logger().info("Camera Type 3: Published image.")

def main(args=None):
    rclpy.init(args=args)
    node = CameraType3()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
