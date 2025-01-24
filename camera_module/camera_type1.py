import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import random

class CameraType1(Node):
    def __init__(self):
        super().__init__('camera_type1')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer = self.create_timer(1.0, self.capture_image)

    def capture_image(self):
        msg = Image()
        msg.data = bytes([random.randint(0, 255) for _ in range(100)])
        self.publisher_.publish(msg)
        self.get_logger().info("Captured and published an image.")

    def run(self):
        rclpy.spin(self)
