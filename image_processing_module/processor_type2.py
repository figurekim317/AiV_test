import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ProcessorType2(Node):
    def __init__(self):
        super().__init__('processor_type2')
        self.subscription = self.create_subscription(
            Image, 'camera/image_raw', self.process_image, 10)

    def process_image(self, msg):
        self.get_logger().info("Processor Type 2: Processing received image data.")

    def run(self):
        rclpy.spin(self)
