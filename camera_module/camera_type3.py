import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import random

class CameraType3(Node):
    """CameraType3 Node: Periodically publishes random image data."""

    def __init__(self):
        super().__init__('camera_type3')

        # Publisher for image data
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)

        # Timer to capture and publish an image every second
        self.timer = self.create_timer(1.0, self.capture_image)

        self.get_logger().info("CameraType3 initialized and publishing images every second.")

    def capture_image(self):
        """Generates an image with random data and publishes it."""
        msg = Image()

        # Set timestamp in the message header
        msg.header.stamp = self.get_clock().now().to_msg()

        # Generate random image data (dummy values for simulation)
        msg.data = bytes([random.randint(0, 255) for _ in range(100)])

        # Publish the image message
        self.image_pub.publish(msg)
        self.get_logger().info("Camera Type 3: Image captured and published successfully.")

def main(args=None):
    """Main function to initialize and run the ROS 2 node."""
    rclpy.init(args=args)
    node = CameraType3()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
