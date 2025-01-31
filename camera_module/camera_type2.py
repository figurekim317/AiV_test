import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import random

class CameraType2(Node):
    """CameraType2 Node: Publishes image data upon receiving a capture request."""

    def __init__(self):
        super().__init__('camera_type2')

        # Publisher for image data
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)

        # Subscription to receive capture requests
        self.capture_sub = self.create_subscription(
            String, 'camera/request', self.capture_callback, 10)

        self.get_logger().info("CameraType2 initialized and ready to capture images.")

    def capture_callback(self, msg):
        """Callback function that triggers image capture upon receiving a request."""
        if msg.data == "capture":
            self.get_logger().info("Camera Type 2: Capture request received. Capturing image...")
            self.publish_image()

    def publish_image(self):
        """Generates an image with random data and publishes it."""
        msg = Image()

        # Set timestamp in the message header
        msg.header.stamp = self.get_clock().now().to_msg()

        # Generate random image data (dummy values for simulation)
        msg.data = bytes([random.randint(0, 255) for _ in range(100)])

        # Publish the image message
        self.image_pub.publish(msg)
        self.get_logger().info("Camera Type 2: Image captured and published successfully.")

def main(args=None):
    """Main function to initialize and run the ROS 2 node."""
    rclpy.init(args=args)
    node = CameraType2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
