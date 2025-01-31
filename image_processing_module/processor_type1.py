import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from interface_module.msg import ProcessingResult

class ProcessorType1(Node):
    """ProcessorType1 Node: Subscribes to image data and publishes processed results."""

    def __init__(self):
        super().__init__('processor_type1')

        # Subscriber for image data from the camera module
        self.subscription = self.create_subscription(
            Image, 'camera/image_raw', self.process_image, 10)

        # Publisher for processing results to the robot module
        self.result_pub = self.create_publisher(ProcessingResult, 'processing/result', 10)

        self.get_logger().info("Processor Type 1 initialized and listening for image data.")

    def process_image(self, msg):
        """Processes received image data and publishes the results."""
        self.get_logger().info("Processor Type 1: Processing received image data.")

        # Ensure the image data is not empty before processing
        if not msg.data:
            self.get_logger().warn("Processor Type 1: Received empty image data, skipping processing.")
            return

        # Compute a simple metric (e.g., average pixel value as a basic feature)
        avg_pixel_value = sum(msg.data) / len(msg.data) if msg.data else 0.0

        # Generate a processing result message
        result = ProcessingResult()
        result.header.stamp = self.get_clock().now().to_msg()  # Add timestamp
        result.x = avg_pixel_value
        result.y = avg_pixel_value + 1.0
        result.z = avg_pixel_value + 2.0
        result.X = avg_pixel_value - 0.5
        result.Y = avg_pixel_value - 1.5
        result.Z = avg_pixel_value - 2.5
        result.processing_status = "completed"

        # Publish the processing result
        self.result_pub.publish(result)
        self.get_logger().info(f"Processor Type 1: Published processing result - x: {result.x}, y: {result.y}, z: {result.z}")

def main(args=None):
    """Main function to initialize and run the ROS 2 node."""
    rclpy.init(args=args)
    node = ProcessorType1()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
