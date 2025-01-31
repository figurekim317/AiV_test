import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from interface_module.msg import ProcessingResult

class ProcessorType2(Node):
    """ProcessorType2 Node: Applies a simple threshold-based filtering on image data."""

    def __init__(self):
        super().__init__('processor_type2')

        # Subscriber for image data from the camera module
        self.subscription = self.create_subscription(
            Image, 'camera/image_raw', self.process_image, 10)

        # Publisher for filtered processing results
        self.result_pub = self.create_publisher(ProcessingResult, 'processing/result2', 10)

        self.get_logger().info("Processor Type 2 initialized and listening for image data.")

    def process_image(self, msg):
        """Applies a simple thresholding filter to the received image data."""
        self.get_logger().info("Processor Type 2: Processing received image data.")

        # Ensure image data is not empty
        if not msg.data:
            self.get_logger().warn("Processor Type 2: Received empty image data, skipping processing.")
            return

        # Apply threshold filtering (keep values > 128, set others to 0)
        filtered_data = [value if value > 128 else 0 for value in msg.data]

        # Compute basic statistics
        avg_filtered_value = sum(filtered_data) / len(filtered_data) if filtered_data else 0.0

        # Generate processing result message
        result = ProcessingResult()
        result.header.stamp = self.get_clock().now().to_msg()
        result.x = avg_filtered_value
        result.y = avg_filtered_value + 1.0
        result.z = avg_filtered_value + 2.0
        result.X = avg_filtered_value - 0.5
        result.Y = avg_filtered_value - 1.5
        result.Z = avg_filtered_value - 2.5
        result.processing_status = "filtered"

        # Publish the result
        self.result_pub.publish(result)
        self.get_logger().info(f"Processor Type 2: Published filtered processing result - x: {result.x}")

def main(args=None):
    """Main function to initialize and run the ROS 2 node."""
    rclpy.init(args=args)
    node = ProcessorType2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
