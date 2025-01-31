import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from interface_module.msg import ProcessingResult

class ProcessorType3(Node):
    """ProcessorType3 Node: Applies a binary thresholding operation to image data."""

    def __init__(self):
        super().__init__('processor_type3')

        # Subscriber for image data from the camera module
        self.subscription = self.create_subscription(
            Image, 'camera/image_raw', self.process_image, 10)

        # Publisher for thresholded processing results
        self.result_pub = self.create_publisher(ProcessingResult, 'processing/result3', 10)

        self.get_logger().info("Processor Type 3 initialized and listening for image data.")

    def process_image(self, msg):
        """Applies a binary thresholding operation to the received image data."""
        self.get_logger().info("Processor Type 3: Processing received image data.")

        # Ensure image data is not empty
        if not msg.data:
            self.get_logger().warn("Processor Type 3: Received empty image data, skipping processing.")
            return

        # Apply binary thresholding (values >= 200 set to 255, others set to 0)
        thresholded_data = [255 if value >= 200 else 0 for value in msg.data]

        # Compute basic statistics
        avg_thresholded_value = sum(thresholded_data) / len(thresholded_data) if thresholded_data else 0.0

        # Generate processing result message
        result = ProcessingResult()
        result.header.stamp = self.get_clock().now().to_msg()
        result.x = avg_thresholded_value
        result.y = avg_thresholded_value * 1.2
        result.z = avg_thresholded_value * 0.8
        result.X = avg_thresholded_value - 0.3
        result.Y = avg_thresholded_value + 0.3
        result.Z = avg_thresholded_value - 1.2
        result.processing_status = "thresholded"

        # Publish the result
        self.result_pub.publish(result)
        self.get_logger().info(f"Processor Type 3: Published thresholded processing result - x: {result.x}")

def main(args=None):
    """Main function to initialize and run the ROS 2 node."""
    rclpy.init(args=args)
    node = ProcessorType3()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
