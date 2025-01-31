import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from interface_module.srv import ProcessImage
from interface_module.action import ExecuteMotion
from rclpy.action import ActionClient

class MainNode(Node):
    """MainNode: Orchestrates communication between camera, image processing, and robot modules."""

    def __init__(self):
        super().__init__('main_node')

        # Publisher to request image capture from the camera module
        self.camera_request_pub = self.create_publisher(String, 'camera/request', 10)

        # Subscribers to listen for robot status updates and camera images
        self.robot_status_sub = self.create_subscription(
            String, 'robot/status', self.robot_status_callback, 10)
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)

        # Service client for image processing
        self.process_image_client = self.create_client(ProcessImage, 'image_processing/process_image')

        # Action client for sending motion execution requests to the robot
        self.robot_motion_client = ActionClient(self, ExecuteMotion, 'robot/execute_motion')

        self.get_logger().info("Main Node Initialized and ready for operations.")

    def robot_status_callback(self, msg):
        """Handles robot status updates. If motion is completed, requests new image capture."""
        if msg.data == "motion_completed":
            self.get_logger().info("Robot completed motion. Requesting new capture.")
            self.request_capture()

    def request_capture(self):
        """Sends a capture request to the camera node."""
        msg = String()
        msg.data = "capture"
        self.camera_request_pub.publish(msg)
        self.get_logger().info("Image capture requested.")

    def image_callback(self, msg):
        """Handles received images and requests image processing."""
        self.get_logger().info("Received image from camera. Sending for processing.")

        # Ensure the image processing service is available before requesting
        if not self.process_image_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Image processing service unavailable.")
            return

        request = ProcessImage.Request()
        request.image_data = msg  # Pass the received image to the service request

        future = self.process_image_client.call_async(request)
        future.add_done_callback(self.process_image_response)

    def process_image_response(self, future):
        """Handles image processing service response and initiates robot motion if successful."""
        try:
            response = future.result()
            if response.processed:
                self.get_logger().info(f"Image processed successfully: {response.message}")
                self.execute_robot_motion("processed_motion")
            else:
                self.get_logger().warn(f"Image processing failed: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Error while processing image: {str(e)}")

    def execute_robot_motion(self, motion_type):
        """Sends a motion execution request to the robot via an action client."""
        if not self.robot_motion_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Robot motion action server unavailable.")
            return

        goal_msg = ExecuteMotion.Goal()
        goal_msg.motion_type = motion_type

        self.get_logger().info(f"Sending robot motion request: {motion_type}")
        future = self.robot_motion_client.send_goal_async(goal_msg, feedback_callback=self.motion_feedback_callback)
        future.add_done_callback(self.motion_result_callback)

    def motion_feedback_callback(self, feedback_msg):
        """Handles feedback from the robot motion execution."""
        self.get_logger().info(f"Robot motion progress: {feedback_msg.feedback.progress:.2f}%")

    def motion_result_callback(self, future):
        """Handles the final result of the robot motion execution."""
        try:
            result = future.result().result
            if result.success:
                self.get_logger().info(f"Robot motion completed successfully: {result.result_message}")
            else:
                self.get_logger().warn(f"Robot motion failed: {result.result_message}")
        except Exception as e:
            self.get_logger().error(f"Error in robot motion execution: {str(e)}")

def main(args=None):
    """Main function to initialize and run the ROS 2 node."""
    rclpy.init(args=args)
    node = MainNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
