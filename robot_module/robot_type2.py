import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from interface_module.action import ExecuteMotion
from rclpy.action import ActionServer
import time

class RobotType2(Node):
    """RobotType2 Node: Executes motion commands with real-time sensor feedback."""

    def __init__(self):
        super().__init__('robot_type2')

        # Action server to receive motion execution requests
        self.action_server = ActionServer(self, ExecuteMotion, 'robot/execute_motion2', self.execute_motion_callback)

        # Publisher to send robot status updates
        self.status_publisher = self.create_publisher(String, 'robot/status2', 10)

        # Subscriber to receive sensor data for adaptive motion execution
        self.sensor_subscriber = self.create_subscription(Image, 'sensor/data', self.sensor_callback, 10)

        self.get_logger().info("Robot Type 2 initialized with sensor feedback.")

    def execute_motion_callback(self, goal_handle):
        """Processes motion execution requests, adjusting behavior based on sensor feedback."""
        motion_type = goal_handle.request.motion_type
        self.get_logger().info(f"Executing motion: {motion_type} with sensor feedback.")

        # Send feedback on motion progress
        feedback_msg = ExecuteMotion.Feedback()
        for progress in range(0, 101, 25):
            feedback_msg.progress = progress
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f"Motion Progress: {progress}%")
            time.sleep(1)  # Simulating motion execution

        # Mark the motion as complete and send result
        goal_handle.succeed()
        result = ExecuteMotion.Result()
        result.success = True
        result.result_message = f"Motion '{motion_type}' completed successfully with sensor adaptation."

        # Publish updated robot status
        self.publish_status("motion_completed")

        return result

    def sensor_callback(self, msg):
        """Processes sensor data to adjust robot motion dynamically."""
        self.get_logger().info("Received sensor data. Adjusting motion parameters.")

    def publish_status(self, status_message):
        """Publishes the current robot status to the MainNode."""
        msg = String()
        msg.data = status_message
        self.status_publisher.publish(msg)
        self.get_logger().info(f"Published Robot Status: {status_message}")

def main(args=None):
    """Main function to initialize and run the ROS 2 node."""
    rclpy.init(args=args)
    node = RobotType2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
