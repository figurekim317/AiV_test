import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobotType1(Node):
    def __init__(self):
        super().__init__('robot_type1')
        self.subscription = self.create_subscription(
            String, 'process/output', self.execute_motion, 10)

    def execute_motion(self, msg):
        self.get_logger().info(f"Executing motion based on input: {msg.data}")

    def run(self):
        rclpy.spin(self)
