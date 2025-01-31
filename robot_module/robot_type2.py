import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from interface_module.action import ExecuteMotion
from rclpy.action import ActionServer
import time

class RobotType2(Node):
    def __init__(self):
        super().__init__('robot_type2')

        # Action 서버 설정 (MainNode로부터 동작 명령 수신)
        self.action_server = ActionServer(self, ExecuteMotion, 'robot/execute_motion2', self.execute_motion_callback)

        # 상태 퍼블리셔 설정 (로봇 상태 전송)
        self.status_publisher = self.create_publisher(String, 'robot/status2', 10)

        # 센서 데이터 구독 (로봇이 환경을 인식하고 반응)
        self.sensor_subscriber = self.create_subscription(Image, 'sensor/data', self.sensor_callback, 10)

        self.get_logger().info("Robot Type 2 initialized with sensor feedback.")

    def execute_motion_callback(self, goal_handle):
        """로봇이 받은 모션 실행 요청을 처리하는 콜백 함수"""
        motion_type = goal_handle.request.motion_type
        self.get_logger().info(f"Executing motion: {motion_type} with sensor feedback.")

        # 동작 진행률 피드백
        feedback_msg = ExecuteMotion.Feedback()
        for progress in range(0, 101, 25):
            feedback_msg.progress = progress
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f"Motion Progress: {progress}%")
            time.sleep(1)  # 동작 진행을 시뮬레이션

        # 실행 완료 후 결과 전송
        goal_handle.succeed()
        result = ExecuteMotion.Result()
        result.success = True
        result.result_message = f"Motion '{motion_type}' completed successfully with sensor adaptation."

        # 상태 업데이트
        self.publish_status("motion_completed")

        return result

    def sensor_callback(self, msg):
        """센서 데이터를 수신하고 처리"""
        self.get_logger().info("Received sensor data. Adjusting motion parameters.")

    def publish_status(self, status_message):
        """로봇 상태를 퍼블리시"""
        msg = String()
        msg.data = status_message
        self.status_publisher.publish(msg)
        self.get_logger().info(f"Published Robot Status: {status_message}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotType2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
