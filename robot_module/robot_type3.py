import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from interface_module.action import ExecuteMotion
from rclpy.action import ActionServer
import time

class RobotType3(Node):
    def __init__(self):
        super().__init__('robot_type3')

        # Action 서버 설정 (MainNode로부터 동작 명령 수신)
        self.action_server = ActionServer(self, ExecuteMotion, 'robot/execute_motion3', self.execute_motion_callback)

        # 상태 퍼블리셔 설정 (로봇 상태 전송)
        self.status_publisher = self.create_publisher(String, 'robot/status3', 10)

        self.get_logger().info("Robot Type 3 initialized with multi-motion execution.")

    def execute_motion_callback(self, goal_handle):
        """로봇이 받은 모션 실행 요청을 처리하는 콜백 함수"""
        motions = goal_handle.request.motion_type.split(",")  # 여러 개의 모션 실행 가능
        self.get_logger().info(f"Executing multiple motions: {motions}")

        feedback_msg = ExecuteMotion.Feedback()
        for motion in motions:
            self.get_logger().info(f"Starting motion: {motion}")
            for progress in range(0, 101, 20):
                feedback_msg.progress = progress
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info(f"Motion {motion} Progress: {progress}%")
                time.sleep(1)  # 동작 진행을 시뮬레이션

        # 실행 완료 후 결과 전송
        goal_handle.succeed()
        result = ExecuteMotion.Result()
        result.success = True
        result.result_message = f"All motions '{motions}' completed successfully."

        # 상태 업데이트
        self.publish_status("motion_completed")

        return result

    def publish_status(self, status_message):
        """로봇 상태를 퍼블리시"""
        msg = String()
        msg.data = status_message
        self.status_publisher.publish(msg)
        self.get_logger().info(f"Published Robot Status: {status_message}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotType3()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
