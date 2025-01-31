import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from interface_module.srv import ProcessImage
from interface_module.action import ExecuteMotion
from rclpy.action import ActionClient

class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')

        # Publisher & Subscriber 설정
        self.camera_request_pub = self.create_publisher(String, 'camera/request', 10)
        self.robot_status_sub = self.create_subscription(String, 'robot/status', self.robot_status_callback, 10)
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)

        # Service 및 Action 클라이언트 설정
        self.process_image_client = self.create_client(ProcessImage, 'image_processing/process_image')
        self.robot_motion_client = ActionClient(self, ExecuteMotion, 'robot/execute_motion')

        self.get_logger().info("Main Node Initialized.")

    def robot_status_callback(self, msg):
        """로봇이 작업을 완료하면 새로운 이미지 캡처 요청"""
        if msg.data == "motion_completed":
            self.get_logger().info("Robot completed motion. Requesting new capture.")
            self.request_capture()

    def request_capture(self):
        """카메라 노드에 이미지 캡처 요청을 보냄"""
        msg = String()
        msg.data = "capture"
        self.camera_request_pub.publish(msg)
        self.get_logger().info("Requested image capture.")

    def image_callback(self, msg):
        """카메라에서 이미지를 수신하면 이미지 프로세싱 요청"""
        self.get_logger().info("Received image from camera. Sending for processing.")
        
        # 서비스 요청이 가능할 때까지 대기
        if not self.process_image_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Image processing service unavailable.")
            return

        request = ProcessImage.Request()
        request.image_data = msg

        future = self.process_image_client.call_async(request)
        future.add_done_callback(self.process_image_response)

    def process_image_response(self, future):
        """이미지 프로세싱 결과를 받고 로봇에 작업 요청"""
        try:
            response = future.result()
            if response.processed:
                self.get_logger().info(f"Image processed successfully: {response}")
                self.execute_robot_motion("processed_motion")
            else:
                self.get_logger().warn("Image processing failed.")
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {str(e)}")

    def execute_robot_motion(self, motion_type):
        """로봇에게 특정 모션 실행 요청"""
        if not self.robot_motion_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Robot motion action server unavailable.")
            return

        goal_msg = ExecuteMotion.Goal()
        goal_msg.motion_type = motion_type

        self.get_logger().info(f"Sending robot motion request: {motion_type}")
        self.robot_motion_client.send_goal_async(goal_msg, feedback_callback=self.motion_feedback_callback)

    def motion_feedback_callback(self, feedback_msg):
        """로봇 모션 실행 중 피드백을 수신하여 로그 출력"""
        self.get_logger().info(f"Robot motion progress: {feedback_msg.feedback.progress}%")

def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
