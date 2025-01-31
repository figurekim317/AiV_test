import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from interface_module.msg import ProcessingResult

class ProcessorType2(Node):
    def __init__(self):
        super().__init__('processor_type2')

        # 토픽 및 퍼블리셔 설정
        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.process_image, 10)
        self.result_pub = self.create_publisher(ProcessingResult, 'processing/result2', 10)

        self.get_logger().info("Processor Type 2 initialized and listening for image data.")

    def process_image(self, msg):
        self.get_logger().info("Processor Type 2: Processing received image data.")

        # 데이터 필터링 예제 (단순 임계값 적용)
        filtered_data = [value if value > 128 else 0 for value in msg.data]

        # 결과 메시지 생성
        result = ProcessingResult()
        result.header = Header()
        result.header.stamp = self.get_clock().now().to_msg()
        result.x = sum(filtered_data) / len(filtered_data) if filtered_data else 0.0
        result.y = result.x + 1.0
        result.z = result.x + 2.0
        result.X = result.x - 0.5
        result.Y = result.x - 1.5
        result.Z = result.x - 2.5
        result.processing_status = "filtered"

        # 결과 퍼블리시
        self.result_pub.publish(result)
        self.get_logger().info(f"Published filtered processing result: {result}")

def main(args=None):
    rclpy.init(args=args)
    node = ProcessorType2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
