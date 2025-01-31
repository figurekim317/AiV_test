import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from interface_module.msg import ProcessingResult

class ProcessorType3(Node):
    def __init__(self):
        super().__init__('processor_type3')

        # 토픽 및 퍼블리셔 설정
        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.process_image, 10)
        self.result_pub = self.create_publisher(ProcessingResult, 'processing/result3', 10)

        self.get_logger().info("Processor Type 3 initialized and listening for image data.")

    def process_image(self, msg):
        self.get_logger().info("Processor Type 3: Processing received image data.")

        # 경계값 연산 예제 (값이 200 이상이면 255, 아니면 0)
        thresholded_data = [255 if value >= 200 else 0 for value in msg.data]

        # 결과 메시지 생성
        result = ProcessingResult()
        result.header = Header()
        result.header.stamp = self.get_clock().now().to_msg()
        result.x = sum(thresholded_data) / len(thresholded_data) if thresholded_data else 0.0
        result.y = result.x * 1.2
        result.z = result.x * 0.8
        result.X = result.x - 0.3
        result.Y = result.x + 0.3
        result.Z = result.x - 1.2
        result.processing_status = "thresholded"

        # 결과 퍼블리시
        self.result_pub.publish(result)
        self.get_logger().info(f"Published thresholded processing result: {result}")

def main(args=None):
    rclpy.init(args=args)
    node = ProcessorType3()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
