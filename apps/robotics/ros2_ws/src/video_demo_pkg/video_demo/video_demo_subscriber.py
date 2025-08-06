import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2

# 이미지 토픽 이름은 발행하는 노드와 동일해야 합니다.
IMAGE_TOPIC_NAME = '/camera/image_raw'

class DemoVideoSubscriber(Node):
    def __init__(self):
        super().__init__('video_demo_subscriber_node')
        
        self.get_logger().info(f"Subscribing to topic: {IMAGE_TOPIC_NAME}")
        
        # CvBridge 인스턴스 생성
        self.br = CvBridge()
        
        # 이미지 토픽 구독
        self.subscription = self.create_subscription(
            Image,
            IMAGE_TOPIC_NAME,
            self.image_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        
        self.get_logger().info("Demo video subscriber node initialized.")
        
    def image_callback(self, data):
        """
        Image 메시지가 수신될 때마다 호출되는 콜백 함수
        """
        # self.get_logger().info('Receiving video frame')
        
        try:
            # ROS Image 메시지를 OpenCV 이미지(numpy 배열)로 변환
            current_frame = self.br.imgmsg_to_cv2(data)
            
            # OpenCV 창에 이미지 표시
            cv2.imshow("Demo Video Viewer", current_frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image or show frame: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DemoVideoSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down demo video subscriber node...')
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()