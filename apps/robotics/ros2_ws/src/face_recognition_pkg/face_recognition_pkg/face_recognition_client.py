import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import threading

class FaceRecognitionClient(Node):
    def __init__(self):
        super().__init__('face_recognition_client_node')

        self.pub_db = self.create_publisher(Image, 'face_image_db', 10)
        self.pub_query = self.create_publisher(Image, 'face_image_query', 10)
        self.sub_result = self.create_subscription(Bool, 'face_match_result', self.result_callback, 10)

        self.bridge = CvBridge()

        self.db_image_path = '/home/c102/S13P11C102/apps/robotics/ros2_ws/src/face_recognition_pkg/images/face1.jpg'
        self.query_image_path = '/home/c102/S13P11C102/apps/robotics/ros2_ws/src/face_recognition_pkg/images/face5.jpg'

        self.db_image = cv2.imread(self.db_image_path)
        self.query_image = cv2.imread(self.query_image_path)

        if self.db_image is None or self.query_image is None:
            self.get_logger().error('이미지를 불러오는 데 실패했습니다.')
            return

        self.result_received = threading.Event()
        self.send_images()
        self._wait_time = 20
        self.timer = self.create_timer(self._wait_time, self.timeout_check)

    def send_images(self):
        msg_db = self.bridge.cv2_to_imgmsg(self.db_image, encoding='bgr8')
        msg_query = self.bridge.cv2_to_imgmsg(self.query_image, encoding='bgr8')

        self.pub_db.publish(msg_db)
        self.pub_query.publish(msg_query)
        self.get_logger().info('이미지를 전송했습니다.')

    def result_callback(self, msg):
        self.result_received.set()
        self.get_logger().info(f'서버로부터 결과를 받음: {"일치" if msg.data else "불일치"}')

    def timeout_check(self):
        if self.result_received.is_set():
            self.get_logger().info('이미 결과를 받았으므로 타이머 종료.')
            self.timer.cancel()
        else:
            self.get_logger().warn(self._wait_time + '초 내 결과를 받지 못함. 이미지를 재전송합니다.')
            self.send_images()

def main(args=None):
    rclpy.init(args=args)
    node = FaceRecognitionClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
