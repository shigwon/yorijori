import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from keras_facenet import FaceNet
import cv2
import numpy as np

class FaceRecognitionNode(Node):

    def __init__(self):
        super().__init__('face_recognition_server_node')

        self.bridge = CvBridge()
        self.embedder = FaceNet()
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

        self.image1 = None
        self.image2 = None

        self.subscription1 = self.create_subscription(
            Image, 'face_image_db', self.image1_callback, 10)
        self.subscription2 = self.create_subscription(
            Image, 'face_image_query', self.image2_callback, 10)

        self.result_publisher = self.create_publisher(Bool, 'face_match_result', 10)
        self.waiting_for_images = True

    def crop_face_nearby(self, img, min_size=100):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)
        big_faces = [f for f in faces if f[2] >= min_size and f[3] >= min_size]
        if not big_faces:
            return None
        x, y, w, h = max(big_faces, key=lambda r: r[2] * r[3])
        return img[y:y+h, x:x+w]

    def image1_callback(self, msg):
        if not self.waiting_for_images:
            return
        self.image1 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.get_logger().info("DB 이미지 수신됨")
        self.try_compare()

    def image2_callback(self, msg):
        if not self.waiting_for_images:
            return
        self.image2 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.get_logger().info("Query 이미지 수신됨")
        self.try_compare()

    def try_compare(self):
        if self.image1 is None or self.image2 is None:
            return

        self.waiting_for_images = False

        face1 = self.crop_face_nearby(self.image1, min_size=120)
        face2 = self.crop_face_nearby(self.image2, min_size=120)

        if face1 is None or face2 is None:
            self.get_logger().warn("두 이미지 중 얼굴이 감지되지 않음.")
            self.publish_result(False)
        else:
            emb1 = self.embedder.embeddings([face1])[0]
            emb2 = self.embedder.embeddings([face2])[0]
            dist = np.linalg.norm(emb1 - emb2)

            match = bool(dist < 0.6)
            self.get_logger().info(f'유사도 거리: {dist:.4f} → {"일치" if match else "불일치"}')
            self.publish_result(match)

        # 상태 초기화
        self.image1 = None
        self.image2 = None
        self.waiting_for_images = True

    def publish_result(self, result_bool):
        result = Bool()
        result.data = result_bool
        self.result_publisher.publish(result)
        self.get_logger().info(f'결과 전송: {"일치" if result_bool else "불일치"}')


def main(args=None):
    rclpy.init(args=args)
    node = FaceRecognitionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
