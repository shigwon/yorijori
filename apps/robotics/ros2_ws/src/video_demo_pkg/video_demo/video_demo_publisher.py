import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import os

VIDEO_PATH = '/home/c102/S13P11C102/apps/robotics/media/video.mp4'
IMAGE_TOPIC_NAME = '/camera/image_raw/demo'

class DemoVideoPublisherNode(Node):
    def __init__(self):
        super().__init__('video_demo_publisher_node')
        
        self.get_logger().info(f"Using video file: {VIDEO_PATH}")
        self.get_logger().info(f"Publishing to topic: {IMAGE_TOPIC_NAME}")

        # 비디오 파일 경로가 유효한지 확인
        if not os.path.exists(VIDEO_PATH):
            self.get_logger().error(f"Video file not found at: {VIDEO_PATH}")
            raise FileNotFoundError(f"Video file not found at: {VIDEO_PATH}")

        # OpenCV 비디오 캡처 객체 생성
        self.cap = cv2.VideoCapture(VIDEO_PATH)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open video file.")
            raise IOError("Could not open video file.")

        # CvBridge는 OpenCV 이미지와 ROS 메시지 간의 변환을 담당합니다.
        self.br = CvBridge()
        
        # 이미지 토픽 퍼블리셔 생성
        self.publisher_ = self.create_publisher(Image, IMAGE_TOPIC_NAME, 10)

        # 동영상의 FPS(초당 프레임 수)를 가져와서 타이머 주기를 설정
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        if fps == 0:
            self.get_logger().warn("Could not get video FPS. Defaulting to 10 FPS.")
            fps = 10
        
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(f"Timer set to {timer_period:.4f} seconds (based on video FPS: {fps:.2f}).")

    def timer_callback(self):
        # 동영상에서 프레임 읽기
        ret, frame = self.cap.read()

        if ret:
            # OpenCV 이미지를 ROS 2 Image 메시지로 변환
            ros_image_msg = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
            
            # 메시지 발행
            self.publisher_.publish(ros_image_msg)
        else:
            # 동영상의 끝에 도달하면
            self.get_logger().info("End of video file. Stopping publisher.")
            self.cap.release()
            self.timer.cancel()
            self.destroy_node()

    def destroy_node(self):
        self.get_logger().info('Shutting down video publisher node...')
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = DemoVideoPublisherNode()
        rclpy.spin(node)
    except (FileNotFoundError, IOError) as e:
        rclpy.get_logger().error(str(e))
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()