#!/usr/bin/env python3
import json
import base64
from urllib.parse import unquote

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
from keras_facenet import FaceNet

class FaceDBMatcherNode(Node):
    def __init__(self):
        super().__init__('face_db_matcher_node')

        # 파라미터
        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        self.declare_parameter('threshold', 0.6)
        self.declare_parameter('min_face_size', 120)
        self.declare_parameter('frame_skip', 10)  # N프레임당 1회 처리

        self.camera_topic = self.get_parameter('camera_topic').value
        self.threshold = float(self.get_parameter('threshold').value)
        self.min_face = int(self.get_parameter('min_face_size').value)
        self.frame_skip = int(self.get_parameter('frame_skip').value)

        self.bridge = CvBridge()
        self.embedder = FaceNet()
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

        self.db = {}                   # orderId -> embedding
        self.active_order_id = None    # 현재 매칭 대상
        self._frame_count = 0

        # 토픽
        self.create_subscription(String, '/face_db/register', self.register_callback, 10)
        self.pub_bool = self.create_publisher(Bool, '/face_match_result', 10)
        self.pub_order = self.create_publisher(String, '/face_match_by_order', 10)

        self.cam_sub = None
        self.get_logger().info("FaceDBMatcherNode (stream matching) started.")

    # ---- 카메라 온/오프 ----
    def start_camera(self):
        if self.cam_sub is None:
            self.cam_sub = self.create_subscription(Image, self.camera_topic, self.on_image, 10)
            self.get_logger().info(f"Camera subscribed: {self.camera_topic}")

    def stop_camera(self):
        if self.cam_sub is not None:
            self.destroy_subscription(self.cam_sub)
            self.cam_sub = None
            self.get_logger().info("Camera unsubscribed")

    # ---- 유틸 ----
    def _decode_base64_to_bgr(self, s: str):
        if not isinstance(s, str) or len(s) == 0:
            return None
        s = unquote(s)
        if ';base64,' in s:
            s = s.split(';base64,', 1)[1]
        try:
            img_bytes = base64.b64decode(s, validate=False)
            arr = np.frombuffer(img_bytes, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            return img
        except Exception as e:
            self.get_logger().warning(f"Base64 decode/imdecode failed: {e}")
            return None

    def _detect_biggest_face(self, bgr):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)
        faces = [f for f in faces if f[2] >= self.min_face and f[3] >= self.min_face]
        if not faces:
            return None
        x, y, w, h = max(faces, key=lambda r: r[2] * r[3])
        return bgr[y:y+h, x:x+w]

    def _embed_face(self, face_bgr):
        rgb = cv2.cvtColor(face_bgr, cv2.COLOR_BGR2RGB)
        emb = self.embedder.embeddings([rgb])[0]
        return emb

    def _l2(self, a, b):
        return float(np.linalg.norm(a - b))

    # ---- 등록 콜백 ----
    def register_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
            order_id = str(payload.get("orderId", "")).strip()
            img_b64 = payload.get("image_base64", "")

            if not order_id:
                return

            bgr = self._decode_base64_to_bgr(img_b64)
            if bgr is None:
                self.get_logger().warning(f"Registration failed for {order_id}: invalid image")
                return

            face = self._detect_biggest_face(bgr)
            if face is None:
                self.get_logger().warning(f"Registration failed for {order_id}: no face")
                return

            emb = self._embed_face(face)
            self.db[order_id] = emb
            self.active_order_id = order_id
            self._frame_count = 0
            self.start_camera()

            self.get_logger().info(f"Registered orderId={order_id} (DB size={len(self.db)}), stream matching started.")
        except Exception as e:
            self.get_logger().error(f'register_callback error: {e}')

    # ---- 카메라 프레임 처리 ----
    def on_image(self, img_msg: Image):
        if not self.active_order_id or self.active_order_id not in self.db:
            return

        self._frame_count += 1
        if self._frame_count % max(1, self.frame_skip) != 0:
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return

        face = self._detect_biggest_face(bgr)
        if face is None:
            return

        try:
            q = self._embed_face(face)
            ref = self.db[self.active_order_id]
            dist = self._l2(q, ref)
            matched = dist < self.threshold
        except Exception as e:
            self.get_logger().warn(f"embedding error: {e}")
            return

        out = {"orderId": self.active_order_id, "matched": bool(matched), "distance": float(dist)}
        s = String(); s.data = json.dumps(out, ensure_ascii=False)
        self.pub_order.publish(s)

        b = Bool(); b.data = bool(matched)
        self.pub_bool.publish(b)

        if matched:
            self.get_logger().info(f"[MATCH] orderId={self.active_order_id} dist={dist:.4f} < {self.threshold}")
            # 한 번 매칭되면 스트림 중단 (다음 주문에서 다시 /face_db/register 올 것)
            self.stop_camera()
            self.active_order_id = None


def main(args=None):
    rclpy.init(args=args)
    node = FaceDBMatcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
