#!/usr/bin/env python3
import json
import base64

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

        # ===== 설정 =====
        self.camera_topic = '/left/image_rect'
        self.threshold = 0.7     # L2 거리 임계값 (0.9~1.0 권장)
        self.min_face = 50         # px
        self.frame_skip = 10       # N프레임당 1회 처리

        # ===== 리소스 =====
        self.bridge = CvBridge()
        self.embedder = FaceNet()
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        )

        # ===== 상태 =====
        self.db = {}                   # orderId -> embedding
        self.active_order_id = None    # 현재 매칭 대상
        self._frame_count = 0
        self.cam_sub = None
        self.match_done = False        # 매칭 완료 여부

        # ===== 토픽 =====
        self.create_subscription(String, '/face_db/register', self.register_callback, 10)
        self.pub_bool = self.create_publisher(Bool, '/face_match_result', 10)
        self.pub_order = self.create_publisher(String, '/face_match_by_order', 10)

        self.get_logger().info("FaceDBMatcherNode (stream matching) started.")

    def start_camera(self):
        if self.cam_sub is None:
            self.cam_sub = self.create_subscription(Image, self.camera_topic, self.on_image, 10)
            self.get_logger().info(f"Camera subscribed: {self.camera_topic}")

    def stop_camera(self):
        if self.cam_sub is not None:
            try:
                self.destroy_subscription(self.cam_sub)
            except Exception:
                pass
            self.cam_sub = None
            self.get_logger().info("Camera unsubscribed")

    def _decode_base64_to_bgr(self, s: str):
        if not isinstance(s, str) or not s:
            return None
        try:
            if ';base64,' in s:
                s = s.split(';base64,', 1)[1]
            img_bytes = base64.b64decode(s, validate=False)
            arr = np.frombuffer(img_bytes, dtype=np.uint8)
            return cv2.imdecode(arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().warning(f"Base64 decode/imdecode failed: {e}")
            return None

    def _detect_biggest_face(self, bgr):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=4)
        if len(faces) == 0:
            return None
        faces = [f for f in faces if f[2] >= self.min_face and f[3] >= self.min_face]
        if not faces:
            return None
        x, y, w, h = max(faces, key=lambda r: r[2] * r[3])
        return bgr[y:y + h, x:x + w]

    def _embed_face(self, face_bgr):
        # FaceNet 입력 규격에 맞게 전처리 (160x160 RGB)
        rgb = cv2.cvtColor(face_bgr, cv2.COLOR_BGR2RGB)
        rgb = cv2.resize(rgb, (160, 160))
        emb = self.embedder.embeddings([rgb.astype(np.float32)])[0]
        return emb

    def _l2(self, a, b) -> float:
        return float(np.linalg.norm(a - b))

    def register_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"register_callback JSON parse error: {e}")
            return

        try:
            order_id = str(payload.get("orderId", "")).strip()
            img_b64 = payload.get("image_base64", "")
            if not order_id:
                self.get_logger().warning("register_callback: empty orderId")
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
            self.match_done = False
            self.start_camera()

            self.get_logger().info(
                f"Registered orderId={order_id} (DB size={len(self.db)}), stream matching started."
            )
        except Exception as e:
            self.get_logger().error(f'register_callback error: {e}')

    def on_image(self, img_msg: Image):
        if self.match_done:
            return

        aoid = self.active_order_id
        if not aoid or aoid not in self.db:
            return

        self._frame_count += 1
        if self._frame_count % max(1, self.frame_skip) != 0:
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warning(f"cv_bridge error: {e}")
            return

        face = self._detect_biggest_face(bgr)
        if face is None:
            return

        try:
            q = self._embed_face(face)
            ref = self.db[aoid]
            dist = self._l2(q, ref)
            matched = dist < self.threshold
        except Exception as e:
            self.get_logger().warning(f"embedding error: {e}")
            return

        self.get_logger().info(f"[CHECK] orderId={aoid} dist={dist:.4f} matched={matched}")

        out = {"orderId": aoid, "matched": bool(matched), "distance": float(dist)}
        s = String()
        s.data = json.dumps(out, ensure_ascii=False)
        self.pub_order.publish(s)

        b = Bool()
        b.data = bool(matched)
        self.pub_bool.publish(b)

        if matched:
            self.match_done = True           # 한 번만 동작
            self.active_order_id = None      # 상태 초기화
            self.stop_camera()               # 카메라 구독 해제
            self.get_logger().info(f"[MATCH SUCCESS] orderId={aoid} dist={dist:.4f}")

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
