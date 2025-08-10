#!/usr/bin/env python3
import os
import json
import time
import math
import base64
import mimetypes
from urllib.parse import urlparse
from urllib.request import urlopen, Request
from dataclasses import dataclass, field
from typing import List, Optional, Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32MultiArray
from geometry_msgs.msg import Point


def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2.0) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2.0) ** 2
    return 2 * R * math.asin(math.sqrt(a))


@dataclass
class Order:
    orderId: str
    code: str
    tel: str
    customerLatitude: float
    customerLongitude: float
    spaceNum: int
    faceImageUrl: str
    createdTs: float = field(default_factory=lambda: time.time())
    status: str = "픽업존대기중"      # 픽업존대기중 → 배달중 → 배달완료
    arrival_notified: bool = False   # 도착 이벤트(정지/얼굴등록) 1회만 수행
    face_path: str = ""              # 캐시된 파일 경로
    face_b64: str = ""               # 캐시된 base64(데이터 페이로드)


class OrdersStoreNode(Node):
    def __init__(self):
        super().__init__('orders_store_node')

        # ---- 파라미터 ----
        self.declare_parameter('max_orders', 3)
        self.declare_parameter('allow_overwrite', False)
        self.declare_parameter('arrive_radius_m', 5.0)
        self.declare_parameter('door_open_sec', 30.0)         # 문 열린 유지 시간
        self.declare_parameter('face_cache_dir', '/tmp/face_cache')
        self.declare_parameter('face_download_timeout', 5.0)
        self.declare_parameter('face_max_bytes', 3 * 1024 * 1024)

        self.max_orders = int(self.get_parameter('max_orders').value)
        self.allow_overwrite = bool(self.get_parameter('allow_overwrite').value)
        self.arrive_radius_m = float(self.get_parameter('arrive_radius_m').value)
        self.door_open_sec = float(self.get_parameter('door_open_sec').value)
        self.face_cache_dir = str(self.get_parameter('face_cache_dir').value)
        self.face_dl_timeout = float(self.get_parameter('face_download_timeout').value)
        self.face_max_bytes = int(self.get_parameter('face_max_bytes').value)
        os.makedirs(self.face_cache_dir, exist_ok=True)

        # ---- 상태 ----
        self.orders: List[Order] = []
        self._stored_once = False
        self.current_target_index: Optional[int] = None
        self.robot_lat: Optional[float] = None
        self.robot_lon: Optional[float] = None

        # ---- 구독 ----
        self.create_subscription(String, '/server/order_list', self.on_order_list, 10)
        self.create_subscription(Point, '/slam/position', self.location_callback, 10)
        self.create_subscription(String, '/face_match_by_order', self.on_face_match, 10)

        # ---- 퍼블리시 ----
        self.pub_robot_status = self.create_publisher(String, '/robot/status', 10)                 # 대기중/주행중
        self.pub_delivery_state = self.create_publisher(String, '/robot/delivery_state', 10)       # 픽업존대기중/배달중/배달완료
        self.pub_delivery_state_json = self.create_publisher(String, '/robot/delivery_state_json', 10)  # {"orderId","state"}
        self.pub_drive_enable = self.create_publisher(Bool, '/robot/drive_enable', 10)             # True=주행, False=정지
        self.pub_face_register = self.create_publisher(String, '/face_db/register', 10)            # {"orderId","image_base64"}
        self.pub_food_bay = self.create_publisher(Int32MultiArray, '/food_bay_cmd', 10)            # [bay, 1/0]

        # 베이별 자동 닫기 타이머
        self._bay_close_timers: Dict[int, rclpy.timer.Timer] = {}

        self.set_robot_status("대기중")
        self.get_logger().info('OrdersStoreNode ready (drive_enable + face cache + food bay).')

    # ========== 주문 수신 ==========
    def on_order_list(self, msg: String):
        if self._stored_once and not self.allow_overwrite:
            self.get_logger().info('Order list already stored; ignoring further messages.')
            return

        try:
            payload = json.loads(msg.data)
            arr = payload["orders"] if isinstance(payload, dict) and "orders" in payload else payload
            if not isinstance(arr, list):
                self.get_logger().warn('order_list must be list or {"orders":[...]}. Ignored.')
                return

            new_list: List[Order] = []
            for o in arr[: self.max_orders]:
                try:
                    ord_obj = Order(
                        orderId=str(o.get("orderId", "")),
                        code=str(o.get("code", "")),
                        tel=str(o.get("tel", "")),
                        customerLatitude=float(o.get("customerLatitude")),
                        customerLongitude=float(o.get("customerLongitude")),
                        spaceNum=int(o.get("spaceNum")),
                        faceImageUrl=str(o.get("faceImageUrl", ""))
                    )

                    # 얼굴 이미지 미리 캐시
                    if ord_obj.faceImageUrl:
                        try:
                            face_path, face_b64 = self._download_face_to_cache(ord_obj.orderId, ord_obj.faceImageUrl)
                            ord_obj.face_path = face_path or ""
                            ord_obj.face_b64 = face_b64 or ""
                            if ord_obj.face_path:
                                self.get_logger().info(f"[FaceCache] {ord_obj.orderId} -> {ord_obj.face_path}")
                        except Exception as e:
                            self.get_logger().warn(f"[FaceCache] {ord_obj.orderId} download failed: {e}")

                    new_list.append(ord_obj)

                except Exception as e:
                    self.get_logger().warn(f"Skip invalid item: {e}")

            self.orders = new_list
            self._stored_once = True

            if self.orders:
                self.current_target_index = 0
                self.set_order_state(self.current_target_index, "배달중")
                self.set_robot_status("주행중")
                # 주문 수신 즉시 출발
                self.publish_drive_enable(True)
            else:
                self.current_target_index = None
                self.set_robot_status("대기중")
                self.publish_drive_enable(False)

            self.get_logger().info(f"Stored {len(self.orders)} orders. IDs={[o.orderId for o in self.orders]}")

        except Exception as e:
            self.get_logger().error(f'on_order_list error: {e}')

    # ========== 위치 갱신 ==========
    def location_callback(self, msg: Point):
        self.robot_lat = float(msg.x)
        self.robot_lon = float(msg.y)
        self.check_arrival()

    def check_arrival(self):
        if self.current_target_index is None or not self.orders:
            return
        if self.current_target_index >= len(self.orders):
            return
        if self.robot_lat is None or self.robot_lon is None:
            return

        target = self.orders[self.current_target_index]
        if target.status in ("픽업존대기중", "배달완료"):
            return

        try:
            dist = haversine_m(self.robot_lat, self.robot_lon, target.customerLatitude, target.customerLongitude)
        except Exception as e:
            self.get_logger().warn(f"distance calc failed: {e}")
            return

        if dist <= self.arrive_radius_m:
            if target.status != "픽업존대기중":
                self.set_order_state(self.current_target_index, "픽업존대기중")
                self.get_logger().info(f"[Arrived] orderId={target.orderId}, bay={target.spaceNum}, dist={dist:.2f} m")

            if not target.arrival_notified:
                # 도착 → 차량 정지
                self.publish_drive_enable(False)

                # 얼굴 DB 등록 (캐시된 base64 우선)
                img_b64 = target.face_b64
                if not img_b64 and target.faceImageUrl:
                    try:
                        _, img_b64_retry = self._download_face_to_cache(target.orderId, target.faceImageUrl)
                        img_b64 = img_b64_retry or ""
                        target.face_b64 = img_b64
                    except Exception as e:
                        self.get_logger().warn(f"[FaceCache] retry failed ({target.orderId}): {e}")

                if img_b64:
                    reg = String()
                    reg.data = json.dumps({"orderId": target.orderId, "image_base64": img_b64}, ensure_ascii=False)
                    self.pub_face_register.publish(reg)
                else:
                    self.get_logger().warn(f"[FaceDB] No face image available for orderId={target.orderId}")

                target.arrival_notified = True
        else:
            if target.status != "배달중":
                self.set_order_state(self.current_target_index, "배달중")

    # ========== 얼굴 매칭 결과 ==========
    def on_face_match(self, msg: String):
        try:
            data = json.loads(msg.data)
            order_id = str(data.get("orderId", ""))
            matched = bool(data.get("matched", False))
        except Exception as e:
            self.get_logger().warn(f"face_match parse error: {e}")
            return

        if self.current_target_index is None or not self.orders:
            return
        if self.current_target_index >= len(self.orders):
            return

        target = self.orders[self.current_target_index]
        if order_id != target.orderId:
            return

        if target.status == "픽업존대기중" and matched:
            # 음식칸 열기 → 일정 시간 뒤 닫기
            self.open_food_bay_then_close(target.spaceNum, self.door_open_sec)

            # 주문 완료 → 다음 주문으로
            self.set_order_state(self.current_target_index, "배달완료")
            self.get_logger().info(f"[Completed] orderId={target.orderId}")

            self.current_target_index += 1
            if self.current_target_index is not None and self.current_target_index < len(self.orders):
                nxt = self.orders[self.current_target_index]
                self.set_order_state(self.current_target_index, "배달중")
                self.set_robot_status("주행중")
                # 다음 주문 시작: 출발
                self.publish_drive_enable(True)
                self.get_logger().info(f"Next target -> orderId={nxt.orderId}, bay={nxt.spaceNum}")
            else:
                self.current_target_index = None
                self.set_robot_status("대기중")
                self.publish_drive_enable(False)
                self.get_logger().info("All deliveries completed. Robot is now 대기중.")

    # ========== Food bay helpers ==========
    def publish_food_bay(self, bay: int, state: int):
        msg = Int32MultiArray()
        msg.data = [int(bay), int(state)]  # 1=open, 0=close
        self.pub_food_bay.publish(msg)
        self.get_logger().info(f"[FoodBay] bay={bay} -> {'OPEN' if state==1 else 'CLOSE'}")

    def open_food_bay_then_close(self, bay: int, duration_sec: float):
        # 기존 타이머 있으면 취소
        old = self._bay_close_timers.get(bay)
        if old is not None:
            try:
                old.cancel()
            except Exception:
                pass
            self._bay_close_timers.pop(bay, None)

        # 즉시 열기
        self.publish_food_bay(bay, 1)

        # duration 후 닫기
        def cb():
            try:
                self.publish_food_bay(bay, 0)
            finally:
                t = self._bay_close_timers.pop(bay, None)
                if t is not None:
                    try:
                        t.cancel()
                    except Exception:
                        pass

        timer = self.create_timer(float(duration_sec), cb)
        self._bay_close_timers[bay] = timer
        self.get_logger().info(f"Scheduled auto-close for bay {bay} in {duration_sec:.1f}s")

    # ========== 상태 퍼블리시 ==========
    def set_robot_status(self, s: str):
        out = String(); out.data = s
        self.pub_robot_status.publish(out)

    def set_order_state(self, idx: int, state: str):
        self.orders[idx].status = state
        # 간단 문자열
        out = String(); out.data = state
        self.pub_delivery_state.publish(out)
        # 식별 포함 JSON
        j = String()
        j.data = json.dumps({"orderId": self.orders[idx].orderId, "state": state}, ensure_ascii=False)
        self.pub_delivery_state_json.publish(j)

    def publish_drive_enable(self, enable: bool):
        m = Bool(); m.data = bool(enable)
        self.pub_drive_enable.publish(m)

    # ========== 얼굴 캐시 헬퍼 ==========
    def _download_face_to_cache(self, order_id: str, url_or_data: str):
        """
        url_or_data:
         - data URL (data:image/...;base64,....)
         - 순수 base64 문자열
         - http/https URL
        return: (face_path, face_b64)
        """
        s = (url_or_data or "").strip()

        # data URL
        if s.startswith('data:') and ';base64,' in s:
            b64 = s.split(';base64,', 1)[1]
            data = base64.b64decode(b64, validate=False)
            ext = self._guess_ext_from_data_url(s)
            face_path = os.path.join(self.face_cache_dir, f"{order_id}{ext}")
            with open(face_path, 'wb') as f:
                f.write(data)
            return face_path, b64

        # 순수 base64일 가능성
        if not s.lower().startswith('http'):
            try:
                data = base64.b64decode(s, validate=False)
                face_path = os.path.join(self.face_cache_dir, f"{order_id}.jpg")
                with open(face_path, 'wb') as f:
                    f.write(data)
                b64 = base64.b64encode(data).decode('ascii')
                return face_path, b64
            except Exception:
                raise ValueError("faceImageUrl is neither http(s) nor base64")

        # http/https URL
        parsed = urlparse(s)
        if parsed.scheme not in ('http', 'https'):
            raise ValueError(f"unsupported scheme: {parsed.scheme}")

        req = Request(s, headers={'User-Agent': 'FaceFetcher/1.0'})
        with urlopen(req, timeout=self.face_dl_timeout) as resp:
            content_len = resp.getheader('Content-Length')
            if content_len is not None:
                try:
                    if int(content_len) > self.face_max_bytes:
                        raise ValueError(f"image too large: {content_len} > {self.face_max_bytes}")
                except Exception:
                    pass

            data = resp.read(self.face_max_bytes + 1)
            if len(data) > self.face_max_bytes:
                raise ValueError(f"image too large (>{self.face_max_bytes} bytes)")

            ctype = (resp.getheader('Content-Type') or '').split(';')[0].strip()
            ext = mimetypes.guess_extension(ctype) or '.jpg'
            face_path = os.path.join(self.face_cache_dir, f"{order_id}{ext}")
            with open(face_path, 'wb') as f:
                f.write(data)

            b64 = base64.b64encode(data).decode('ascii')
            return face_path, b64

    def _guess_ext_from_data_url(self, data_url: str) -> str:
        try:
            head = data_url.split(',', 1)[0]
            ctype = head.split(':', 1)[1].split(';', 1)[0]
            return mimetypes.guess_extension(ctype) or '.jpg'
        except Exception:
            return '.jpg'


def main(args=None):
    rclpy.init(args=args)
    node = OrdersStoreNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
