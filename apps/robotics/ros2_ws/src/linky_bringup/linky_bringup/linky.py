#!/usr/bin/env python3
import os
import json
import time
import math
import base64
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Tuple

import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point

# ===== 고정 설정(필요시 여기서만 수정) =====
MAX_ORDERS        = 3
ALLOW_OVERWRITE   = False
ARRIVE_RADIUS_M   = 5.0
DOOR_OPEN_SEC     = 5.0
FACE_CACHE_DIR    = "/tmp/face_cache"
FACE_MAX_BYTES    = 3 * 1024 * 1024

# ===== 공용 유틸 =====
def haversine_m(lat1, lon1, lat2, lon2) -> float:
    R = 6371000.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2.0) ** 2
         + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2))
         * math.sin(dlon / 2.0) ** 2)
    return 2 * R * math.asin(math.sqrt(a))

def _to_float(x, default=None) -> Optional[float]:
    try:
        if x is None:
            return default
        return float(x)
    except Exception:
        return default

def _to_int(x, default=None) -> Optional[int]:
    try:
        if x is None:
            return default
        return int(x)
    except Exception:
        return default

def _extract_pure_b64(s: str) -> str:
    """'data:image/...;base64,....' 또는 순수 base64 → 순수 base64만 반환"""
    if not isinstance(s, str):
        return ""
    s = s.strip()
    if not s:
        return ""
    if ';base64,' in s:
        s = s.split(';base64,', 1)[1]
    return s

def _validate_b64_and_optionally_save(order_id: str,
                                      b64: str,
                                      max_bytes: int,
                                      cache_dir: str) -> Tuple[str, str]:
    """
    base64 디코드/용량 검증 후 (선택) 캐시에 저장.
    return: (normalized_b64, face_path or "")
    """
    try:
        raw = base64.b64decode(b64, validate=False)
    except Exception:
        raise ValueError("invalid base64 data")

    if len(raw) > max_bytes:
        raise ValueError(f"image too large ({len(raw)} > {max_bytes})")

    face_path = ""
    try:
        os.makedirs(cache_dir, exist_ok=True)
        face_path = os.path.join(cache_dir, f"{order_id}.jpg")
        with open(face_path, "wb") as f:
            f.write(raw)
    except Exception:
        face_path = ""  # 저장 실패는 치명적 아님

    normalized_b64 = base64.b64encode(raw).decode("ascii")
    return normalized_b64, face_path

# ===== 데이터 모델 =====
@dataclass
class Order:
    orderId: str
    code: str
    tel: str
    customerLatitude: float
    customerLongitude: float
    spaceNum: int
    createdTs: float = field(default_factory=lambda: time.time())
    status: str = "픽업존대기중"      # 픽업존대기중 → 배달중 → 배달완료
    arrival_notified: bool = False   # 도착 이벤트 1회 보장
    face_path: str = ""              # 캐시 파일 경로(옵션)
    face_b64: str = ""               # 서버에서 받은 base64(정규화 형태)

# ===== 노드 =====
class OrdersStoreNode(Node):
    def __init__(self):
        super().__init__('linky')

        # ---- 설정(상수 바인딩) ----
        self.max_orders       = int(MAX_ORDERS)
        self.allow_overwrite  = bool(ALLOW_OVERWRITE)
        self.arrive_radius_m  = float(ARRIVE_RADIUS_M)
        self.door_open_sec    = float(DOOR_OPEN_SEC)
        self.face_cache_dir   = str(FACE_CACHE_DIR)
        self.face_max_bytes   = int(FACE_MAX_BYTES)
        os.makedirs(self.face_cache_dir, exist_ok=True)

        # ---- 상태 ----
        self.orders: List[Order] = []
        self._stored_once = False
        self.current_target_index: Optional[int] = None
        self.robot_lat: Optional[float] = None
        self.robot_lon: Optional[float] = None

        # ---- 구독 ----
        self.create_subscription(String, '/server/order_list', self.on_order_list, 10)
        self.create_subscription(String, '/face_match_by_order', self.on_face_match, 10)

        # ---- 퍼블리시 ----
        self.pub_robot_status       = self.create_publisher(String, '/robot/status', 10)                 # 대기중/주행중
        self.pub_delivery_state     = self.create_publisher(String, '/robot/delivery_state', 10)         # 픽업존대기중/배달중/배달완료
        self.pub_delivery_state_json= self.create_publisher(String, '/robot/delivery_state_json', 10)    # {"orderId","state"}
        self.pub_drive_enable       = self.create_publisher(Bool,   '/robot/drive_enable', 10)           # True=주행, False=정지
        self.pub_face_register      = self.create_publisher(String, '/face_db/register', 10)             # {"orderId","image_base64"}
        self.pub_food_bay           = self.create_publisher(Int32MultiArray, '/food_bay_cmd', 10)        # [bay, OPEN/CLOSE]

        # 베이별 자동 닫기 타이머
        self._bay_close_timers: Dict[int, Timer] = {}

        self.set_robot_status("대기중")
        self.get_logger().info('OrdersStoreNode ready (drive_enable + base64 face cache + food bay).')

    # ========== 주문 수신 ==========
    def on_order_list(self, msg: String):
        if self._stored_once and not self.allow_overwrite:
            self.get_logger().info('Order list already stored; ignoring further messages.')
            return

        try:
            payload = json.loads(msg.data)
            arr = payload["orders"] if isinstance(payload, dict) and "orders" in payload else payload
            if not isinstance(arr, list):
                self.get_logger().warning('order_list must be list or {"orders":[...]}. Ignored.')
                return

            new_list: List[Order] = []
            for o in arr[: self.max_orders]:
                try:
                    lat = _to_float(o.get("customerLatitude"))
                    lon = _to_float(o.get("customerLongitude"))
                    bay = _to_int(o.get("spaceNum"))
                    if lat is None or lon is None or bay is None:
                        raise ValueError("customerLatitude/Longitude/spaceNum required")

                    ord_obj = Order(
                        orderId=str(o.get("orderId", "")),
                        code=str(o.get("code", "")),     # 주문 번호/코드
                        tel=str(o.get("tel", "")),
                        customerLatitude=lat,
                        customerLongitude=lon,
                        spaceNum=bay,
                    )

                    # === 서버 base64 파싱 ===
                    # 우선순위: faceImageBase64, face_base64
                    img_b64_raw = (o.get("faceImageBase64")
                                   or o.get("face_base64")
                                   or o.get("faceImageUrl")
                                   or "")
                    img_b64_raw = _extract_pure_b64(str(img_b64_raw))

                    if img_b64_raw:
                        try:
                            normalized_b64, face_path = _validate_b64_and_optionally_save(
                                ord_obj.orderId, img_b64_raw, self.face_max_bytes, self.face_cache_dir
                            )
                            ord_obj.face_b64 = normalized_b64
                            ord_obj.face_path = face_path or ""
                            if face_path:
                                self.get_logger().info(f"[FaceCache] {ord_obj.orderId} -> {face_path}")
                        except Exception as e:
                            self.get_logger().warning(f"[FaceCache] base64 invalid for {ord_obj.orderId}: {e}")
                    else:
                        self.get_logger().warning(f"[FaceCache] no base64 provided for {ord_obj.orderId}")

                    new_list.append(ord_obj)

                except Exception as e:
                    self.get_logger().warning(f"Skip invalid item: {e}")

            self.orders = new_list
            self._stored_once = True

            if self.orders:
                self.current_target_index = 0
                self.set_order_state(self.current_target_index, "배달중")
                self.set_robot_status("주행중")
                self.publish_drive_enable(True)   # 주문 수신 즉시 출발
            else:
                self.current_target_index = None
                self.set_robot_status("대기중")
                self.publish_drive_enable(False)

            self.get_logger().info(f"Stored {len(self.orders)} orders. IDs={[o.orderId for o in self.orders]}")

        except Exception as e:
            self.get_logger().error(f'on_order_list error: {e}')

    # ========== 위치 갱신 ==========
    def location_callback(self, msg: Point):
        self.robot_lat = _to_float(msg.x)
        self.robot_lon = _to_float(msg.y)
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
            dist = haversine_m(self.robot_lat, self.robot_lon,
                               target.customerLatitude, target.customerLongitude)
        except Exception as e:
            self.get_logger().warning(f"distance calc failed: {e}")
            return

        if dist <= self.arrive_radius_m:
            if target.status != "픽업존대기중":
                self.set_order_state(self.current_target_index, "픽업존대기중")
                self.get_logger().info(
                    f"[Arrived] orderId={target.orderId}, bay={target.spaceNum}, dist={dist:.2f} m"
                )

            if not target.arrival_notified:
                # 도착 → 차량 정지
                self.publish_drive_enable(False)

                # 얼굴 DB 등록 (캐시된 base64만 사용)
                img_b64 = target.face_b64
                if img_b64:
                    reg = String()
                    reg.data = json.dumps({"orderId": target.orderId,
                                           "image_base64": img_b64}, ensure_ascii=False)
                    self.pub_face_register.publish(reg)
                else:
                    self.get_logger().warning(f"[FaceDB] No face image base64 for orderId={target.orderId}")

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
            self.get_logger().warning(f"face_match parse error: {e}")
            return

        if self.current_target_index is None or not self.orders:
            return
        if self.current_target_index >= len(self.orders):
            return

        target = self.orders[self.current_target_index]
        if order_id != target.orderId:
            return

        if target.status == "픽업존대기중" and matched:
            # 음식칸 열기 → 일정 시간 뒤 1회 닫기
            self.open_food_bay_then_close(target.spaceNum, self.door_open_sec)

            # 주문 완료 → 다음 주문으로
            self.set_order_state(self.current_target_index, "배달완료")
            self.get_logger().info(f"[Completed] orderId={target.orderId}")

            self.current_target_index += 1
            if self.current_target_index is not None and self.current_target_index < len(self.orders):
                nxt = self.orders[self.current_target_index]
                self.set_order_state(self.current_target_index, "배달중")
                self.set_robot_status("주행중")
                self.publish_drive_enable(True)
                self.get_logger().info(
                    f"Next target -> orderId={nxt.orderId}, bay={nxt.spaceNum}"
                )
            else:
                self.current_target_index = None
                self.set_robot_status("대기중")
                self.publish_drive_enable(False)
                self.get_logger().info("All deliveries completed. Robot is now 대기중.")

    def publish_food_bay(self, bay: int, state: str):
        msg = String()
        msg.data = f"{bay} {state}"
        self.pub_food_bay.publish(msg)

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
        self.publish_food_bay(bay, 'OPEN')

        # duration 후 1회 닫기 (콜백 내부에서 자기 자신 cancel)
        def cb():
            try:
                self.publish_food_bay(bay, 'CLOSE')
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

    def set_robot_status(self, s: str):
        out = String()
        out.data = s
        self.pub_robot_status.publish(out)

    def set_order_state(self, idx: int, state: str):
        self.orders[idx].status = state
        out = String()
        out.data = state
        self.pub_delivery_state.publish(out)
        j = String()
        j.data = json.dumps({"orderId": self.orders[idx].orderId, "state": state}, ensure_ascii=False)
        self.pub_delivery_state_json.publish(j)

    def publish_drive_enable(self, enable: bool):
        m = Bool()
        m.data = bool(enable)
        self.pub_drive_enable.publish(m)

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
