#!/usr/bin/env python3
import os, json, time, math, base64
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Tuple

import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from std_msgs.msg import String, Bool, Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point  # 좌표 변환 I/O

MAX_ORDERS        = 3
ALLOW_OVERWRITE   = True
ARRIVE_RADIUS_M   = 1.0
DOOR_OPEN_SEC     = 1.0
FACE_CACHE_DIR    = "/tmp/face_cache"
FACE_MAX_BYTES    = 3 * 1024 * 1024
ROBOT_ID          = 1

FINAL_GOAL_X = 12.34
FINAL_GOAL_Y = -5.67

STATE_HEARTBEAT_SEC = 0.0  # 0: heartbeat off

def _to_float(x, default=None) -> Optional[float]:
    try:
        if x is None: return default
        return float(x)
    except Exception:
        return default

def _to_int(x, default=None) -> Optional[int]:
    try:
        if x is None: return default
        return int(x)
    except Exception:
        return default

def _extract_pure_b64(s: str) -> str:
    if not isinstance(s, str): return ""
    s = s.strip()
    if not s: return ""
    if ';base64,' in s:
        s = s.split(';base64,', 1)[1]
    return s

def _validate_b64_and_optionally_save(order_id: str, b64: str, max_bytes: int, cache_dir: str) -> Tuple[str, str]:
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
        face_path = ""
    normalized_b64 = base64.b64encode(raw).decode("ascii")
    return normalized_b64, face_path

@dataclass
class Order:
    orderId: str
    code: str
    tel: str
    customerLatitude: float
    customerLongitude: float
    spaceNum: int
    createdTs: float = field(default_factory=lambda: time.time())
    status: str = "READY_FOR_PICKUP"
    arrival_notified: bool = False
    face_path: str = ""
    face_b64: str = ""
    slam_x: Optional[float] = None
    slam_y: Optional[float] = None
    last_state_pub_ts: float = 0.0
    initial_state_sent: bool = False  # ★ 초기 상태 1회 송신 체크

class OrdersStoreNode(Node):
    def __init__(self):
        super().__init__('linky')

        # ------- 런타임 설정 -------
        self.max_orders       = int(MAX_ORDERS)
        self.allow_overwrite  = bool(ALLOW_OVERWRITE)
        self.arrive_radius_m  = float(ARRIVE_RADIUS_M)
        self.door_open_sec    = float(DOOR_OPEN_SEC)
        self.face_cache_dir   = str(FACE_CACHE_DIR)
        self.face_max_bytes   = int(FACE_MAX_BYTES)
        os.makedirs(self.face_cache_dir, exist_ok=True)

        # ------- 상태 -------
        self.orders: List[Order] = []
        self._stored_once = False
        self.robot_x: Optional[float] = None
        self.robot_y: Optional[float] = None

        # 좌표 변환 seq 매핑
        self._seq_counter: int = 1
        self._seq_to_oid: Dict[int, str] = {}
        self._pending_xy: Dict[str, bool] = {}

        # 방문 순서/인덱스
        self.plan_order_ids: List[str] = []
        self.plan_idx: int = 0

        # 복귀
        self.return_goal: Optional[Tuple[float, float]] = (FINAL_GOAL_X, FINAL_GOAL_Y)
        self.returning: bool = False

        # 로봇 상태 캐시(중복 전송 방지)
        self._last_robot_status: Optional[str] = None

        self._targets_published: bool = False

        # ------- 통신 -------

        # 구독
        self.create_subscription(String, '/server/order_list', self.on_order_list, 10)
        self.create_subscription(String, '/face_match_by_order', self.on_face_match, 10)
        self.create_subscription(Point,  '/pose_xy_out', self.on_xy, 10)          # 변환 응답(seq 포함)
        self.create_subscription(String, '/visit_order', self.on_visit_order, 10)  # 플래너 방문 순서
        self.create_subscription(Odometry, '/visual_slam/tracking/odometry', self.odom_cb, 20)

        latched_qos = QoSProfile(depth=1,
                                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=QoSReliabilityPolicy.RELIABLE)

        # 퍼블리시
        self.pub_robot_status   = self.create_publisher(String, '/robot/status', 10)
        self.pub_delivery_state = self.create_publisher(String, '/robot/delivery_state', 10)
        self.pub_drive_enable   = self.create_publisher(Bool,   '/drive_enable', 10)
        self.pub_face_register  = self.create_publisher(String, '/face_db/register', 10)
        self.pub_food_bay       = self.create_publisher(String, '/food_bay_cmd', 10)

        self.pub_gps_req        = self.create_publisher(Point,  '/gps_ll_in', 10)             # 변환 요청(seq 포함)
        self.pub_targets_xy     = self.create_publisher(Float32MultiArray, '/targets_xy', latched_qos)
        self.pub_targets_order  = self.create_publisher(String, '/targets_order',  latched_qos)

        self._bay_close_timers: Dict[int, Timer] = {}

        # 초기 상태: WAITING
        self.set_robot_status("WAITING")
        self.get_logger().info('OrdersStoreNode 준비 완료 (플래너 순서 기반 SLAM 배달)')

        # 초기 모든 베이 OPEN
        for bay_num in range(1, self.max_orders + 1):
            self.publish_food_bay(bay_num, 'OPEN')
            self.get_logger().info(f"[초기화] BAY={bay_num} OPEN")

    # ------------- 주문 수신 -------------
    def on_order_list(self, msg: String):
        if self._stored_once and not self.allow_overwrite:
            self.get_logger().info('[주문 수신] 이미 저장된 주문이 있어 무시')
            return
        try:
            payload = json.loads(msg.data)
            self.get_logger().info(f"[주문 수신 원본]\n{json.dumps(payload, ensure_ascii=False, indent=2)}")
        except Exception as e:
            self.get_logger().error(f'[주문 수신] JSON 파싱 실패: {e}')
            return

        if isinstance(payload, dict) and "orders" in payload:
            arr = payload["orders"]
        elif isinstance(payload, list):
            arr = payload
        elif isinstance(payload, dict):
            arr = [payload]
        else:
            self.get_logger().warning('[주문 수신] 형식 오류')
            return

        # 초기화
        self.orders.clear()
        self._pending_xy.clear()
        self._seq_to_oid.clear()
        self.plan_order_ids.clear()
        self.plan_idx = 0
        self.returning = False

        # 주문 적재 + 좌표 변환 요청 발행
        for o in arr[: self.max_orders]:
            lat = _to_float(o.get("customerLatitude"))
            lon = _to_float(o.get("customerLongitude"))
            bay = _to_int(o.get("spaceNum"))
            if lat is None or lon is None or bay is None:
                self.get_logger().warning("[주문 수신] 위도/경도/베이번호 누락 → 스킵")
                continue

            ord_obj = Order(
                orderId=str(o.get("orderId", "")),
                code=str(o.get("code", "")),
                tel=str(o.get("tel", "")),
                customerLatitude=lat,
                customerLongitude=lon,
                spaceNum=bay,
            )

            img_b64_raw = _extract_pure_b64(str(o.get("faceImage", "")))
            if img_b64_raw:
                try:
                    normalized_b64, face_path = _validate_b64_and_optionally_save(
                        ord_obj.orderId, img_b64_raw, self.face_max_bytes, self.face_cache_dir
                    )
                    ord_obj.face_b64 = normalized_b64
                    ord_obj.face_path = face_path or ""
                except Exception as e:
                    self.get_logger().warning(f"[주문 수신] 얼굴이미지 오류: {e}")

            # 초기 상태 1회: READY_FOR_PICKUP
            self._publish_order_state_once(ord_obj, "READY_FOR_PICKUP")

            self.orders.append(ord_obj)

            # 좌표 변환 요청: x=lat, y=lon, z=seq
            seq = int(self._seq_counter)
            self._seq_counter = 1 if self._seq_counter >= 1_000_000_000 else self._seq_counter + 1
            self._seq_to_oid[seq] = ord_obj.orderId
            self._pending_xy[ord_obj.orderId] = True

            req = Point()
            req.x = float(lat); req.y = float(lon); req.z = float(seq)
            self.pub_gps_req.publish(req)

            self.get_logger().info(f"[주문 등록] ID={ord_obj.orderId}, BAY={ord_obj.spaceNum}, GPS=({lat},{lon}), SEQ={seq}")

        self._stored_once = True

        # 로봇 상태 준비
        if self.orders:
            self.set_robot_status("WORKING")
        else:
            self.set_robot_status("WAITING")
            self.publish_drive_enable(False)

        self._targets_published = False

    # 좌표 변환 응답
    def on_xy(self, msg: Point):
        try:
            seq = int(round(float(msg.z)))
            x = float(msg.x); y = float(msg.y)
        except Exception as e:
            self.get_logger().warning(f"[좌표 변환 응답] 파싱 오류: {e}")
            return

        oid = self._seq_to_oid.pop(seq, None)
        if not oid:
            self.get_logger().warning(f"[좌표 변환 응답] Unknown SEQ={seq}")
            return

        for o in self.orders:
            if o.orderId == oid:
                o.slam_x = x; o.slam_y = y
                self.get_logger().info(f"[좌표 변환 완료] ID={oid}, SLAM=({x:.2f},{y:.2f}) (SEQ={seq})")
                break
        self._pending_xy.pop(oid, None)

        if not self._pending_xy and not self._targets_published and self.orders:
            self._start_mission_once()

    # ★ 출발 시 한 번만 실행
    def _start_mission_once(self):
        if any(o.slam_x is None or o.slam_y is None for o in self.orders):
            self.get_logger().warning("[플래너 전송] 누락된 SLAM 좌표가 있어 발행 생략")
            return

        # 플래너 입력 1회 발행
        xy = Float32MultiArray()
        flat = []
        for o in self.orders:
            flat.extend([o.slam_x, o.slam_y])
        xy.data = flat
        self.pub_targets_xy.publish(xy)

        ids_msg = String()
        ids_msg.data = json.dumps([o.orderId for o in self.orders], ensure_ascii=False)
        self.pub_targets_order.publish(ids_msg)

        # 방문 순서 초기화(필요시 플래너 override)
        if not self.plan_order_ids:
            self.plan_order_ids = [o.orderId for o in self.orders]
            self.plan_idx = 0

        # ★ 출발과 동시에 "모든 주문"을 DELIVERING 상태로 전환
        for o in self.orders:
            self._mark_state(o.orderId, "DELIVERING")

        # 베이 CLOSE 후 주행 시작
        for bay_num in range(1, self.max_orders + 1):
            self.publish_food_bay(bay_num, 'CLOSE')
        self.publish_drive_enable(True)
        self.set_robot_status("WORKING")

        self._targets_published = True
        self.get_logger().info(f"[플래너 전송] {len(self.orders)}개 좌표 1회 발행 및 출발 (모두 DELIVERING)")

    # 방문 순서 수신
    def on_visit_order(self, msg: String):
        try:
            ids = json.loads(msg.data)
            ids = [str(x) for x in ids if str(x) in {o.orderId for o in self.orders}]
        except Exception as e:
            self.get_logger().warning(f"[방문 순서] 파싱 오류: {e}")
            return
        if not ids:
            self.get_logger().warning("[방문 순서] 유효한 ID 없음")
            return

        self.plan_order_ids = ids
        self.plan_idx = 0

        # ★ 출발 보장이 이 경로에서 이루어질 수도 있으니, 여기서도 모두 DELIVERING 보장
        for o in self.orders:
            self._mark_state(o.orderId, "DELIVERING")

        for bay_num in range(1, self.max_orders + 1):
            self.publish_food_bay(bay_num, 'CLOSE')
        self.publish_drive_enable(True)
        self.set_robot_status("WORKING")

        self.get_logger().info(f"[방문 순서 채택] {self.plan_order_ids}")

    # ODOM & 도착 판정
    def odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.check_arrival()

    def _current_target(self) -> Optional[Order]:
        if 0 <= self.plan_idx < len(self.plan_order_ids):
            oid = self.plan_order_ids[self.plan_idx]
            for o in self.orders:
                if o.orderId == oid:
                    return o
        return None

    def check_arrival(self):
        # 복귀
        if self.returning and self.return_goal and self.robot_x is not None and self.robot_y is not None:
            dist_home = math.hypot(self.robot_x - self.return_goal[0], self.robot_y - self.return_goal[1])
            if dist_home <= self.arrive_radius_m:
                self.returning = False
                self.publish_drive_enable(False)
                self.set_robot_status("WAITING")
                self.get_logger().info("[복귀 완료] 최종 목적지 도착, 대기")
            return

        tgt = self._current_target()
        if not tgt or self.robot_x is None or self.robot_y is None:
            return
        if tgt.slam_x is None or tgt.slam_y is None:
            return

        dist = math.hypot(self.robot_x - tgt.slam_x, self.robot_y - tgt.slam_y)
        if dist <= self.arrive_radius_m:
            if tgt.arrival_notified:
                return
            # 도착 → 주행만 정지, 상태는 계속 DELIVERING 유지 (요청사항)
            self.publish_drive_enable(False)
            self.get_logger().info(f"[도착] 주문={tgt.orderId}, BAY={tgt.spaceNum}")

            # 얼굴 DB 등록(있을 때)
            if tgt.face_b64:
                reg = String()
                reg.data = json.dumps({"orderId": tgt.orderId, "image_base64": tgt.face_b64}, ensure_ascii=False)
                self.pub_face_register.publish(reg)
            tgt.arrival_notified = True
        else:
            # 주행 중 상태 유지(이미 DELIVERING)
            if tgt.status != "DELIVERING":
                self._mark_state(tgt.orderId, "DELIVERING")

    # 얼굴 인증 결과
    def on_face_match(self, msg: String):
        try:
            data = json.loads(msg.data)
            order_id = str(data.get("orderId", ""))
            matched = bool(data.get("matched", False))
        except Exception:
            return
        tgt = self._current_target()
        if not tgt or order_id != tgt.orderId:
            return

        # 매칭 성공 → 베이 열고 닫고 → DELIVERED
        if tgt.status in ("READY_FOR_PICKUP", "DELIVERING") and matched:
            self.open_food_bay_then_close(tgt.spaceNum, self.door_open_sec)
            self._mark_state(tgt.orderId, "DELIVERED")

            # 다음 주문
            self.plan_idx += 1
            nxt = self._current_target()
            if nxt:
                self.set_robot_status("WORKING")
                self.publish_drive_enable(True)
            else:
                # 모든 주문 완료 → 복귀 경로 1회 발행
                goal = self.return_goal
                if not goal or not all(map(math.isfinite, goal)):
                    self.publish_drive_enable(False)
                    self.set_robot_status("WAITING")
                    return
                self.returning = True

                xy = Float32MultiArray()
                xy.data = [float(goal[0]), float(goal[1])]
                self.pub_targets_xy.publish(xy)

                ids_msg = String()
                ids_msg.data = json.dumps([], ensure_ascii=False)  # 복귀에는 주문 없음
                self.pub_targets_order.publish(ids_msg)

                for bay_num in range(1, self.max_orders + 1):
                    self.publish_food_bay(bay_num, 'CLOSE')

                self.set_robot_status("WORKING")
                self.publish_drive_enable(True)

    # 베이 제어
    def publish_food_bay(self, bay: int, state: str):
        self.pub_food_bay.publish(String(data=f"{bay} {state}"))

    def open_food_bay_then_close(self, bay: int, duration_sec: float):
        old = self._bay_close_timers.pop(bay, None)
        if old:
            try: old.cancel()
            except Exception: pass
        self.publish_food_bay(bay, 'OPEN')

        def cb():
            try:
                self.publish_food_bay(bay, 'CLOSE')
            finally:
                t = self._bay_close_timers.pop(bay, None)
                if t:
                    try: t.cancel()
                    except Exception: pass

        timer = self.create_timer(duration_sec, cb)
        self._bay_close_timers[bay] = timer

    # 상태 퍼블리시(중복 방지)
    def set_robot_status(self, s: str):
        if s == self._last_robot_status:
            return
        self._last_robot_status = s
        self.pub_robot_status.publish(String(data=s))

    def _publish_order_state_once(self, ord_obj: Order, state: str):
        if ord_obj.initial_state_sent:
            return
        ord_obj.status = state
        try:
            oid_num = int(ord_obj.orderId)
        except Exception:
            oid_num = ord_obj.orderId
        self.pub_delivery_state.publish(String(data=json.dumps({
            "orderId": oid_num, "robotId": ROBOT_ID, "state": state
        }, ensure_ascii=False)))
        ord_obj.last_state_pub_ts = time.time()
        ord_obj.initial_state_sent = True

    def _mark_state(self, order_id: str, state: str):
        now = time.time()
        for o in self.orders:
            if o.orderId == order_id:
                old_state = o.status
                if old_state == state:
                    if STATE_HEARTBEAT_SEC > 0.0 and (now - o.last_state_pub_ts) >= STATE_HEARTBEAT_SEC:
                        self.pub_delivery_state.publish(String(data=json.dumps({
                            "orderId": order_id, "robotId": ROBOT_ID, "state": state
                        }, ensure_ascii=False)))
                        o.last_state_pub_ts = now
                    return
                o.status = state
                try:
                    oid_num = int(order_id)
                except Exception:
                    oid_num = order_id
                self.pub_delivery_state.publish(String(data=json.dumps({
                    "orderId": oid_num, "robotId": ROBOT_ID, "state": state
                }, ensure_ascii=False)))
                o.last_state_pub_ts = now
                self.get_logger().info(f"[상태 변경] {order_id}: {old_state} → {state}")
                return

    def publish_drive_enable(self, enable: bool):
        self.pub_drive_enable.publish(Bool(data=bool(enable)))

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
