#!/usr/bin/env python3
import json
import math
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, String

# ===== 설정 =====
DS_RESAMPLE = 0.05   # 경로 치밀화 간격 [m]

ODOM_TOPIC    = '/visual_slam/tracking/odometry'
TARGETS_TOPIC = '/targets_xy'     # Float32MultiArray: [x1,y1,x2,y2,...]
WAYPTS_TOPIC  = '/waypoints'      # Float32MultiArray: [x0,y0,x1,y1,...]

# ---------- 유틸 ----------
def _is_finite_xy(p: Tuple[float, float]) -> bool:
    x, y = p
    return math.isfinite(x) and math.isfinite(y)

def dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    dx, dy = a[0] - b[0], a[1] - b[1]
    return math.hypot(dx, dy)

def polyline_length(poly: List[Tuple[float, float]]) -> float:
    return sum(dist(poly[i], poly[i + 1]) for i in range(len(poly) - 1))

def densify(polyline: List[Tuple[float, float]], ds: float) -> List[Tuple[float, float]]:
    ds = max(1e-6, float(ds))
    if len(polyline) < 2:
        return polyline[:]
    out: List[Tuple[float, float]] = []
    for i in range(len(polyline) - 1):
        x0, y0 = polyline[i]
        x1, y1 = polyline[i + 1]
        seg_len = dist((x0, y0), (x1, y1))
        if seg_len < 1e-9:
            continue
        steps = max(1, int(seg_len / ds))
        for k in range(steps):
            t = k / steps
            out.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
    out.append(polyline[-1])
    return out

def nearest_neighbor_order(start: Tuple[float, float], targets: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    if not targets:
        return [start]
    rem = targets[:]
    path = [start]
    cur = start
    while rem:
        j = min(range(len(rem)), key=lambda k: dist(cur, rem[k]))
        cur = rem.pop(j)
        path.append(cur)
    return path

# ---------- 노드 ----------
class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        # ODOM 상태
        self.have_pose = False
        self.odom_xy: Tuple[float, float] = (0.0, 0.0)

        # “한 번만” 계획 상태
        self.start_xy0: Optional[Tuple[float, float]] = None  # 웨이포인트 수신 시점(또는 그 직후 첫 ODOM)의 시작점
        self.planned_once: bool = False                        # 이번 미션(웨이포인트 세트)에 대해 이미 계획했는가

        # 타겟
        self.targets_xy: List[Tuple[float, float]] = []
        self.targets_ids: List[str] = []  # 있으면 방문 순서 매핑에 활용

        # 퍼블리셔 (late-joiner 지원)
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.pub_wp = self.create_publisher(Float32MultiArray, WAYPTS_TOPIC, latched_qos)
        self.pub_visit_order = self.create_publisher(String, '/visit_order', 10)

        # 구독
        self.create_subscription(Odometry, ODOM_TOPIC, self.odom_cb, 20)
        self.create_subscription(Float32MultiArray, TARGETS_TOPIC, self.targets_cb, 10)
        self.create_subscription(String, '/targets_order', self.targets_order_cb, 10)

        self.get_logger().info('path_planner_node ready (start locked at targets reception; NN only)')

    # ---- 콜백 ----
    def odom_cb(self, msg: Odometry):
        self.odom_xy = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
        if not _is_finite_xy(self.odom_xy):
            return
        was_ready = self.have_pose
        self.have_pose = True

        # 웨이포인트는 이미 있고(planned_once False), 아직 시작점 잠금이 안 됐으면
        # → 다음에 들어온 첫 ODOM을 시작점으로 잠그고 1회 계획
        if (not was_ready) and (not self.planned_once) and self.start_xy0 is None and self.targets_xy:
            self.start_xy0 = self.odom_xy
            self.plan_once()

    def targets_cb(self, msg: Float32MultiArray):
        # 새 웨이포인트 세트 수신 → 새 미션으로 간주하고 리셋
        data = list(msg.data or [])
        n = (len(data) // 2) * 2
        if n < 2:
            self.targets_xy = []
            self.get_logger().warning('/targets_xy empty; cleared targets')
        else:
            pts = [(float(data[i]), float(data[i + 1])) for i in range(0, n, 2)]
            # NaN/중복 제거
            uniq: List[Tuple[float, float]] = []
            seen = set()
            for x, y in pts:
                if not (math.isfinite(x) and math.isfinite(y)):
                    continue
                key = (round(x, 4), round(y, 4))
                if key not in seen:
                    seen.add(key)
                    uniq.append((x, y))
            self.targets_xy = uniq
            self.get_logger().info(f'Received {len(self.targets_xy)} target(s)')

        # 새 미션 리셋
        self.planned_once = False
        self.start_xy0 = None

        # “웨이포인트를 받은 시점의 현재 좌표”가 시작점 요구사항
        # ODOM이 이미 준비되어 있으면 즉시 잠그고 1회 계획
        if self.have_pose and self.targets_xy:
            self.start_xy0 = self.odom_xy
            self.plan_once()
        # 아니면 ODOM 처음 들어올 때(odom_cb) 잠그고 계획

    def targets_order_cb(self, msg: String):
        # ID 배열은 경로계산엔 영향 X. 계산된 순서에 맞춰 재배열하여 /visit_order에 내보내는 용도.
        try:
            ids = json.loads(msg.data)
            assert isinstance(ids, list)
            ids = [str(x) for x in ids]
        except Exception as e:
            self.get_logger().warning(f'/targets_order parse error: {e}')
            return
        self.targets_ids = ids
        if self.targets_xy and len(self.targets_xy) != len(self.targets_ids):
            self.get_logger().warning('targets_xy vs targets_ids length mismatch')

    # ---- 1회 계획 ----
    def plan_once(self):
        if self.planned_once:
            return
        if self.start_xy0 is None or not _is_finite_xy(self.start_xy0):
            self.get_logger().warning('Locked start pose invalid; skip planning')
            return
        if not self.targets_xy:
            self.get_logger().warning('No targets to plan; skip')
            return

        start = self.start_xy0
        targets = self.targets_xy[:]

        # 1) 순서 구성: 최근접 이웃(NN)만 사용
        order = nearest_neighbor_order(start, targets)  # [start, t1, t2, ...]

        if len(order) < 2:
            self.get_logger().warning('Nothing to publish (order too short)')
            return

        # 2) 치밀화
        dense = densify(order, DS_RESAMPLE)

        # 3) 퍼블리시 (/waypoints)
        msg = Float32MultiArray()
        flat: List[float] = []
        for x, y in dense:
            flat.extend((float(x), float(y)))
        msg.data = flat
        self.pub_wp.publish(msg)

        self.planned_once = True
        self.get_logger().info(
            f'[PLAN ONCE NN] /waypoints: {len(dense)} pts | route_len={polyline_length(order):.2f} m | '
            f'start=({start[0]:.3f},{start[1]:.3f}) targets={len(targets)}'
        )

        # 4) (옵션) 방문 ID 순서 퍼블리시: 계산된 순서에 맞춰 재배열
        if self.targets_ids and len(self.targets_ids) == len(self.targets_xy):
            def keyxy(p): return (round(p[0], 4), round(p[1], 4))
            idx_map = { keyxy(self.targets_xy[i]): i for i in range(len(self.targets_xy)) }
            visit_ids: List[str] = []
            for pt in order[1:]:  # start 제외
                i = idx_map.get(keyxy(pt), None)
                if i is not None and 0 <= i < len(self.targets_ids):
                    visit_ids.append(self.targets_ids[i])
            if visit_ids:
                m = String()
                m.data = json.dumps(visit_ids, ensure_ascii=False)
                self.pub_visit_order.publish(m)
                self.get_logger().info(f'Published /visit_order: {visit_ids}')

def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
