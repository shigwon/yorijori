#!/usr/bin/env python3
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Bool

# ===== 설정(필요 최소) =====
WHEELBASE         = 0.22   # m
LOOKAHEAD         = 1.0    # m
STEER_LIMIT_DEG   = 30.0   # deg
TARGET_SPEED_NORM = 0.40   # [-1, 1]
CTRL_HZ           = 30.0
K_STEER           = 1.3    # 조향 이득

ODOM_TOPIC      = '/visual_slam/tracking/odometry'
WAYPTS_TOPIC    = '/waypoints'
CTRL_TOPIC      = 'vehicle_control'     # 루트 슬래시 없음
DRIVE_EN_TOPIC  = '/drive_enable'       # Linky가 퍼블리시

def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller_node')

        # 상수 -> 멤버
        self.L         = float(WHEELBASE)
        self.Ld        = float(LOOKAHEAD)
        self.steer_lim = float(STEER_LIMIT_DEG)
        self.v_cmd     = float(TARGET_SPEED_NORM)
        self.hz        = float(CTRL_HZ)
        self.k_steer   = float(K_STEER)

        # 상태
        self.path: List[Tuple[float, float]] = []
        self.have_pose = False
        self.drive_enabled = False          # ★ Linky와 호환: 기본 False
        self.x = 0.0; self.y = 0.0; self.yaw = 0.0
        self.last_near_idx = 0

        # 구독/퍼블리시
        self.create_subscription(Odometry, ODOM_TOPIC, self.odom_cb, 20)

        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.create_subscription(Float32MultiArray, WAYPTS_TOPIC, self.waypoints_cb, latched_qos)

        # ★ drive_enable 게이팅
        self.create_subscription(Bool, DRIVE_EN_TOPIC, self.drive_en_cb, 10)

        self.ctrl_pub = self.create_publisher(Float32MultiArray, CTRL_TOPIC, 10)
        self.timer = self.create_timer(1.0 / self.hz, self.on_timer)

        self.get_logger().info("pure_pursuit_controller_node ready (tracking only, gated by /drive_enable)")

    # ---------- callbacks ----------
    def drive_en_cb(self, msg: Bool):
        self.drive_enabled = bool(msg.data)

    def waypoints_cb(self, msg: Float32MultiArray):
        data = list(msg.data or [])
        n = (len(data) // 2) * 2
        if n < 4:
            self.get_logger().warning("/waypoints too short; ignore")
            return

        pts: List[Tuple[float, float]] = []
        for i in range(0, n, 2):
            x = float(data[i]); y = float(data[i+1])
            if math.isfinite(x) and math.isfinite(y):
                pts.append((x, y))

        if len(pts) < 2:
            self.get_logger().warning("/waypoints invalid values; ignore")
            return

        self.path = pts
        self.last_near_idx = 0
        self.get_logger().info(f"Waypoints updated: {len(self.path)} pts")

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.have_pose = True

    # ---------- core ----------
    def nearest_index(self):
        if not self.path:
            return 0
        start = max(0, self.last_near_idx - 50)
        best_i = start
        best_d2 = float('inf')
        for i in range(start, len(self.path)):
            px, py = self.path[i]
            d2 = (px - self.x) ** 2 + (py - self.y) ** 2
            if d2 < best_d2:
                best_d2 = d2; best_i = i
            else:
                if i - best_i > 200:
                    break
        self.last_near_idx = best_i
        return best_i

    def target_index(self, near_i):
        if near_i >= len(self.path) - 1:
            return len(self.path) - 1
        acc = 0.0; i = near_i
        while i < len(self.path) - 1 and acc < self.Ld:
            x0, y0 = self.path[i]
            x1, y1 = self.path[i + 1]
            acc += math.hypot(x1 - x0, y1 - y0)
            i += 1
        return min(i, len(self.path) - 1)

    def on_timer(self):
        # 준비 안 됐거나 drive_disable이면 정지 명령만
        if not self.have_pose or len(self.path) < 2 or not self.drive_enabled:
            self.publish_control(0.0, 0.0)
            return

        ni = self.nearest_index()
        ti = self.target_index(ni)
        tx, ty = self.path[ti]

        # 차량 좌표계로 변환
        dx = tx - self.x; dy = ty - self.y
        cy = math.cos(self.yaw); sy = math.sin(self.yaw)
        x_b = cy * dx + sy * dy           # 전방(+)
        y_b = sy * dx - cy * dy           # 좌측(+)

        # 목표점이 뒤쪽이면 약간 더 앞 점을 보도록 보정
        if x_b < 0.0 and ti < len(self.path) - 1:
            ti = min(ti + 5, len(self.path) - 1)
            tx, ty = self.path[ti]
            dx = tx - self.x; dy = ty - self.y
            x_b = cy * dx + sy * dy
            y_b = sy * dx - cy * dy

        # 조향 계산
        Ld_eff = max(0.2, math.hypot(x_b, y_b))
        kappa = 2.0 * y_b / (Ld_eff * Ld_eff)
        steer_deg = math.degrees(math.atan(self.L * kappa)) * self.k_steer
        steer_deg = clamp(steer_deg, -self.steer_lim, self.steer_lim)

        # 속도는 항상 v_cmd (정지/재출발은 /drive_enable로 제어)
        self.publish_control(steer_deg, self.v_cmd)

    def publish_control(self, steer_deg: float, speed_norm: float):
        m = Float32MultiArray()
        m.data = [float(steer_deg), float(speed_norm)]
        self.ctrl_pub.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
