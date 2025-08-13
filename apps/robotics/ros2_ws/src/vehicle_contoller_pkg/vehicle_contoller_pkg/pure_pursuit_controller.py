#!/usr/bin/env python3
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray

import matplotlib.pyplot as plt
from matplotlib.patches import Circle


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller_node')

        self.declare_parameter('wheelbase', 0.22)
        self.declare_parameter('lookahead', 1.0)
        self.declare_parameter('steer_limit_deg', 30.0)
        self.declare_parameter('target_speed_norm', 0.40)
        self.declare_parameter('resample_ds', 0.05)

        self.L = float(self.get_parameter('wheelbase').value)
        self.Ld = float(self.get_parameter('lookahead').value)
        self.steer_lim = float(self.get_parameter('steer_limit_deg').value)
        self.v_cmd = float(self.get_parameter('target_speed_norm').value)
        self.ds = float(self.get_parameter('resample_ds').value)

        self.hz = 30.0
        self.goal_tol = 0.15
        self.stop_at_goal = True
        self.k_steer = 1.3

        self.path: List[Tuple[float, float]] = []
        self.base_waypoints: List[Tuple[float, float]] = [
            (0.9715180397033691, -0.3445192575454712),
            (2.96824312210083, -0.0661897897720337),
            (5.96824312210083, -0.561897897720337),
        ]
        self.update_path_from_base()

        self.pose_sub = self.create_subscription(PoseStamped, '/visual_slam/tracking/vo_pose', self.pose_cb, 20)
        self.wp_sub = self.create_subscription(Float32MultiArray, '/waypoints', self.waypoints_cb, 10)
        self.ctrl_pub = self.create_publisher(Float32MultiArray, 'vehicle_control', 10)
        self.timer = self.create_timer(1.0 / self.hz, self.on_timer)

        self.have_pose = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_near_idx = 0
        self.finished = False

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(7, 7))
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.grid(True, linestyle='--', alpha=0.3)
        self.ax.set_xlabel('x (forward →)')
        self.ax.set_ylabel('y (left +)')

        xs = [p[0] for p in self.path]
        ys = [p[1] for p in self.path]
        self.l_path, = self.ax.plot(xs, ys, '-', lw=2, label='path')
        self.s_start = self.ax.scatter(xs[0], ys[0], s=40, c='g', label='start')
        self.s_goal = self.ax.scatter(xs[-1], ys[-1], s=40, c='b', label='goal')
        self.s_robot = self.ax.scatter(self.x, self.y, s=50, c='k', label='robot')
        self.l_heading, = self.ax.plot([self.x, self.x], [self.y, self.y], 'k-', lw=2, alpha=0.7, label='heading')
        self.s_near = self.ax.scatter([], [], s=30, c='orange', label='nearest')
        self.s_target = self.ax.scatter([], [], s=30, c='r', label='target')
        self.look_circle = Circle((self.x, self.y), self.Ld, edgecolor='m', facecolor='none', ls='--', lw=1.5, label='lookahead')
        self.ax.add_patch(self.look_circle)
        self.ax.legend(loc='best', fontsize=8)
        self._autoscale_view()
        plt.show(block=False)

    def waypoints_cb(self, msg: Float32MultiArray):
        data = list(msg.data)
        n = (len(data) // 2) * 2
        if n < 4:
            return
        self.base_waypoints = [(float(data[i]), float(data[i + 1])) for i in range(0, n, 2)]
        self.update_path_from_base()
        self.finished = False
        self.last_near_idx = 0
        self._update_path_artists()

    def update_path_from_base(self):
        self.path = self.generate_dense_path(self.base_waypoints, self.ds)

    def generate_dense_path(self, waypoints: List[Tuple[float, float]], ds: float) -> List[Tuple[float, float]]:
        out: List[Tuple[float, float]] = []
        if len(waypoints) < 2:
            return out
        for i in range(len(waypoints) - 1):
            x0, y0 = waypoints[i]
            x1, y1 = waypoints[i + 1]
            dx, dy = x1 - x0, y1 - y0
            L = math.hypot(dx, dy)
            if L < 1e-9:
                continue
            steps = max(1, int(L / ds))
            for k in range(steps):
                t = k / steps
                out.append((x0 + dx * t, y0 + dy * t))
        out.append(waypoints[-1])
        return out

    def pose_cb(self, msg: PoseStamped):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        q = msg.pose.orientation
        self.yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.have_pose = True

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
                best_d2 = d2
                best_i = i
            else:
                if i - best_i > 200:
                    break
        self.last_near_idx = best_i
        return best_i

    def target_index(self, near_i):
        if near_i >= len(self.path) - 1:
            return len(self.path) - 1
        acc = 0.0
        i = near_i
        while i < len(self.path) - 1 and acc < self.Ld:
            x0, y0 = self.path[i]
            x1, y1 = self.path[i + 1]
            acc += math.hypot(x1 - x0, y1 - y0)
            i += 1
        return min(i, len(self.path) - 1)

    def on_timer(self):
        if not self.have_pose or self.finished or len(self.path) < 2:
            return

        gx, gy = self.path[-1]
        if math.hypot(gx - self.x, gy - self.y) < self.goal_tol:
            if self.stop_at_goal:
                self.publish_control(0.0, 0.0)
            self.finished = True
            self._update_plot(near_idx=None, target_idx=None)
            return

        ni = self.nearest_index()
        ti = self.target_index(ni)
        tx, ty = self.path[ti]

        dx = tx - self.x
        dy = ty - self.y
        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)
        x_b = cy * dx + sy * dy
        y_b = sy * dx - cy * dy  # ← 수정된 부호(좌/우 일관)

        if x_b < 0.0 and ti < len(self.path) - 1:
            ti = min(ti + 5, len(self.path) - 1)
            tx, ty = self.path[ti]
            dx = tx - self.x
            dy = ty - self.y
            x_b = cy * dx + sy * dy
            y_b = sy * dx - cy * dy  # ← 여기서도 동일한 식을 사용해야 함

        Ld_eff = max(0.2, math.hypot(x_b, y_b))
        kappa = 2.0 * y_b / (Ld_eff * Ld_eff)
        steer_deg = math.degrees(math.atan(self.L * kappa)) * self.k_steer
        steer_deg = clamp(steer_deg, -self.steer_lim, self.steer_lim)

        self.publish_control(steer_deg, self.v_cmd)
        self._update_plot(near_idx=ni, target_idx=ti)

    def publish_control(self, steer_deg: float, speed_norm: float):
        msg = Float32MultiArray()
        msg.data = [float(steer_deg), float(speed_norm)]
        self.ctrl_pub.publish(msg)

    def _update_path_artists(self):
        xs = [p[0] for p in self.path]
        ys = [p[1] for p in self.path]
        self.l_path.set_data(xs, ys)
        if xs and ys:
            self.s_start.set_offsets([[xs[0], ys[0]]])
            self.s_goal.set_offsets([[xs[-1], ys[-1]]])
        self._autoscale_view()
        self.fig.canvas.draw_idle()
        plt.pause(0.001)

    def _update_plot(self, near_idx: int, target_idx: int):
        self.s_robot.set_offsets([[self.x, self.y]])
        hx = self.x + 0.4 * math.cos(self.yaw)
        hy = self.y + 0.4 * math.sin(self.yaw)
        self.l_heading.set_data([self.x, hx], [self.y, hy])

        if near_idx is not None and 0 <= near_idx < len(self.path):
            nx, ny = self.path[near_idx]
            self.s_near.set_offsets([[nx, ny]])
        if target_idx is not None and 0 <= target_idx < len(self.path):
            tx, ty = self.path[target_idx]
            self.s_target.set_offsets([[tx, ty]])

        self.look_circle.center = (self.x, self.y)
        self.look_circle.set_radius(self.Ld)

        self._autoscale_view()
        self.fig.canvas.draw_idle()
        plt.pause(0.001)

    def _autoscale_view(self):
        if not self.path:
            cx, cy = self.x, self.y
            r = max(1.0, self.Ld * 2.0)
            self.ax.set_xlim(cx - r, cx + r)
            self.ax.set_ylim(cy - r, cy + r)
            return
        xs = [p[0] for p in self.path] + [self.x]
        ys = [p[1] for p in self.path] + [self.y]
        pad = 1.0
        self.ax.set_xlim(min(xs) - pad, max(xs) + pad)
        self.ax.set_ylim(min(ys) - pad, max(ys) + pad)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.ioff()
        try:
            plt.show()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
