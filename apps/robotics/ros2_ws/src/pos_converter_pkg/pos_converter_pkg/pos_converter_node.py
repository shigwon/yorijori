#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

# --------- 캘리브레이션 대응점 (SLAM <-> GPS 평면) ----------
# [ [SLAM_X, SLAM_Y], [GPS_LAT, GPS_LON] ]
PAIRS = [
    [[0.0,   0.0],   [0.0,   0.0]],
    [[10.0, 10.10],  [10.0, 10.10]],
    [[20.0, 10.0],   [20.0, 10.0]],
    [[34.0, 15.0],   [34.0, 15.0]],
]

class PoseConverterMini(Node):
    def __init__(self):
        super().__init__('pose_converter_mini')

        # 유사변환 파라미터
        self.calibrated = False
        self.s = 1.0
        self.R = np.eye(2)
        self.t = np.zeros(2)
        self.Rt = np.eye(2)

        # 캘리브레이션
        self._calibrate_from_pairs()

        # Pub/Sub
        self.pub_gps = self.create_publisher(Point, '/gps_ll_out', 10)   # x=lat, y=lon (평면)
        self.pub_xy  = self.create_publisher(Point, '/pose_xy_out', 10)  # x,y in SLAM

        self.create_subscription(Odometry, '/visual_slam/tracking/odometry', self.on_odom, 10)
        self.create_subscription(Point, '/gps_ll_in', self.on_gps, 10)

        self.get_logger().info('pose_converter_mini ready (SLAM <-> GPS planar)')

    # ---------- Calibration ----------
    def _calibrate_from_pairs(self):
        if len(PAIRS) < 2:
            self.get_logger().error('Need at least 2 point pairs for similarity fit.')
            return
        X = np.array([slam for slam, _ in PAIRS], dtype=float)  # SLAM
        Y = np.array([gps  for _, gps in PAIRS], dtype=float)   # GPS-like
        self.s, self.R, self.t = self._fit_similarity(X, Y)
        self.Rt = self.R.T
        self.calibrated = True
        self.get_logger().info(
            f'Calibrated: s={self.s:.6f}, R=\n{self.R}\n t={self.t}'
        )

    @staticmethod
    def _fit_similarity(X: np.ndarray, Y: np.ndarray):
        mu_x = X.mean(axis=0); mu_y = Y.mean(axis=0)
        Xc = X - mu_x; Yc = Y - mu_y
        H = (Xc.T @ Yc) / X.shape[0]
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[1, :] *= -1.0
            R = Vt.T @ U.T
        var_x = np.mean(np.sum(Xc**2, axis=1))
        s = S.sum() / max(var_x, 1e-12)
        t = mu_y - s * (R @ mu_x)
        return float(s), R, t

    # ---------- Callbacks ----------
    def on_odom(self, msg: Odometry):
        """SLAM (x,y) -> GPS 평면 (lat,lon)"""
        if not self.calibrated:
            return
        px = float(msg.pose.pose.position.x)
        py = float(msg.pose.pose.position.y)
        p = np.array([px, py], dtype=float)

        g = self.s * (self.R @ p) + self.t  # -> GPS 평면

        out = Point()
        out.x = float(g[0])  # lat-like
        out.y = float(g[1])  # lon-like
        out.z = msg.pose.pose.position.z
        self.pub_gps.publish(out)

        # 레이트 제한이 필요하면 아래를 대신 사용:
        # now = self.get_clock().now()
        # if (now - self._last_pub).nanoseconds >= int(1e9 / PUBLISH_HZ):
        #     self.pub_gps.publish(out); self._last_pub = now

    def on_gps(self, msg: Point):
        """GPS 평면 (lat,lon) -> SLAM (x,y)"""
        if not self.calibrated:
            return
        g = np.array([float(msg.x), float(msg.y)], dtype=float)

        p = self.Rt @ ((g - self.t) / max(self.s, 1e-12))  # -> SLAM

        out = Point()
        out.x = float(p[0])
        out.y = float(p[1])
        out.z = msg.z
        self.pub_xy.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = PoseConverterMini()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
