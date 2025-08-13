#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

# SLAM(x,y) ↔ GPS(lat,lon) 짝을 코드 안에 고정
# 예시 값이니 네 값으로 교체하세요.
PAIRS = [
    # x,   y,    lat,      lon
    0.0,  0.0,   10.0,     10.0,
    10.0, 0.0,   10.0,     100.0,
    0.0,  10.0,  20.0,     10.1,
    # 필요한 만큼 계속 추가 가능: x,y,lat,lon, ...
]

class PoseConverterMiniStatic(Node):
    def __init__(self):
        super().__init__('pos_converter_node')
        self.calibrated = False
        self.s = 1.0
        self.R = np.eye(2)
        self.t = np.zeros(2)

        # pub/sub
        self.pub_ll = self.create_publisher(Point, '/gps_ll_out', 10)   # x=lat, y=lon
        self.pub_xy = self.create_publisher(Point, '/pose_xy_out', 10)  # x,y (slam)
        self.create_subscription(Point, '/pose_xy_in', self.on_xy, 10)
        self.create_subscription(Point, '/gps_ll_in', self.on_ll, 10)

        # 시작 시 고정 점들로 보정
        self._calibrate_from_static_pairs()

    def _calibrate_from_static_pairs(self):
        d = list(PAIRS)
        if len(d) < 8 or (len(d) % 4) != 0:
            self.get_logger().error('PAIRS must be [x,y,lat,lon]*N (N>=2)')
            return
        X, Y = [], []
        for i in range(0, len(d), 4):
            X.append([float(d[i]), float(d[i+1])])       # SLAM
            Y.append([float(d[i+2]), float(d[i+3])])     # GPS
        X = np.asarray(X); Y = np.asarray(Y)
        self.s, self.R, self.t = self._fit_similarity(X, Y)
        self.calibrated = True
        self.get_logger().info(
            f'Calibrated.\n s={self.s:.12f}\n R=\n{self.R}\n t=[{self.t[0]:.12f}, {self.t[1]:.12f}]'
        )

    def on_xy(self, msg: Point):
        if not self.calibrated:
            return
        p = np.array([float(msg.x), float(msg.y)])
        g = self.s * (self.R @ p) + self.t
        out = Point(); out.x, out.y, out.z = float(g[0]), float(g[1]), msg.z
        self.pub_ll.publish(out)

    def on_ll(self, msg: Point):
        if not self.calibrated:
            return
        g = np.array([float(msg.x), float(msg.y)])
        p = (1.0 / self.s) * (g - self.t)
        p = self.R.T @ p
        out = Point(); out.x, out.y, out.z = float(p[0]), float(p[1]), msg.z
        self.pub_xy.publish(out)

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

def main(args=None):
    rclpy.init(args=args)
    node = PoseConverterMiniStatic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
