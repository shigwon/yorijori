import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import random

class SlamNode(Node):
    def __init__(self):
        super().__init__('slam_node')
        self.publisher_ = self.create_publisher(Point, 'slam/position', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = Point()
        msg.x = random.uniform(-10.0, 10.0)
        msg.y = random.uniform(-10.0, 10.0)
        msg.z = random.uniform(-1.0, 1.0)
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Published position: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = SlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
