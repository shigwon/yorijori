import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from adafruit_servokit import ServoKit
import board
import busio

class ServoMotorNode(Node):
    def __init__(self):
        super().__init__('servo_motor_controller_node')

        self.get_logger().info('Initializing Servo Motor Node...')

        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.kit = ServoKit(channels=16, i2c=i2c, address=0x60)
            self.kit.servo[0].angle = 90  # 중앙 위치 초기화
            self.get_logger().info('PCA9685 initialized at address 0x60, angle set to 90.')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize ServoKit: {e}')
            raise

        self.subscription = self.create_subscription(
            Float32,
            '/angle',
            self.angle_callback,
            10
        )

    def angle_callback(self, msg):
        angle = msg.data
        angle = max(0.0, min(180.0, angle))  # 범위 제한
        self.get_logger().info(f'Setting servo angle to: {angle}')
        try:
            self.kit.servo[0].angle = angle
        except Exception as e:
            self.get_logger().error(f'Failed to set angle: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ServoMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down servo motor node.')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
