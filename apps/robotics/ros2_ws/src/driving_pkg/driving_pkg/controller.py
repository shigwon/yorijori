import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from adafruit_servokit import ServoKit
from adafruit_pca9685 import PCA9685
import board
import busio

import time

class Controller(Node):
    def __init__(self):
        super().__init__('controller_node')

        i2c = busio.I2C(board.SCL, board.SDA)

        # 스티어링 서보 (0x60)
        self.servo_kit = ServoKit(channels=16, i2c=i2c, address=0x60)

        # 스로틀용 PCA9685 (0x40)
        self.pca_throttle = PCA9685(i2c, address=0x40)
        self.pca_throttle.frequency = 60

        # MotorHatB 채널 설정 (TB6612FNG)
        self.in1 = 3
        self.in2 = 4
        self.ena = 5

        # Steering 보정값
        self.STEER_CENTER = 70  # 실제 바퀴가 정면을 보는 서보 각도
        self.STEER_RANGE = 30   # ±30도 논리각 → ±30도 실제 서보 움직임

        self.subscription = self.create_subscription(
            Float32MultiArray,
            'vehicle_control',
            self.control_callback,
            10
        )

        self.get_logger().info("Vehicle control node initialized.")

    def logical_to_servo_angle(self, logical_angle):
        angle = self.STEER_CENTER + (logical_angle / self.STEER_RANGE) * 30.0
        return max(0, min(180, angle))

    def control_callback(self, msg):
        if len(msg.data) != 2:
            self.get_logger().error("Expected 2 values: [angle, speed]")
            return

        logical_angle, speed = msg.data

        # 서보 각도로 변환
        try:
            servo_angle = self.logical_to_servo_angle(logical_angle)
            self.servo_kit.servo[0].angle = servo_angle
            self.get_logger().info(f"Steering angle set to {servo_angle} (from logical {logical_angle})")
        except Exception as e:
            self.get_logger().error(f"Servo error: {e}")

        # DC 모터 제어
        try:
            pulse = int(0xFFFF * abs(speed))
            if speed > 0:
                self.pca_throttle.channels[self.in1].duty_cycle = 0xFFFF
                self.pca_throttle.channels[self.in2].duty_cycle = 0
            elif speed < 0:
                self.pca_throttle.channels[self.in1].duty_cycle = 0
                self.pca_throttle.channels[self.in2].duty_cycle = 0xFFFF
            else:
                self.pca_throttle.channels[self.in1].duty_cycle = 0
                self.pca_throttle.channels[self.in2].duty_cycle = 0

            self.pca_throttle.channels[self.ena].duty_cycle = pulse
            self.get_logger().info(f"Throttle set to {speed}")
        except Exception as e:
            self.get_logger().error(f"Motor error: {e}")

    def destroy_node(self):
        self.pca_throttle.deinit()
        self.get_logger().info("Node destroyed, PWM deinitialized.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
