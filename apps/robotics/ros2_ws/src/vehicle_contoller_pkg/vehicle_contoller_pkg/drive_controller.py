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
        super().__init__('drive_controller_node')

        self.hardware_ready = False

        try:
            i2c = busio.I2C(board.SCL, board.SDA)

            self.servo_kit = ServoKit(channels=16, i2c=i2c, address=0x60)
            self.pca_throttle = PCA9685(i2c, address=0x40)
            self.pca_throttle.frequency = 60

            self.in1 = 3
            self.in2 = 4
            self.ena = 5

            self.STEER_CENTER = 70
            self.STEER_RANGE = 30

            self.hardware_ready = True
            self.get_logger().info("Hardware initialized.")

        except Exception as e:
            self.get_logger().warn(f"Hardware not initialized (running in mock mode): {e}")

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
        self.get_logger().info(f"Received control: angle={logical_angle}, speed={speed}")

        if not self.hardware_ready:
            self.get_logger().warn("Hardware not available. Skipping control.")
            return

        try:
            servo_angle = self.logical_to_servo_angle(logical_angle)
            self.servo_kit.servo[0].angle = servo_angle
            self.get_logger().info(f"Steering angle set to {servo_angle} (from logical {logical_angle})")
        except Exception as e:
            self.get_logger().error(f"Servo error: {e}")

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
        if self.hardware_ready:
            self.pca_throttle.deinit()
            self.get_logger().info("PWM deinitialized.")
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
