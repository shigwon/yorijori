import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from adafruit_pca9685 import PCA9685
import board
import busio

class MotorHatB:
    def __init__(self, pwm, in1, in2, ena):
        self.pwm = pwm
        self.in1 = in1
        self.in2 = in2
        self.ena = ena
        self.pwm.frequency = 60

    def set_throttle(self, speed):  # -1.0 ~ 1.0
        pulse = int(0xFFFF * min(abs(speed), 1.0))
        if speed > 0:
            self.pwm.channels[self.in1].duty_cycle = 0xFFFF
            self.pwm.channels[self.in2].duty_cycle = 0
        elif speed < 0:
            self.pwm.channels[self.in1].duty_cycle = 0
            self.pwm.channels[self.in2].duty_cycle = 0xFFFF
        else:
            self.pwm.channels[self.in1].duty_cycle = 0
            self.pwm.channels[self.in2].duty_cycle = 0

        self.pwm.channels[self.ena].duty_cycle = pulse

class DCMotorNode(Node):
    def __init__(self):
        super().__init__('dc_motor_node')

        self.get_logger().info('DC Motor Node started')

        # ROS2 구독자 설정
        self.subscriber = self.create_subscription(
            Float32,
            '/throttle',
            self.throttle_callback,
            10
        )

        # PCA9685 초기화
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c, address=0x40)

        # TB6612FNG에 연결된 채널 (예: 3, 4, 5)
        self.motor = MotorHatB(self.pca, in1=3, in2=4, ena=5)

    def throttle_callback(self, msg):
        speed = max(min(msg.data, 1.0), -1.0)
        self.get_logger().info(f'Setting throttle to {speed}')
        self.motor.set_throttle(speed)

    def destroy_node(self):
        self.get_logger().info('Shutting down DC Motor Node')
        self.pca.deinit()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DCMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
