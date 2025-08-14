#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool

from adafruit_servokit import ServoKit
from adafruit_pca9685 import PCA9685
import board
import busio


class Controller(Node):
    def __init__(self):
        super().__init__('drive_controller_node')

        self.declare_parameter('servo_addr', 0x60)
        self.declare_parameter('throttle_addr', 0x40)
        self.declare_parameter('pwm_freq', 60)

        self.declare_parameter('ch_in1', 3)
        self.declare_parameter('ch_in2', 4)
        self.declare_parameter('ch_ena', 5)
        self.declare_parameter('steer_channel', 0)

        self.declare_parameter('steer_center_deg', 70.0)
        self.declare_parameter('steer_max_delta_deg', 60.0)
        self.declare_parameter('logical_angle_range', 60.0)

        self.declare_parameter('steer_gain', 1.5)
        self.declare_parameter('invert_steer', False)
        self.declare_parameter('angle_limit_deg', 90.0)
        self.declare_parameter('servo_min_us', 500)
        self.declare_parameter('servo_max_us', 2500)

        self.declare_parameter('soft_start_ms', 400)
        self.declare_parameter('log_control_debug', False)

        self.servo_addr = int(self.get_parameter('servo_addr').value)
        self.throttle_addr = int(self.get_parameter('throttle_addr').value)
        self.pwm_freq = int(self.get_parameter('pwm_freq').value)

        self.in1 = int(self.get_parameter('ch_in1').value)
        self.in2 = int(self.get_parameter('ch_in2').value)
        self.ena = int(self.get_parameter('ch_ena').value)
        self.steer_ch = int(self.get_parameter('steer_channel').value)

        self.STEER_CENTER = float(self.get_parameter('steer_center_deg').value)
        self.STEER_MAX_DELTA = float(self.get_parameter('steer_max_delta_deg').value)
        self.LOGICAL_RANGE = max(1e-6, float(self.get_parameter('logical_angle_range').value))

        self.STEER_GAIN = float(self.get_parameter('steer_gain').value)
        self.INVERT_STEER = bool(self.get_parameter('invert_steer').value)
        self.ANGLE_LIMIT = float(self.get_parameter('angle_limit_deg').value)
        self.SERVO_MIN_US = int(self.get_parameter('servo_min_us').value)
        self.SERVO_MAX_US = int(self.get_parameter('servo_max_us').value)

        self.SOFT_START_MS = int(self.get_parameter('soft_start_ms').value)
        self.LOG_CONTROL_DEBUG = bool(self.get_parameter('log_control_debug').value)

        self.hardware_ready = False
        self.drive_enabled = True
        self.enable_ts = 0.0

        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.servo_kit = ServoKit(channels=16, i2c=i2c, address=self.servo_addr)
            self.servo_kit.servo[self.steer_ch].set_pulse_width_range(self.SERVO_MIN_US, self.SERVO_MAX_US)
            self.pca_throttle = PCA9685(i2c, address=self.throttle_addr)
            self.pca_throttle.frequency = self.pwm_freq
            self._motor_stop()
            self._set_steer(self.STEER_CENTER)
            self.hardware_ready = True
            self.get_logger().info(f"HW OK servo=0x{self.servo_addr:02X} throttle=0x{self.throttle_addr:02X} freq={self.pwm_freq}")
        except Exception as e:
            self.get_logger().warning(f"HW init failed (mock): {e}")

        self.create_subscription(Float32MultiArray, 'vehicle_control', self.control_callback, 10)
        self.create_subscription(Bool, '/drive_enable', self.enable_callback, 10)
        self.get_logger().info("drive_controller_node ready")

    def logical_to_servo_angle(self, input_angle_deg: float) -> float:
        a = float(input_angle_deg)
        if self.INVERT_STEER:
            a = -a
        a *= self.STEER_GAIN
        if a > self.ANGLE_LIMIT:
            a = self.ANGLE_LIMIT
        if a < -self.ANGLE_LIMIT:
            a = -self.ANGLE_LIMIT
        out = self.STEER_CENTER + a
        if out < 0.0:
            out = 0.0
        if out > 180.0:
            out = 180.0
        return out

    def enable_callback(self, msg: Bool):
        self.drive_enabled = bool(msg.data)
        if not self.hardware_ready:
            return
        if self.drive_enabled:
            self.enable_ts = time.time()
        else:
            try:
                self._motor_stop()
                self._set_steer(self.STEER_CENTER)
            except Exception as e:
                self.get_logger().error(f"disable fail: {e}")

    def control_callback(self, msg: Float32MultiArray):
        if not self.hardware_ready:
            return
        if len(msg.data) == 0:
            return

        steer_input = float(msg.data[0])
        speed = float(msg.data[1]) if len(msg.data) > 1 else 0.0

        try:
            servo_angle = self.logical_to_servo_angle(steer_input)
            self._set_steer(servo_angle)
        except Exception as e:
            self.get_logger().error(f"servo err: {e}")

        try:
            if not self.drive_enabled:
                self._motor_stop()
            else:
                self._motor_drive(self._soft_start_limited_speed(speed))
        except Exception as e:
            self.get_logger().error(f"motor err: {e}")

    def _soft_start_limited_speed(self, target_speed: float) -> float:
        s = max(-1.0, min(1.0, float(target_speed)))
        if self.SOFT_START_MS <= 0:
            return s
        elapsed_ms = (time.time() - self.enable_ts) * 1000.0
        if elapsed_ms <= 0:
            return 0.0
        ramp = min(1.0, elapsed_ms / float(self.SOFT_START_MS))
        return s * ramp

    def _set_steer(self, angle: float):
        if not self.hardware_ready:
            return
        a = float(angle)
        if a < 0.0:
            a = 0.0
        if a > 180.0:
            a = 180.0
        self.servo_kit.servo[self.steer_ch].angle = a

    def _motor_stop(self):
        if not hasattr(self, 'pca_throttle'):
            return
        self.pca_throttle.channels[self.in1].duty_cycle = 0
        self.pca_throttle.channels[self.in2].duty_cycle = 0
        self.pca_throttle.channels[self.ena].duty_cycle = 0

    def _motor_drive(self, speed: float):
        if not hasattr(self, 'pca_throttle'):
            return
        s = max(-1.0, min(1.0, float(speed)))
        pulse = int(0xFFFF * abs(s))
        if s > 0:
            self.pca_throttle.channels[self.in1].duty_cycle = 0
            self.pca_throttle.channels[self.in2].duty_cycle = 0xFFFF
        elif s < 0:
            self.pca_throttle.channels[self.in1].duty_cycle = 0xFFFF
            self.pca_throttle.channels[self.in2].duty_cycle = 0
        else:
            self.pca_throttle.channels[self.in1].duty_cycle = 0
            self.pca_throttle.channels[self.in2].duty_cycle = 0
        self.pca_throttle.channels[self.ena].duty_cycle = pulse

    def destroy_node(self):
        if self.hardware_ready:
            try:
                self._motor_stop()
                self.pca_throttle.deinit()
            except Exception as e:
                self.get_logger().error(f"deinit err: {e}")
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
