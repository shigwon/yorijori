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

        # ===== Parameters =====
        self.declare_parameter('servo_addr', 0x60)
        self.declare_parameter('throttle_addr', 0x40)
        self.declare_parameter('pwm_freq', 60)

        self.declare_parameter('ch_in1', 3)
        self.declare_parameter('ch_in2', 4)
        self.declare_parameter('ch_ena', 5)
        self.declare_parameter('steer_channel', 0)

        self.declare_parameter('steer_center_deg', 70.0)
        self.declare_parameter('steer_max_delta_deg', 30.0)  # 서보가 좌/우로 최대 움직일 각도
        self.declare_parameter('logical_angle_range', 30.0)  # 들어오는 logical_angle의 ±범위

        self.declare_parameter('soft_start_ms', 400)         # drive_enable True 직후 속도 램프업 시간(ms)
        self.declare_parameter('log_control_debug', False)   # 잦은 로그를 debug로만 찍기

        # Read params
        self.servo_addr = int(self.get_parameter('servo_addr').value)
        self.throttle_addr = int(self.get_parameter('throttle_addr').value)
        self.pwm_freq = int(self.get_parameter('pwm_freq').value)

        self.in1 = int(self.get_parameter('ch_in1').value)
        self.in2 = int(self.get_parameter('ch_in2').value)
        self.ena = int(self.get_parameter('ch_ena').value)
        self.steer_ch = int(self.get_parameter('steer_channel').value)

        self.STEER_CENTER = float(self.get_parameter('steer_center_deg').value)
        self.STEER_MAX_DELTA = float(self.get_parameter('steer_max_delta_deg').value)
        self.LOGICAL_RANGE = max(1e-3, float(self.get_parameter('logical_angle_range').value))

        self.SOFT_START_MS = int(self.get_parameter('soft_start_ms').value)
        self.LOG_CONTROL_DEBUG = bool(self.get_parameter('log_control_debug').value)

        # ===== State =====
        self.hardware_ready = False
        self.drive_enabled = True
        self.enable_ts = 0.0  # epoch seconds when drive_enable became True

        # ===== HW Init =====
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.servo_kit = ServoKit(channels=16, i2c=i2c, address=self.servo_addr)
            self.pca_throttle = PCA9685(i2c, address=self.throttle_addr)
            self.pca_throttle.frequency = self.pwm_freq

            # 초기 안전 상태
            self._motor_stop()
            self._set_steer(self.STEER_CENTER)

            self.hardware_ready = True
            self.get_logger().info(f"Hardware initialized (servo=0x{self.servo_addr:02X}, throttle=0x{self.throttle_addr:02X}, freq={self.pwm_freq})")
        except Exception as e:
            self.get_logger().warn(f"Hardware not initialized (mock mode): {e}")

        # ===== Subs =====
        self.create_subscription(Float32MultiArray, 'vehicle_control', self.control_callback, 10)
        self.create_subscription(Bool, '/robot/drive_enable', self.enable_callback, 10)

        self.get_logger().info("Vehicle control node ready (parametrized + soft-start).")

    # ===== Mapping =====
    def logical_to_servo_angle(self, logical_angle: float) -> float:
        """
        logical_angle ∈ [-LOGICAL_RANGE, +LOGICAL_RANGE]
        → 물리 서보 각도 = CENTER + (logical_angle / LOGICAL_RANGE) * STEER_MAX_DELTA
        """
        ratio = float(logical_angle) / self.LOGICAL_RANGE
        angle = self.STEER_CENTER + ratio * self.STEER_MAX_DELTA
        return max(0.0, min(180.0, angle))

    # ===== Callbacks =====
    def enable_callback(self, msg: Bool):
        self.drive_enabled = bool(msg.data)
        self.get_logger().info(f"drive_enable = {self.drive_enabled}")

        if not self.hardware_ready:
            return

        if self.drive_enabled:
            # soft-start 시작 시간 기록
            self.enable_ts = time.time()
        else:
            # 즉시 정지 & 조향 중앙
            try:
                self._motor_stop()
                self._set_steer(self.STEER_CENTER)
                self.get_logger().info("Drive disabled → motor stop & steer center")
            except Exception as e:
                self.get_logger().error(f"Stop on disable failed: {e}")

    def control_callback(self, msg: Float32MultiArray):
        if len(msg.data) != 2:
            self.get_logger().error("Expected 2 values: [angle, speed]")
            return

        logical_angle, speed = msg.data
        if self.LOG_CONTROL_DEBUG:
            self.get_logger().debug(f"Received control: angle={logical_angle}, speed={speed}")
        else:
            # 가끔만 찍고 싶으면 아래 주석을 해제해 샘플링 로깅 로직을 추가해도 됨
            pass

        if not self.hardware_ready:
            self.get_logger().warn("Hardware not available. Skipping control.")
            return

        # 조향: 항상 반영 (원하면 drive_enabled일 때만 반영하도록 바꿀 수 있음)
        try:
            servo_angle = self.logical_to_servo_angle(float(logical_angle))
            self._set_steer(servo_angle)
            if self.LOG_CONTROL_DEBUG:
                self.get_logger().debug(f"Steer -> {servo_angle:.1f} deg")
        except Exception as e:
            self.get_logger().error(f"Servo error: {e}")

        # 구동: drive_enabled로 게이팅 + soft-start
        try:
            if not self.drive_enabled:
                self._motor_stop()
                if self.LOG_CONTROL_DEBUG:
                    self.get_logger().debug("Drive disabled → throttle forced to 0")
            else:
                self._motor_drive(self._soft_start_limited_speed(float(speed)))
        except Exception as e:
            self.get_logger().error(f"Motor error: {e}")

    # ===== Soft-start =====
    def _soft_start_limited_speed(self, target_speed: float) -> float:
        """
        drive_enable True 직후 SOFT_START_MS 동안 속도를 선형 램프업.
        speed ∈ [-1.0, +1.0]
        """
        target_speed = max(-1.0, min(1.0, target_speed))
        if self.SOFT_START_MS <= 0:
            return target_speed

        elapsed_ms = (time.time() - self.enable_ts) * 1000.0
        if elapsed_ms <= 0:
            return 0.0

        ramp_ratio = min(1.0, elapsed_ms / float(self.SOFT_START_MS))
        # 부호를 보존한 채로 램프 제한
        limited = target_speed * ramp_ratio
        return limited

    # ===== low-level helpers =====
    def _set_steer(self, angle: float):
        if not self.hardware_ready:
            return
        a = max(0.0, min(180.0, float(angle)))
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
        speed = max(-1.0, min(1.0, speed))
        pulse = int(0xFFFF * abs(speed))
        if speed > 0:
            # 반대로: 전진 명령일 때 in1=0, in2=최대
            self.pca_throttle.channels[self.in1].duty_cycle = 0
            self.pca_throttle.channels[self.in2].duty_cycle = 0xFFFF
        elif speed < 0:
            # 반대로: 후진 명령일 때 in1=최대, in2=0
            self.pca_throttle.channels[self.in1].duty_cycle = 0xFFFF
            self.pca_throttle.channels[self.in2].duty_cycle = 0

        else:
            self.pca_throttle.channels[self.in1].duty_cycle = 0
            self.pca_throttle.channels[self.in2].duty_cycle = 0
        self.pca_throttle.channels[self.ena].duty_cycle = pulse
        if self.LOG_CONTROL_DEBUG:
            self.get_logger().debug(f"Throttle -> {speed:.2f} (pulse=0x{pulse:04X})")

    # ===== teardown =====
    def destroy_node(self):
        if self.hardware_ready:
            try:
                self._motor_stop()
                self.pca_throttle.deinit()
                self.get_logger().info("PWM deinitialized.")
            except Exception as e:
                self.get_logger().error(f"Deinit error: {e}")
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



'''
ros2 topic pub /vehicle_control std_msgs/Float32MultiArray "{data: [0,1]}"
  

  '''
