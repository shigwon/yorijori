#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from adafruit_servokit import ServoKit

# ===== 고정 설정 =====
I2C_ADDRESS   = 0x60
CHANNELS      = [1, 2, 3]     # 사용 베이(=서보 채널) 목록
SERVO_MIN_US  = 500
SERVO_MAX_US  = 2500
ANGLE_OPEN    = 0.0           # 열림 각도
ANGLE_CLOSED  = 90.0          # 닫힘 각도
STARTUP_OPEN  = True         # 시작 시 열림 상태로 둘지

class FoodBayControllerNode(Node):
    def __init__(self):
        super().__init__('food_bay_controller_node')

        self.addr         = I2C_ADDRESS
        self.channels     = list(CHANNELS)
        self.min_us       = SERVO_MIN_US
        self.max_us       = SERVO_MAX_US
        self.ang_open     = float(ANGLE_OPEN)
        self.ang_closed   = float(ANGLE_CLOSED)
        self.startup_open = bool(STARTUP_OPEN)

        self.kit = ServoKit(channels=16, address=self.addr)
        for ch in self.channels:
            self.kit.servo[ch].set_pulse_width_range(self.min_us, self.max_us)
            self.kit.servo[ch].actuation_range = 180

        init_angle = self.ang_open if self.startup_open else self.ang_closed
        for ch in self.channels:
            self.kit.servo[ch].angle = float(init_angle)

        self.sub_txt = self.create_subscription(
            String, '/food_bay_cmd', self.on_cmd_text, 10
        )

    def on_cmd_text(self, msg: String):
        text = (msg.data or "").strip().upper()
        parts = text.split()
        if len(parts) < 2:
            self.get_logger().warning(f'Bad command: "{text}" (format: "<bay> OPEN|CLOSE")')
            return
        try:
            bay = int(parts[0])
        except Exception:
            self.get_logger().warning(f'Invalid bay number: {parts[0]}')
            return
        cmd = parts[1]
        if cmd == "OPEN":
            self._apply_cmd(bay, True)
        elif cmd == "CLOSE":
            self._apply_cmd(bay, False)
        else:
            self.get_logger().warning(f'Unknown command: "{cmd}" (use OPEN or CLOSE)')

    def _apply_cmd(self, bay: int, is_open: bool):
        target_angle = self.ang_open if is_open else self.ang_closed
        try:
            self.kit.servo[bay].angle = float(target_angle)
        except Exception as e:
            self.get_logger().error(f'Failed to move bay {bay}: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = FoodBayControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for ch in node.channels:
            node.kit.servo[ch].angle = None
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
