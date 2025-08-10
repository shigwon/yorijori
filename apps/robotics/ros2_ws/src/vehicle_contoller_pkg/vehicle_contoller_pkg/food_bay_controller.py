#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

from adafruit_servokit import ServoKit
import board
import busio


class FoodBayControllerNode(Node):
    def __init__(self):
        super().__init__('food_bay_controller_node')

        # ===== Parameters =====
        self.declare_parameter('i2c_address', 0x60)
        self.declare_parameter('channels', [1, 2, 3])     # 사용 베이 채널 목록
        self.declare_parameter('angle_open', 90.0)        # 열림 각도 (90~180)
        self.declare_parameter('angle_closed', 180.0)     # 닫힘 각도 (90~180)
        self.declare_parameter('startup_open', True)      # 시작/종료 시 열림 상태로 둘지
        self.declare_parameter('log_debug', False)        # 잦은 로그를 debug로만

        self.addr = int(self.get_parameter('i2c_address').value)
        self.channels = list(self.get_parameter('channels').value)
        self.angle_open = float(self.get_parameter('angle_open').value)
        self.angle_closed = float(self.get_parameter('angle_closed').value)
        self.startup_open = bool(self.get_parameter('startup_open').value)
        self.log_debug = bool(self.get_parameter('log_debug').value)

        # 각도 안전 범위 및 관계 보정
        self.angle_open = self._limit_angle(self.angle_open)
        self.angle_closed = self._limit_angle(self.angle_closed)
        if self.angle_open >= self.angle_closed:
            # 열림(작은 각) < 닫힘(큰 각) 되도록 보정
            self.get_logger().warn(
                f"angle_open({self.angle_open}) >= angle_closed({self.angle_closed}) → 강제 보정"
            )
            self.angle_open = 90.0
            self.angle_closed = 180.0

        # 상태 캐시: 베이별 마지막 state(0/1) 저장 → 중복동작 방지
        self._last_state = {int(ch): None for ch in self.channels}

        # ===== HW Init =====
        self.hardware_ready = False
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.servo = ServoKit(channels=16, i2c=i2c, address=self.addr).servo
            # 시작자세 설정
            init_angle = self.angle_open if self.startup_open else self.angle_closed
            for ch in self.channels:
                self._set_angle(ch, init_angle)
                self._last_state[int(ch)] = 1 if self.startup_open else 0
            self.hardware_ready = True
            self.get_logger().info(f'PCA9685 ready @0x{self.addr:02X}, startup={"OPEN" if self.startup_open else "CLOSE"}')
        except Exception as e:
            self.get_logger().error(f'Hardware init failed: {e}')
            self.get_logger().warn('Running in no-hardware mode. Commands will be ignored.')

        # ===== Sub =====
        self.sub = self.create_subscription(Int32MultiArray, '/food_bay_cmd', self.cmd_cb, 10)
        self.get_logger().info("FoodBayControllerNode initialized.")

    # --------- Helpers ----------
    def _limit_angle(self, val: float) -> float:
        return max(90.0, min(180.0, float(val)))

    def _valid_bay(self, bay: int) -> bool:
        return bay in self._last_state

    def _apply(self, bay: int, state: int):
        """state: 1=open → angle_open, 0=close → angle_closed (중복이면 스킵)"""
        if not self._valid_bay(bay):
            self.get_logger().warn(f'Invalid bay {bay}, valid={self.channels}')
            return
        if state not in (0, 1):
            self.get_logger().warn('state must be 0(close) or 1(open)')
            return
        if self._last_state.get(bay) == state:
            if self.log_debug:
                self.get_logger().debug(f'[FoodBay] bay={bay} state={state} (no-op)')
            return

        target = self.angle_open if state == 1 else self.angle_closed
        self._set_angle(bay, target)
        self._last_state[bay] = state
        self.get_logger().info(f'[FoodBay] bay={bay} -> {"OPEN" if state==1 else "CLOSE"} ({target}°)')

    def _set_angle(self, channel: int, angle: float):
        if not self.hardware_ready:
            return
        a = self._limit_angle(angle)
        try:
            self.servo[int(channel)].angle = a
            if self.log_debug:
                self.get_logger().debug(f'set ch{channel} -> {a}°')
        except Exception as e:
            self.get_logger().error(f'set angle ch{channel} failed: {e}')

    # --------- Callback ----------
    def cmd_cb(self, msg: Int32MultiArray):
        data = list(msg.data) if msg.data is not None else []
        if len(data) < 2:
            self.get_logger().warn('Need [bay, state] or [b1,s1,b2,s2,...]')
            return

        # 2개씩 끊어서 처리 (여분은 무시)
        pairs = len(data) // 2
        for i in range(pairs):
            bay = int(data[2*i])
            state = int(data[2*i + 1])
            self._apply(bay, state)

    # --------- Shutdown ----------
    def destroy_node(self):
        if self.hardware_ready:
            try:
                # 종료 자세 유지: 파라미터에 따라 열림/닫힘 유지
                final_angle = self.angle_open if self.startup_open else self.angle_closed
                for ch in self.channels:
                    self._set_angle(ch, final_angle)
                self.get_logger().info(f'Set all bays to {"OPEN" if self.startup_open else "CLOSE"} on shutdown')
            except Exception as e:
                self.get_logger().error(f'Shutdown positioning failed: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FoodBayControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
