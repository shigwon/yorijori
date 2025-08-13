#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class InputTeleop(Node):
    def __init__(self):
        super().__init__('input_drive_teleop')
        self.pub_ctrl = self.create_publisher(Float32MultiArray, 'vehicle_control', 10)

    def run(self):
        while rclpy.ok():
            try:
                cmd = input("\nangle speed%(−100~100) 입력 (종료:q): ").strip()
                if not cmd:
                    continue
                if cmd.lower() in ['q', 'quit', 'exit']:
                    break

                parts = cmd.split()
                if len(parts) != 2:
                    print("⚠ 두 개 값(angle speed%)만 입력하세요")
                    continue

                angle = float(parts[0])
                speed_percent = float(parts[1])

                # 속도 퍼센트 범위: -100 ~ 100으로 제한
                if speed_percent < -100.0:
                    speed_percent = -100.0
                elif speed_percent > 100.0:
                    speed_percent = 100.01
                    

                # 내부 스케일 -1.0 ~ +1.0
                speed = speed_percent / 100.0

                msg = Float32MultiArray()
                msg.data = [angle, speed]
                self.pub_ctrl.publish(msg)
                self.get_logger().info(f"발행: angle={angle}, speed={speed:.2f} (입력 {speed_percent:.1f}%)")

            except Exception as e:
                self.get_logger().error(str(e))

def main():
    rclpy.init()
    node = InputTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
