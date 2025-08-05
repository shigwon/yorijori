import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from adafruit_servokit import ServoKit
import board
import busio

class FoodBayControllerNode(Node):
    def __init__(self):
        super().__init__('food_bay_controller_node')
        
        self.get_logger().info('Initializing Food Bay Controller Node...')
        
        self.hardware_ready = False

        try:
            # I2C 버스 초기화
            i2c = busio.I2C(board.SCL, board.SDA)
            
            # PCA9685 보드 초기화. 주행 컨트롤러와 동일한 주소(0x60)를 사용해도 됨
            # 다른 주소의 PCA9685를 사용한다면 address를 변경
            self.servo_kit = ServoKit(channels=16, i2c=i2c, address=0x60)
            
            # 1, 2, 3번 채널 서보모터 초기화
            for channel in [1, 2, 3]:
                self.servo_kit.servo[channel].angle = 90
                self.get_logger().info(f'Servo channel {channel} initialized to 90 degrees.')

            self.hardware_ready = True
            self.get_logger().info('Hardware initialized.')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize ServoKit: {e}')

        # 각 음식칸 서보모터에 대한 구독 설정
        self.sub_lid1 = self.create_subscription(
            Float32,
            '/food_bay_1_angle',
            lambda msg: self.set_servo_angle(1, msg.data),
            10
        )
        self.sub_lid2 = self.create_subscription(
            Float32,
            '/food_bay_2_angle',
            lambda msg: self.set_servo_angle(2, msg.data),
            10
        )
        self.sub_lid3 = self.create_subscription(
            Float32,
            '/food_bay_3_angle',
            lambda msg: self.set_servo_angle(3, msg.data),
            10
        )

    def set_servo_angle(self, channel, angle):
        if not self.hardware_ready:
            self.get_logger().warn("Hardware not available. Skipping servo control.")
            return

        angle = max(0.0, min(180.0, angle))
        
        self.get_logger().info(f'Setting servo angle for channel {channel} to: {angle}')
        
        try:
            self.servo_kit.servo[channel].angle = angle
        except Exception as e:
            self.get_logger().error(f'Failed to set angle for channel {channel}: {e}')

    def destroy_node(self):
        self.get_logger().info('Shutting down food bay controller node...')
        # 노드 종료 시 모든 서보모터 각도를 중앙으로 설정
        if self.hardware_ready:
            for channel in [1, 2, 3]:
                self.servo_kit.servo[channel].angle = 90
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