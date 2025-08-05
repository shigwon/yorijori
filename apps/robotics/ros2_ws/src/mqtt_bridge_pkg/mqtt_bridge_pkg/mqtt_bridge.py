import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import paho.mqtt.client as mqtt
import threading

class MQTTBridgeNode(Node):
    def __init__(self):
        super().__init__('mqtt_bridge_node')

        # ROS2 퍼블리셔 (예: mqtt에서 받은 메시지 토픽)
        self.mqtt_to_ros_pub = self.create_publisher(String, 'mqtt_incoming', 10)

        # ROS2 구독자 (예: ROS2 메시지를 mqtt로 퍼블리시)
        self.ros_to_mqtt_sub = self.create_subscription(
            String,
            'mqtt_outgoing',
            self.ros_to_mqtt_callback,
            10)

        # MQTT 클라이언트 설정
        self.mqtt_client = mqtt.Client()

        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        self.mqtt_host = '192.168.100.83'  # MQTT 브로커 주소
        self.mqtt_port = 1883

        # MQTT 연결 및 별도 스레드로 loop 시작
        self.mqtt_client.connect(self.mqtt_host, self.mqtt_port)
        self.mqtt_thread = threading.Thread(target=self.mqtt_client.loop_forever)
        self.mqtt_thread.daemon = True
        self.mqtt_thread.start()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f'Connected to MQTT broker with result code {rc}')
        # MQTT에서 구독할 토픽 설정
        client.subscribe('linky/robot/1/test')

    def on_message(self, client, userdata, msg):
        message = msg.payload.decode('utf-8')
        self.get_logger().info(f'MQTT Received: Topic={msg.topic} Message={message}')
        # 받은 MQTT 메시지를 ROS2 토픽으로 퍼블리시
        ros_msg = String()
        ros_msg.data = message
        self.mqtt_to_ros_pub.publish(ros_msg)

    def ros_to_mqtt_callback(self, msg):
        # ROS2에서 받은 메시지를 MQTT로 퍼블리시
        self.get_logger().info(f'Publishing to MQTT: {msg.data}')
        self.mqtt_client.publish('linky/robot/1/test', msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = MQTTBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.get_logger().info('Shutting down MQTT bridge node...')
    node.mqtt_client.disconnect()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
