import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import paho.mqtt.client as mqtt
import threading
import time

class MQTTBridgeNode(Node):
    def __init__(self):
        super().__init__('mqtt_bridge_node')

        self.mqtt_to_ros_pub = self.create_publisher(String, 'mqtt_incoming', 10)

        self.ros_to_mqtt_sub = self.create_subscription(
            String,
            'mqtt_outgoing',
            self.ros_to_mqtt_callback,
            10)

        self.mqtt_host = '192.168.100.83'
        self.mqtt_port = 1883

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.on_disconnect = self.on_disconnect

        self._connected = False
        self._stop_event = threading.Event()

        self.connection_thread = threading.Thread(target=self.connect_loop)
        self.connection_thread.daemon = True
        self.connection_thread.start()

    def connect_loop(self):
        while not self._stop_event.is_set():
            if not self._connected:
                try:
                    self.get_logger().info("Attempting to connect to MQTT broker...")
                    self.mqtt_client.connect(self.mqtt_host, self.mqtt_port)
                    self.mqtt_client.loop_start()
                except Exception as e:
                    self.get_logger().warn(f"MQTT connection failed: {e}")
                    time.sleep(5)
            else:
                time.sleep(1)

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Connected to MQTT broker successfully.")
            self._connected = True
            client.subscribe('linky/robot/1/test')
        else:
            self.get_logger().warn(f"MQTT connection failed with code {rc}")
            self._connected = False

    def on_disconnect(self, client, userdata, rc):
        self._connected = False
        self.get_logger().warn(f"Disconnected from MQTT broker (rc={rc})")

    def on_message(self, client, userdata, msg):
        message = msg.payload.decode('utf-8')
        self.get_logger().info(f"MQTT Received: Topic={msg.topic} Message={message}")
        ros_msg = String()
        ros_msg.data = message
        self.mqtt_to_ros_pub.publish(ros_msg)

    def ros_to_mqtt_callback(self, msg):
        if self._connected:
            self.get_logger().info(f"Publishing to MQTT: {msg.data}")
            self.mqtt_client.publish('linky/robot/1/test', msg.data)
        else:
            self.get_logger().warn("MQTT not connected. Skipping publish.")

    def destroy_node(self):
        self._stop_event.set()
        if self._connected:
            self.mqtt_client.disconnect()
            self.mqtt_client.loop_stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MQTTBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.get_logger().info('Shutting down MQTT bridge node...')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
