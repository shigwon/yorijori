import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

import paho.mqtt.client as mqtt
import threading
import time
import json

MQTT_HOST = '192.168.100.83'
MQTT_PORT = 1883

class MQTTBridgeNode(Node):
    def __init__(self):
        super().__init__('mqtt_bridge_node')

        self.position_sub = self.create_subscription(
            Point,
            '/slam/position',
            self.slam_position_callback,
            10
        )

        self.mqtt_host = MQTT_HOST
        self.mqtt_port = MQTT_PORT

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
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
        else:
            self.get_logger().warn(f"MQTT connection failed with code {rc}")
            self._connected = False

    def on_disconnect(self, client, userdata, rc):
        self._connected = False
        self.get_logger().warn(f"Disconnected from MQTT broker (rc={rc})")

    def slam_position_callback(self, msg: Point):
        if not self._connected:
            self.get_logger().warn("MQTT not connected. Skipping slam position publish.")
            return

        payload = {
            "robotId": "0",
            "command": "updateLocation",
            "x": round(msg.x, 2),
            "y": round(msg.y, 2),
            "z": round(msg.z, 2)
        }

        topic = "linky/robot/0/updateLocation"

        self.get_logger().info(f"Publishing SLAM Position to MQTT: Topic={topic} Payload={payload}")
        self.mqtt_client.publish(topic, json.dumps(payload))

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