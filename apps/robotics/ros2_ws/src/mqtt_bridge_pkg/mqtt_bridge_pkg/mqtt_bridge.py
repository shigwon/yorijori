import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point

import paho.mqtt.client as mqtt
import threading
import time
import json

MQTT_HOST = '192.168.100.83'
MQTT_PORT = 1883

ROBOT_ID = "1"
SERVER_ID = "0"
TOPIC_SERVER_TO_ROBOT = f"linky/robot/{ROBOT_ID}"
TOPIC_ROBOT_TO_SERVER = f"linky/robot/{SERVER_ID}"

class MQTTBridgeNode(Node):
    def __init__(self):
        super().__init__('mqtt_bridge_node')

        self.mqtt_host = MQTT_HOST
        self.mqtt_port = MQTT_PORT

        self.get_logger().info(f"Starting MQTT Bridge Node for Robot ID: {ROBOT_ID}")

        self.location_sub = self.create_subscription(Point, '/slam/position', self.location_callback, 10)
        self.delivery_state_sub = self.create_subscription(String, '/robot/delivery_state', self.update_delivery_state_callback, 10)
        self.robot_status_sub = self.create_subscription(String, '/robot/status', self.update_robot_status_callback, 10)

        self.close_section_pub = self.create_publisher(String, '/robot/close_section', 10)
        self.result_pub = self.create_publisher(String, '/server/request_result', 10)
        self.face_db_register_pub = self.create_publisher(String, '/face_db/register', 10)
        self.orders_pub = self.create_publisher(String, '/server/order_list', 10)


        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_disconnect = self.on_disconnect
        self.mqtt_client.on_message = self.on_message

        self._connected = False
        self._stop_event = threading.Event()
        self.connection_thread = threading.Thread(target=self.connect_loop, daemon=True)
        self.connection_thread.start()

    def connect_loop(self):
        while not self._stop_event.is_set():
            if not self._connected:
                try:
                    self.get_logger().info(f"Attempting to connect to MQTT broker at {self.mqtt_host}:{self.mqtt_port}")
                    self.mqtt_client.connect(self.mqtt_host, self.mqtt_port)
                    self.mqtt_client.loop_start()
                except Exception as e:
                    self.get_logger().warn(f"MQTT connection failed: {e}")
            time.sleep(5)

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Connected to MQTT broker successfully.")
            self._connected = True
            client.subscribe(TOPIC_SERVER_TO_ROBOT + "/closeSection")
            client.subscribe(TOPIC_SERVER_TO_ROBOT + "/result")
            client.subscribe(TOPIC_SERVER_TO_ROBOT + "/orderList")
        else:
            self.get_logger().error(f"MQTT connection failed with code {rc}")
            self._connected = False

    def on_disconnect(self, client, userdata, rc):
        self._connected = False
        self.get_logger().warn(f"Disconnected from MQTT broker (rc={rc})")

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode('utf-8'))
            topic = msg.topic
            self.get_logger().info(f"Received MQTT message on topic: {topic} with payload: {payload}")

            if topic.endswith("/closeSection"):
                self.handle_close_section(payload)
            elif topic.endswith("/result"):
                self.handle_result(payload)
            elif topic.endswith("/orderList"):
                self.handle_orderList(payload)
            else:
                self.get_logger().warn(f"Unknown topic received: {topic}")
        except json.JSONDecodeError:
            self.get_logger().error("Failed to decode JSON payload.")
        except Exception as e:
            self.get_logger().error(f"Error processing MQTT message: {e}")

    def location_callback(self, msg: Point):
        if not self._connected:
            return
        payload = {
            "robotId": ROBOT_ID,
            "latitude": msg.x,
            "longitude": msg.y,
        }
        topic = TOPIC_ROBOT_TO_SERVER + "/updateLocation"
        self.mqtt_client.publish(topic, json.dumps(payload))

    def update_delivery_state_callback(self, msg: String):
        if not self._connected:
            return
        payload = {
            "orderId": "1",
            "robotId": ROBOT_ID,
            "state": msg.data
        }
        topic = TOPIC_ROBOT_TO_SERVER + "/updateDeliveryState"
        self.mqtt_client.publish(topic, json.dumps(payload))

    def update_robot_status_callback(self, msg: String):
        if not self._connected:
            return
        payload = {
            "robotId": ROBOT_ID,
            "status": msg.data
        }
        topic = TOPIC_ROBOT_TO_SERVER + "/updateRobotStatus"
        self.mqtt_client.publish(topic, json.dumps(payload))

    def handle_close_section(self, payload):
        section_num = payload.get("sectionNum")
        section_status = payload.get("sectionStatus")
        if section_num is not None and section_status is not None:
            msg = String()
            msg.data = f"section={section_num},status={section_status}"
            self.close_section_pub.publish(msg)

    def handle_result(self, payload):
        result = payload.get("result")
        if result is not None:
            msg = String()
            msg.data = str(result)
            self.result_pub.publish(msg)

    # 바꾼 handle_orderList
    def handle_orderList(self, payload):
        if not isinstance(payload, list):
            self.get_logger().warn("Invalid order list payload format (expected list).")
            return

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.orders_pub.publish(msg)

    def destroy_node(self):
        self._stop_event.set()
        if self.connection_thread.is_alive():
            self.connection_thread.join()
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
    finally:
        node.get_logger().info('Shutting down MQTT bridge node...')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
