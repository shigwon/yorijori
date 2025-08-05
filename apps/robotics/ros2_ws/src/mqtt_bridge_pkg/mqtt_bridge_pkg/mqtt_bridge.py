import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import String # 예시: 문자열 메시지 사용
from geometry_msgs.msg import Point # 예시: 위치 메시지 사용

import paho.mqtt.client as mqtt
import threading
import time
import json

MQTT_HOST = '192.168.100.83'
MQTT_PORT = 1883

ROBOT_ID = "0"
TOPIC_SERVER_TO_ROBOT = f"server/robot/{ROBOT_ID}/command"
TOPIC_ROBOT_TO_SERVER = f"robot/{ROBOT_ID}/status"

class MQTTBridgeNode(Node):
    def __init__(self):
        super().__init__('mqtt_bridge_node')

        self.get_logger().info(f"Starting MQTT Bridge Node for Robot ID: {ROBOT_ID}")
        
        self.location_sub = self.create_subscription(
            Point,
            '/slam/position',
            self.location_callback,
            10
        )
        
        self.delivery_state_sub = self.create_subscription(
            String,
            '/robot/delivery_state',
            self.update_delivery_state_callback,
            10
        )

        self.robot_status_sub = self.create_subscription(
            String,
            '/robot/status',
            self.update_robot_status_callback,
            10
        )

        self.close_section_pub = self.create_publisher(String, '/robot/close_section', 10)
        self.server_robot_status_pub = self.create_publisher(String, '/server/update_robot_status', 10)
        self.result_pub = self.create_publisher(String, '/server/request_result', 10)

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_disconnect = self.on_disconnect
        self.mqtt_client.on_message = self.on_message # MQTT 메시지 콜백 추가
        
        self._connected = False
        self._stop_event = threading.Event()
        self.connection_thread = threading.Thread(target=self.connect_loop)
        self.connection_thread.daemon = True
        self.connection_thread.start()

    def connect_loop(self):
        while not self._stop_event.is_set():
            if not self._connected:
                try:
                    self.get_logger().info(f"Attempting to connect to MQTT broker at {self.mqtt_host}:{self.mqtt_port}")
                    self.mqtt_client.connect(MQTT_HOST, MQTT_PORT)
                    self.mqtt_client.loop_start()
                except Exception as e:
                    self.get_logger().warn(f"MQTT connection failed: {e}")
            time.sleep(5)

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Connected to MQTT broker successfully.")
            self._connected = True
            # 연결 성공 시 MQTT 토픽 구독
            client.subscribe(TOPIC_SERVER_TO_ROBOT)
            self.get_logger().info(f"Subscribed to MQTT topic: {TOPIC_SERVER_TO_ROBOT}")
        else:
            self.get_logger().error(f"MQTT connection failed with code {rc}")
            self._connected = False

    def on_disconnect(self, client, userdata, rc):
        self._connected = False
        self.get_logger().warn(f"Disconnected from MQTT broker (rc={rc})")

    def on_message(self, client, userdata, msg):
        """ MQTT 메시지가 도착했을 때 실행되는 콜백 함수 """
        try:
            payload = json.loads(msg.payload.decode('utf-8'))
            command = payload.get('command')
            
            self.get_logger().info(f"Received MQTT command: {command} with payload: {payload}")

            if command == 'closeSection':
                self.handle_close_section(payload)
            elif command == 'updateRobotStatus':
                self.handle_update_robot_status(payload)
            elif command == 'result':
                self.handle_result(payload)
            else:
                self.get_logger().warn(f"Unknown command received: {command}")
        except json.JSONDecodeError:
            self.get_logger().error("Failed to decode JSON payload.")
        except Exception as e:
            self.get_logger().error(f"Error processing MQTT message: {e}")

    def location_callback(self, msg: Point):
        if not self._connected: return

        payload = {
            "orderId": "undefined", ########### juno 이거 말해봐야 할 듯.
            "robotId": ROBOT_ID,
            "customerLatitude": msg.x,
            "customerLongitude": msg.y,
        }
        topic = f"robot/{ROBOT_ID}/updateLocation"
        
        self.mqtt_client.publish(topic, json.dumps(payload))
        self.get_logger().info(f"Published location to MQTT: {payload}")

    def update_delivery_state_callback(self, msg: String):
        if not self._connected: return
        
        payload = {
            "orderId": "undefined", # ROS 2 메시지에 orderId가 포함될 경우 수정
            "robotId": ROBOT_ID,
            "state": msg.data
        }
        topic = f"robot/{ROBOT_ID}/updateDeliveryState"
        
        self.mqtt_client.publish(topic, json.dumps(payload))
        self.get_logger().info(f"Published delivery state to MQTT: {payload}")

    def update_robot_status_callback(self, msg: String):
        if not self._connected: return

        # 서버 -> 로봇 status와 혼동되지 않도록 토픽을 분리
        payload = {
            "robotId": ROBOT_ID,
            "status": msg.data
        }
        topic = f"robot/{ROBOT_ID}/status"
        
        self.mqtt_client.publish(topic, json.dumps(payload))
        self.get_logger().info(f"Published robot status to MQTT: {payload}")

    # ======================================================================
    # MQTT 메시지를 ROS 2 메시지로 발행하는 핸들러
    # ======================================================================

    def handle_close_section(self, payload):
        """closeSection 명령을 처리하고 ROS 2 토픽으로 발행"""
        section_num = payload.get("sectionNum")
        section_status = payload.get("sectionStatus")
        
        if section_num is not None and section_status is not None:
            msg = String()
            msg.data = f"section={section_num},status={section_status}"
            self.close_section_pub.publish(msg)
            self.get_logger().info(f"Published to ROS 2 topic '/robot/close_section': {msg.data}")

    def handle_update_robot_status(self, payload):
        """updateRobotStatus 명령을 처리하고 ROS 2 토픽으로 발행"""
        robot_id = payload.get("robotId")
        status = payload.get("status")
        
        if robot_id == ROBOT_ID and status is not None:
            msg = String()
            msg.data = f"robot_id={robot_id},status={status}"
            self.server_robot_status_pub.publish(msg)
            self.get_logger().info(f"Published to ROS 2 topic '/server/update_robot_status': {msg.data}")

    def handle_result(self, payload):
        """result 명령을 처리하고 ROS 2 토픽으로 발행"""
        result = payload.get("result")
        
        if result is not None:
            msg = String()
            msg.data = str(result)
            self.result_pub.publish(msg)
            self.get_logger().info(f"Published to ROS 2 topic '/server/request_result': {msg.data}")

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