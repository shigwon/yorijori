"""
==============================================
 MQTT ↔ ROS2 프로토콜 요약
==============================================

[환경 정보]
- MQTT 브로커: 192.168.100.83:1883
- 로봇 ID: "1"
- 서버 ID: "0"
- 데이터 형식: JSON (UTF-8)

[토픽 네이밍 규칙]
- 서버 → 로봇: linky/robot/{ROBOT_ID}/...
- 로봇 → 서버: linky/robot/{SERVER_ID}/...

------------------------------------------------
 1. MQTT → ROS2 (서버 → 로봇)
------------------------------------------------
1) linky/robot/{ROBOT_ID}/closeSection
   - 특정 구역(section) 폐쇄 요청
   - ROS2 변환: /robot/close_section
   - 데이터 예시:
     {
       "sectionNum": 3,
       "sectionStatus": "CLOSED"
     }
   - ROS2 메시지 예시: "section=3,status=CLOSED"

2) linky/robot/{ROBOT_ID}/result
   - 작업 결과 전송
   - ROS2 변환: /server/request_result
   - 데이터 예시:
     {
       "result": "SUCCESS"
     }

3) linky/robot/{ROBOT_ID}/orderList
   - 주문 목록 전송
   - 처리 방식: 콘솔 로그 출력 (상위 3개만)
   - 데이터 예시:
     [
       {
         "orderId": "A100",
         "code": "X123",
         "tel": "010-0000-0000",
         "customerLatitude": 37.123,
         "customerLongitude": 127.456,
         "spaceNum": 5,
         "faceImageUrl": "http://..."
       },
       ...
     ]

------------------------------------------------
 2. ROS2 → MQTT (로봇 → 서버)
------------------------------------------------
1) /slam/position (geometry_msgs/Point)
   - SLAM 위치 정보
   - MQTT 변환: linky/robot/{SERVER_ID}/updateLocation
   - 데이터 예시:
     {
       "robotId": "1",
       "latitude": 37.123,
       "longitude": 127.456
     }

2) /robot/delivery_state (std_msgs/String)
   - 배송 상태 업데이트
   - MQTT 변환: linky/robot/{SERVER_ID}/updateDeliveryState
   - 데이터 예시:
     {
       "orderId": "1",
       "robotId": "1",
       "state": "DELIVERED"
     }

3) /robot/status (std_msgs/String)
   - 로봇 상태 업데이트
   - MQTT 변환: linky/robot/{SERVER_ID}/updateRobotStatus
   - 데이터 예시:
     {
       "robotId": "1",
       "status": "IDLE"
     }

------------------------------------------------
 3. 주의사항
------------------------------------------------
- 모든 MQTT 메시지는 JSON 직렬화 후 발행.
- 토픽 경로, robotId 값은 서버-로봇 간 사전 합의 필요.
- 네트워크 장애 대비 → MQTT 연결 재시도 로직(connect_loop) 사용.
"""


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

        self.sub_answer = self.create_subscription(String, '/webrtc/answer_sdp', self.ros2_answer_cb, 10)
        self.sub_ice_in = self.create_subscription(String, '/webrtc/ice_candidate', self.ros2_ice_cb, 10)
        
        # 자신의 ICE Candidate 토픽 구독
        self.sub_local_ice = self.create_subscription(String, '/ros2/ice-candidate', self.ros2_local_ice_cb, 10)
        # Offer 토픽 구독
        self.sub_offer = self.create_subscription(String, '/ros2/sdp/offer', self.ros2_offer_cb, 10)

        self.close_section_pub = self.create_publisher(String, '/robot/close_section', 10)
        self.server_robot_status_pub = self.create_publisher(String, '/server/update_robot_status', 10)
        self.result_pub = self.create_publisher(String, '/server/request_result', 10)

        self.webrtc_answer_pub = self.create_publisher(String, '/webrtc/answer_sdp', 10)
        self.webrtc_ice_pub = self.create_publisher(String, '/webrtc/ice_candidate', 10)


            
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_disconnect = self.on_disconnect
        self.mqtt_client.on_message = self.on_message

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
            client.subscribe(TOPIC_SERVER_TO_ROBOT + "/responseWebRTC")
            client.subscribe(TOPIC_SERVER_TO_ROBOT + "/orderList")
            client.subscribe("/openvidu/sdp/answer")
            client.subscribe("/openvidu/ice")

            self.request_webrtc_connection()

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
            elif topic.endswith("/responseWebRTC"):
                self.handle_webrtc_response(payload)
            elif topic.endswith("/orderList"):
                self.handle_orderList(payload)
            elif topic.endswith("/answer"):
                ros_msg = String()
                ros_msg.data = json.dumps(payload)
                self.webrtc_answer_pub.publish(ros_msg)
            elif topic.endswith("/ice"):
                ros_msg = String()
                ros_msg.data = json.dumps(payload)
                self.webrtc_ice_pub.publish(ros_msg)
            else:
                self.get_logger().warn(f"Unknown topic received: {topic}")
        except json.JSONDecodeError:
            self.get_logger().error("Failed to decode JSON payload.")
        except Exception as e:
            self.get_logger().error(f"Error processing MQTT message: {e}")

    def ros2_answer_cb(self, msg):
        self.get_logger().info(f"Publishing answer SDP to MQTT: {msg.data}")
        self.mqtt_client.publish("/ros2/sdp/answer", msg.data)

    def ros2_ice_cb(self, msg):
        self.get_logger().info(f"Publishing ICE candidate to MQTT: {msg.data}")
        self.mqtt_client.publish("/ros2/ice", msg.data)

    # Offer를 MQTT로 발행하는 콜백 함수
    def ros2_offer_cb(self, msg):
        self.get_logger().info(f"Publishing SDP offer to MQTT: {msg.data}")
        # 서버와 약속된 토픽으로 Offer를 발행해야 합니다.
        topic = TOPIC_ROBOT_TO_SERVER + "/sdp/offer"
        self.mqtt_client.publish(topic, msg.data)
        
    # 자신의 ICE Candidate를 MQTT로 발행하는 콜백 함수
    def ros2_local_ice_cb(self, msg):
        self.get_logger().info(f"Publishing local ICE candidate to MQTT: {msg.data}")
        # 서버와 약속된 토픽으로 ICE Candidate를 발행해야 합니다.
        topic = TOPIC_ROBOT_TO_SERVER + "/ice"
        self.mqtt_client.publish(topic, msg.data)
        
    def location_callback(self, msg: Point):
        if not self._connected: return

        payload = {
            "robotId": ROBOT_ID,
            "latitude": msg.x,
            "longitude": msg.y,
        }
        topic = TOPIC_ROBOT_TO_SERVER + "/updateLocation"
        self.mqtt_client.publish(topic, json.dumps(payload))

    def update_delivery_state_callback(self, msg: String):
        if not self._connected: return

        payload = {
            "orderId": "1",
            "robotId": ROBOT_ID,
            "state": msg.data
        }
        topic = TOPIC_ROBOT_TO_SERVER + "/updateDeliveryState"
        self.mqtt_client.publish(topic, json.dumps(payload))

    def update_robot_status_callback(self, msg: String):
        if not self._connected: return

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

    def handle_webrtc_response(self, payload):
        result = payload.get("result")
        data = payload.get("data")
        if result is not None and data is not None:
            self.get_logger().info(f"WebRTC Connection Response: result={result}, data={data}")
        else:
            self.get_logger().warn("Invalid WebRTC response payload.")

    def handle_orderList(self, payload):
        if not isinstance(payload, list):
            self.get_logger().warn("Invalid order list payload format (expected list).")
            return

        for i, order in enumerate(payload[:3], 1):
            order_id = order.get("orderId")
            code = order.get("code")
            tel = order.get("tel")
            lat = order.get("customerLatitude")
            lon = order.get("customerLongitude")
            space_num = order.get("spaceNum")
            face_url = order.get("faceImageUrl")
            self.get_logger().info(f"[Order {i}] ID={order_id}, Code={code}, Tel={tel}, Space={space_num}, Location=({lat}, {lon}), FaceURL={face_url}")

    def request_webrtc_connection(self):
        if not self._connected: return

        payload = {
            "robotId": int(ROBOT_ID),
            "role": "PUBLISHER"
        }
        topic = TOPIC_ROBOT_TO_SERVER + "/requestWebRTC"
        self.mqtt_client.publish(topic, json.dumps(payload))

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