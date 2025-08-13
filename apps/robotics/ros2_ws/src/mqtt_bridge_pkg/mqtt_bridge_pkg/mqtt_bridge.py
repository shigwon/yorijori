#!/usr/bin/env python3
import threading
import time
import json
import base64

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import paho.mqtt.client as mqtt
import cv2

# ===== 고정 설정 =====
MQTT_HOST = '192.168.100.83'
MQTT_PORT = 1883

ROBOT_ID = "1"
SERVER_ID = "0"

# 서버가 구독하는 네임스페이스(서버 ID 기준)
TOPIC_ROBOT_TO_SERVER = f"linky/robot/{SERVER_ID}"
# 서버가 로봇 개별 제어용으로 발행하는 네임스페이스(로봇 ID 기준)
TOPIC_SERVER_TO_ROBOT = f"linky/robot/{ROBOT_ID}"


class MQTTBridgeNode(Node):
    def __init__(self):
        super().__init__('mqtt_bridge_node')

        # 고정 설정(ROS 파라미터 사용 안 함)
        self.camera_topic      = "/left/image_rect"
        self.gps_topic         = "/gps_ll_out"   # PoseConverterMiniStatic 출력: Point(x=lat, y=lon)
        self.publish_fps       = 15.0
        self.jpeg_quality      = 80
        self.max_width         = 320
        self.desired_encoding  = "mono8"
        self.warmup_frames     = 5

        # 내부 상태
        self.bridge            = CvBridge()
        self.last_pub_ts       = 0.0
        self.min_period        = 1.0 / max(0.1, self.publish_fps)

        self._connected        = False
        self._loop_started     = False
        self._stop_event       = threading.Event()
        self.recv_count        = 0
        self.ready_to_stream   = False
        self.last_loc_pub_ts   = 0.0   # 위치 MQTT 발행 스로틀용

        # ROS pub/sub
        self.gps_sub = self.create_subscription(Point, self.gps_topic, self.gps_callback, 10)
        self.delivery_state_sub = self.create_subscription(String, '/robot/delivery_state', self.update_delivery_state_callback, 10)
        self.robot_status_sub   = self.create_subscription(String, '/robot/status',         self.update_robot_status_callback, 10)
        self.image_sub = self.create_subscription(Image, self.camera_topic, self.image_callback, qos_profile_sensor_data)

        self.result_pub        = self.create_publisher(String, '/server/request_result', 10)
        self.orders_pub        = self.create_publisher(String, '/server/order_list', 10)
        self.food_bay_pub      = self.create_publisher(Int32MultiArray, '/food_bay_cmd', 10)

        # 카메라 퍼블리셔 존재 확인 타이머
        self.pub_check_timer = self.create_timer(0.5, self.check_cam_publisher)

        # MQTT
        self.mqtt_host = MQTT_HOST
        self.mqtt_port = MQTT_PORT

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect    = self.on_connect
        self.mqtt_client.on_disconnect = self.on_disconnect
        self.mqtt_client.on_message    = self.on_message
        self.mqtt_client.max_inflight_messages_set(20)
        self.mqtt_client.max_queued_messages_set(0)
        self.mqtt_client.enable_logger()

        self.get_logger().info(f"Starting MQTT Bridge Node for Robot ID: {ROBOT_ID}")

        # 연결 스레드 시작
        self.connection_thread = threading.Thread(target=self.connect_loop, daemon=True)
        self.connection_thread.start()

    # ---------- MQTT ----------
    def connect_loop(self):
        while not self._stop_event.is_set():
            if not self._connected:
                try:
                    self.get_logger().info(f"Attempting MQTT connect to {self.mqtt_host}:{self.mqtt_port}")
                    self.mqtt_client.connect(self.mqtt_host, self.mqtt_port)
                    if not self._loop_started:
                        self.mqtt_client.loop_start()
                        self._loop_started = True
                except Exception as e:
                    self.get_logger().warning(f"MQTT connection failed: {e}")
            time.sleep(5)

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Connected to MQTT broker.")
            self._connected = True
            # 서버→로봇 제어 명령 구독
            client.subscribe(TOPIC_SERVER_TO_ROBOT + "/closeSection")
            client.subscribe(TOPIC_SERVER_TO_ROBOT + "/result")
            client.subscribe(TOPIC_SERVER_TO_ROBOT + "/orderList")
        else:
            self.get_logger().error(f"MQTT connection failed (rc={rc})")
            self._connected = False

    def on_disconnect(self, client, userdata, rc):
        self._connected = False
        self.get_logger().warning(f"Disconnected from MQTT broker (rc={rc})")

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode('utf-8'))
            topic = msg.topic
            if topic.endswith("/closeSection"):
                self.handle_food_bay(payload)
            elif topic.endswith("/result"):
                self.handle_result(payload)
            elif topic.endswith("/orderList"):
                self.handle_orderList(payload)
        except json.JSONDecodeError:
            self.get_logger().error("Failed to decode JSON payload.")
        except Exception as e:
            self.get_logger().error(f"Error processing MQTT message: {e}")

    # ---------- ROS 콜백 ----------
    def check_cam_publisher(self):
        if self.count_publishers(self.camera_topic) > 0:
            self.ready_to_stream = True
            try:
                self.pub_check_timer.cancel()
            except Exception:
                pass
            self.get_logger().info(
                f"Camera publisher detected on {self.camera_topic}. "
                f"Streaming will start after {self.warmup_frames} warmup frames."
            )

    def gps_callback(self, msg: Point):
        """PoseConverterMiniStatic가 퍼블리시한 위경도(Point: x=lat, y=lon)를 MQTT로 보냄."""
        if not self._connected:
            return

        now = time.time()
        if (now - self.last_loc_pub_ts) < 1.0:  # 1Hz 스로틀
            return
        self.last_loc_pub_ts = now

        lat = float(msg.x)
        lon = float(msg.y)

        payload = {
            "robotId": int(ROBOT_ID),
            "latitude": lat,
            "longitude": lon,
        }
        try:
            self.mqtt_client.publish(
                TOPIC_ROBOT_TO_SERVER + "/updateLocation",
                json.dumps(payload),
                qos=0,
                retain=False
            )
        except Exception as e:
            self.get_logger().error(f"MQTT publish error (updateLocation): {e}")

    def update_delivery_state_callback(self, msg: String):
        if not self._connected:
            return
        payload = {
            "orderId": "1",  # 필요 시 실제 orderId 반영
            "robotId": int(ROBOT_ID),
            "state": msg.data
        }
        try:
            self.mqtt_client.publish(
                TOPIC_ROBOT_TO_SERVER + "/updateDeliveryState",
                json.dumps(payload),
                qos=0,
                retain=False
            )
        except Exception as e:
            self.get_logger().error(f"MQTT publish error (updateDeliveryState): {e}")

    def update_robot_status_callback(self, msg: String):
        if not self._connected:
            return
        payload = {
            "robotId": int(ROBOT_ID),
            "status": msg.data
        }
        try:
            self.mqtt_client.publish(
                TOPIC_ROBOT_TO_SERVER + "/updateRobotStatus",
                json.dumps(payload),
                qos=0,
                retain=False
            )
        except Exception as e:
            self.get_logger().error(f"MQTT publish error (updateRobotStatus): {e}")

    def image_callback(self, msg: Image):
        if not self._connected or not self.ready_to_stream:
            return

        self.recv_count += 1
        if self.recv_count <= self.warmup_frames:
            return

        now = time.time()
        if (now - self.last_pub_ts) < self.min_period:
            return

        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.desired_encoding)

            if self.max_width and self.max_width > 0 and img.shape[1] > self.max_width:
                scale = self.max_width / float(img.shape[1])
                new_h = int(img.shape[0] * scale)
                img = cv2.resize(img, (self.max_width, new_h), interpolation=cv2.INTER_AREA)

            ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)])
            if not ok:
                self.get_logger().warning("cv2.imencode failed; skip frame")
                return

            b64_data = base64.b64encode(buf.tobytes()).decode('ascii')
            payload = {
                "robotId": int(ROBOT_ID),
                "image": b64_data
            }

            # 단순화: 항상 서버 토픽으로 전송
            self.mqtt_client.publish(
                TOPIC_ROBOT_TO_SERVER + "/sendStreamingImage",
                json.dumps(payload),
                qos=0,
                retain=False
            )

            self.last_pub_ts = now

        except Exception as e:
            self.get_logger().error(f"Image streaming error: {e}")

    def handle_food_bay(self, payload):
        bay = payload.get("sectionNum")
        status = payload.get("sectionStatus")

        m = String()
        m.data = f"{bay} {cmd}"
        self.food_bay_pub.publish(m)

    def handle_result(self, payload):
        result = payload.get("result")
        if result is not None:
            msg = String()
            msg.data = str(result)
            self.result_pub.publish(msg)

    def handle_orderList(self, payload):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.orders_pub.publish(msg)

    # ---------- 종료 ----------
    def destroy_node(self):
        self._stop_event.set()
        if self.connection_thread.is_alive():
            self.connection_thread.join()
        try:
            if self._connected:
                self.mqtt_client.disconnect()
        except Exception:
            pass
        finally:
            if self._loop_started:
                try:
                    self.mqtt_client.loop_stop()
                except Exception:
                    pass
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
