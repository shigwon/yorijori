import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import asyncio
import json
import cv2
import numpy as np

from aiortc import RTCPeerConnection, VideoStreamTrack, RTCSessionDescription
from aiortc.contrib.media import MediaBlackhole

import paho.mqtt.client as mqtt

# WebRTC용 비디오 스트림 클래스로 ROS 이미지 스트림을 프레임으로 변환
class ROSVideoStreamTrack(VideoStreamTrack):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.bridge = CvBridge()
        self.frame = None
        self.new_frame_event = asyncio.Event()

    def update_frame(self, ros_img_msg):
        # ROS 이미지 메시지를 OpenCV 이미지로 변환
        cv_img = self.bridge.imgmsg_to_cv2(ros_img_msg, desired_encoding='bgr8')
        # BGR -> RGB (WebRTC expects RGB)
        self.frame = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        # Set event so that next frame can be sent
        loop = asyncio.get_event_loop()
        loop.call_soon_threadsafe(self.new_frame_event.set)

    async def recv(self):
        # 기다렸다가 새 프레임 보내기
        await self.new_frame_event.wait()
        self.new_frame_event.clear()

        if self.frame is None:
            return None

        # aiortc용 VideoFrame 생성
        from av import VideoFrame
        video_frame = VideoFrame.from_ndarray(self.frame, format="rgb24")
        video_frame.pts = None
        video_frame.time_base = None
        return video_frame


class WebRTCNode(Node):
    def __init__(self):
        super().__init__('webrtc_streamer_node')

        self.pc = RTCPeerConnection()
        self.video_track = ROSVideoStreamTrack(self)
        self.pc.addTrack(self.video_track)

        self.bridge = CvBridge()

        # MQTT 설정
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message

        self.mqtt_client.connect("mqtt_broker_ip", 1883, 60)
        self.mqtt_client.loop_start()

        # signaling 토픽
        self.mqtt_signaling_topic = "webrtc/signaling"

        # ROS 이미지 구독
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        self.offer_sent = False

    def on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"Connected to MQTT broker with code {rc}")
        client.subscribe(self.mqtt_signaling_topic)

    def on_mqtt_message(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())
        self.get_logger().info(f"Received MQTT signaling message: {data}")

        asyncio.run_coroutine_threadsafe(self.handle_signaling_message(data), asyncio.get_event_loop())

    async def handle_signaling_message(self, data):
        # 상대방에서 offer가 왔을 때 answer 준비
        if data["type"] == "offer":
            offer = RTCSessionDescription(sdp=data["sdp"], type=data["type"])
            await self.pc.setRemoteDescription(offer)
            answer = await self.pc.createAnswer()
            await self.pc.setLocalDescription(answer)

            # MQTT로 answer 전송
            answer_msg = {
                "type": self.pc.localDescription.type,
                "sdp": self.pc.localDescription.sdp
            }
            self.mqtt_client.publish(self.mqtt_signaling_topic, json.dumps(answer_msg))
            self.get_logger().info("Sent answer via MQTT")

        # answer 처리
        elif data["type"] == "answer":
            answer = RTCSessionDescription(sdp=data["sdp"], type=data["type"])
            await self.pc.setRemoteDescription(answer)

        # ICE candidate 처리 (필요하면 추가)
        elif data["type"] == "candidate":
            # TODO: ICE candidate 추가 코드
            pass

    def image_callback(self, msg):
        self.video_track.update_frame(msg)

    async def create_and_send_offer(self):
        offer = await self.pc.createOffer()
        await self.pc.setLocalDescription(offer)

        offer_msg = {
            "type": self.pc.localDescription.type,
            "sdp": self.pc.localDescription.sdp
        }
        self.mqtt_client.publish(self.mqtt_signaling_topic, json.dumps(offer_msg))
        self.get_logger().info("Sent offer via MQTT")

def main(args=None):
    rclpy.init(args=args)
    node = WebRTCNode()

    # asyncio 이벤트 루프 가져오기 및 offer 생성 예약
    loop = asyncio.get_event_loop()
    loop.create_task(node.create_and_send_offer())

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        asyncio.run(node.pc.close())
        node.mqtt_client.loop_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
