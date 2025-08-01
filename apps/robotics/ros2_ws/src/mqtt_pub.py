import paho.mqtt.client as mqtt

def pub(topic_str, message_str):
    mqtt_client = mqtt.Client()
    mqtt_client.connect("192.168.100.83", 1883) # 1번
    mqtt_client.loop_start()

    topic_name = topic_str # 2번
    message = message_str

    mqtt_client.publish(topic = topic_name, payload = message) # 3번
    mqtt_client.loop_stop()

pub("linky/robot/1/test", "1")