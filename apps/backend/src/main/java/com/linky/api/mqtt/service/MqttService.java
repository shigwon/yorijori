package com.linky.api.mqtt.service;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.linky.api.order.dto.request.UpdateDeliveryStateDto;
import com.linky.api.order.entity.OrderSummary;
import com.linky.api.order.service.OrderService;
import com.linky.api.robot.dto.RobotLocationDto;
import com.linky.api.robot.dto.RobotRequestResultDto;
import com.linky.api.robot.service.RobotService;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.integration.annotation.ServiceActivator;
import org.springframework.integration.mqtt.support.MqttHeaders;
import org.springframework.messaging.Message;
import org.springframework.messaging.MessageChannel;
import org.springframework.messaging.support.MessageBuilder;
import org.springframework.stereotype.Service;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;

@Slf4j
@Service
@RequiredArgsConstructor
public class MqttService {

    private final MessageChannel mqttOutboundChannel;
    private final OrderService orderService;
    private final RobotService robotService;
    private final ObjectMapper objectMapper;

    public void sendPayload(int robotId, String topicSuffix, Object payloadObj) {
        String topic = "linky/robot/" + robotId + "/" + topicSuffix;

        try {
            String payload = new ObjectMapper().writeValueAsString(payloadObj);

            Message<String> message = MessageBuilder.withPayload(payload)
                    .setHeader(MqttHeaders.TOPIC, topic)
                    .build();

            mqttOutboundChannel.send(message);

        } catch (JsonProcessingException e) {
            log.error("MQTT Payload 직렬화 오류: {}", e.getMessage(), e);
        }
    }

    public void sendResult(int robotId, RobotRequestResultDto result) {
        sendPayload(robotId, "result", result);
    }

    public void sendOrderList(int robotId, List<OrderSummary> orderList) {
        sendPayload(robotId, "orderList", orderList);
    }

    @ServiceActivator(inputChannel = "mqttInboundChannel")
    public void listenRobotMessage(Message<String> message) {

        log.debug("testsetest  : {}", message.getPayload());

        String topic = (String) message.getHeaders().get("mqtt_receivedTopic");

        if (topic == null) {
            log.warn("토픽이 없는 메세지. 무시.");
            return;
        }

        String payload = message.getPayload();
        String[] tokens = Objects.requireNonNull(topic).split("/");

        if (tokens.length < 4) {
            log.warn("유효하지 않은 토픽 구조. 무시.");
            return;
        }

        String robotId = tokens[2];
        String command = tokens[3];
        List<String> extra = Arrays.asList(tokens).subList(4, tokens.length);

        // 로봇 번호가 0이 아닐 경우 무시
        if (!"0".equals(robotId) || command.equals("result")) {
            log.debug("처리 안 함 payload: {}", payload);
            return;
        }

        log.info("📥 Received Command = {}, Extra = {}, Payload = {}", command, extra, payload);

        try {
            switch (command) {
                case "updateDeliveryState":
                    UpdateDeliveryStateDto updateDeliveryStateDto = objectMapper.readValue(payload, UpdateDeliveryStateDto.class);
                    boolean result = orderService.updateDeliveryState(updateDeliveryStateDto.getOrderId(), updateDeliveryStateDto.getState());
                    sendResult(updateDeliveryStateDto.getRobotId(), new RobotRequestResultDto(result));
                    break;
                case "updateLocation":
                    RobotLocationDto robotLocationDto = objectMapper.readValue(payload, RobotLocationDto.class);
                    robotService.saveLocationToRedis(robotLocationDto);
                    break;
            }
        } catch (JsonProcessingException e) {
            log.warn("객체 매핑 에러, 커멘드 : {}, 에러 : {}", command, e.getMessage());
        }
    }
}
