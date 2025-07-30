package com.linky.api.mqtt.service;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.linky.api.order.entity.OrderSummary;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.integration.mqtt.support.MqttHeaders;
import org.springframework.messaging.Message;
import org.springframework.messaging.MessageChannel;
import org.springframework.messaging.support.MessageBuilder;
import org.springframework.stereotype.Service;

import java.util.List;

@Slf4j
@Service
@RequiredArgsConstructor
public class MqttPublishService {

    private final MessageChannel mqttOutboundChannel;

    public void sendOrderList(int robotId, List<OrderSummary> orderList) {
        String topic = "linky/robot/" + robotId + "/orderList";

        try {
            String payload = new ObjectMapper().writeValueAsString(orderList);

            Message<String> message = MessageBuilder.withPayload(payload)
                    .setHeader(MqttHeaders.TOPIC, topic)
                    .build();

            mqttOutboundChannel.send(message);

        } catch (JsonProcessingException e) {
            log.error(e.getMessage());
        }
    }

}
