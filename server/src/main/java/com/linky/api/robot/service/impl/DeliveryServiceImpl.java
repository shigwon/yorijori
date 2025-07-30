package com.linky.api.robot.service.impl;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.linky.api.file.service.FileService;
import com.linky.api.mqtt.service.MqttPublishService;
import com.linky.api.order.dto.UpdateDeliveryStateDto;
import com.linky.api.order.entity.OrderSummary;
import com.linky.api.order.repository.OrderRepository;
import com.linky.api.order.service.OrderService;
import com.linky.api.robot.service.DeliveryService;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.redisson.api.RBlockingQueue;
import org.redisson.api.RDelayedQueue;
import org.redisson.api.RedissonClient;
import org.springframework.boot.context.event.ApplicationReadyEvent;
import org.springframework.context.event.EventListener;
import org.springframework.integration.annotation.ServiceActivator;
import org.springframework.messaging.Message;
import org.springframework.scheduling.annotation.Async;
import org.springframework.stereotype.Service;

import javax.annotation.PostConstruct;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.TimeUnit;

@Slf4j
@Service
@RequiredArgsConstructor
public class DeliveryServiceImpl implements DeliveryService {

    private final OrderRepository orderRepository;
    private final MqttPublishService mqttPublishService;
    private final RedissonClient redissonClient;
    private RBlockingQueue<String> queue;
    private RDelayedQueue<String> delayedQueue;
    private final FileService fileService;
    private final OrderService orderService;
    private final ObjectMapper objectMapper;


    @PostConstruct
    public void init() {
        this.queue = redissonClient.getBlockingQueue("robot-delivery-queue");
        this.delayedQueue = redissonClient.getDelayedQueue(queue);
    }

    public void resetTimer(int robotId) {
        String msg = "robot:" + robotId;
        delayedQueue.remove(msg);
        delayedQueue.offer(msg, 3, TimeUnit.MINUTES);
    }

    public void interruptTimer(int robotId) {
        String msg = "robot:" + robotId;
        delayedQueue.remove(msg);
        log.info("robot {} 의 음식 칸 다 참 [ 인터럽트 ]", robotId);
        sendOrderList(robotId);
    }

    @Async
    @EventListener(ApplicationReadyEvent.class)
    public void startTimerListener(){
        while (true) {
            try {
                String msg = queue.take();
                String robotId = msg.replace("robot:", "");
                log.info("로봇 {} 의 3분 타이머 만료 신호 받음!", robotId);
                sendOrderList(Integer.parseInt(robotId));
            } catch (InterruptedException e) {
                log.warn("타이머 리스너 인터럽트.");
                Thread.currentThread().interrupt();
                break;
            }
        }
    }

    @ServiceActivator(inputChannel = "mqttInboundChannel")
    public void listenRobotMessage(Message<String> message) {

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
        if (!"0".equals(robotId)) {
            log.debug("로봇 번호가 0이 아님. 처리 안 함. robotId={}", robotId);
            return;
        }

        log.info("📥 Received from robot {}: Command = {}, Extra = {}, Payload = {}", robotId, command, extra, payload);

        try {
            switch (command) {
                case "updateDeliveryState":
                    UpdateDeliveryStateDto updateDeliveryStateDto = objectMapper.readValue(payload, UpdateDeliveryStateDto.class);
                    orderService.updateDeliveryState(updateDeliveryStateDto.getOrderId(), updateDeliveryStateDto.getState());
                    break;
            }
        } catch (JsonProcessingException e) {
            log.warn("객체 매핑 에러, 커멘드 : {}, 에러 : {}", command, e.getMessage());
        }
    }

    public void sendOrderList(int robotId) {
        List<OrderSummary> orderList = orderRepository.searchOrderList(robotId);

        for (OrderSummary order : orderList) {
            //order.setFaceImage(fileService.downloadFileToS3ByUrl(order.getFaceImageUrl()));
            log.info(order.toString());
        }

        mqttPublishService.sendOrderList(robotId, orderList);
    }


}
