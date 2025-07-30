package com.linky.api.robot.service.impl;

import com.linky.api.file.service.FileService;
import com.linky.api.mqtt.service.MqttPublishService;
import com.linky.api.order.entity.OrderSummary;
import com.linky.api.order.repository.OrderRepository;
import com.linky.api.robot.service.DeliveryService;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.redisson.api.RBlockingQueue;
import org.redisson.api.RDelayedQueue;
import org.redisson.api.RedissonClient;
import org.springframework.boot.context.event.ApplicationReadyEvent;
import org.springframework.context.event.EventListener;
import org.springframework.integration.annotation.ServiceActivator;
import org.springframework.integration.mqtt.support.MqttHeaders;
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


    @PostConstruct
    public void init() {
        this.queue = redissonClient.getBlockingQueue("robot-delivery-queue");
        this.delayedQueue = redissonClient.getDelayedQueue(queue);
    }

    public void resetTimer(int robotId) {
        String msg = "robot:" + robotId;
        delayedQueue.remove(msg);
        delayedQueue.offer(msg, 1, TimeUnit.MINUTES);
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

        String topic = (String) message.getHeaders().get(MqttHeaders.RECEIVED_TOPIC);

        if (topic == null) {
            return;
        }

        String payload = message.getPayload();

        log.info("payload :  {}", payload);

        String[] tokens = Objects.requireNonNull(topic).split("/");

        if (tokens.length < 4)
            return;

        String robotId = tokens[2];
        String command = tokens[3];
        List <String> extra = Arrays.asList(tokens).subList(4, tokens.length);

        log.info("📥 Received from robot {}: Command = {}, Extra = {}, Payload = {}", robotId, command, extra, payload);

        //Todo: 커맨드 별로 처리
    }

    public void sendOrderList(int robotId) {
        List<OrderSummary> orderList = orderRepository.searchOrderList(robotId);

        for (OrderSummary order : orderList) {
            order.setFaceImage(fileService.downloadFileToS3ByUrl(order.getFaceImageUrl()));
            log.info(order.toString());
        }

        mqttPublishService.sendOrderList(robotId, orderList);
    }


}
