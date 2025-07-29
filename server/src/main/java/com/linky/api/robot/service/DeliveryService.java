package com.linky.api.robot.service;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.redisson.api.RBlockingQueue;
import org.redisson.api.RDelayedQueue;
import org.redisson.api.RedissonClient;
import org.springframework.boot.context.event.ApplicationReadyEvent;
import org.springframework.context.event.EventListener;
import org.springframework.scheduling.annotation.Async;
import org.springframework.stereotype.Service;

import javax.annotation.PostConstruct;
import java.util.concurrent.TimeUnit;

@Slf4j
@Service
@RequiredArgsConstructor
public class DeliveryService {

    private final RedissonClient redissonClient;
    private RBlockingQueue<String> queue;
    private RDelayedQueue<String> delayedQueue;

    @PostConstruct
    public void init() {
        this.queue = redissonClient.getBlockingQueue("robot-delivery-queue");
        this.delayedQueue = redissonClient.getDelayedQueue(queue);
    }

    public void resetTimer(String robotId) {
        String msg = "robot:" + robotId;
        delayedQueue.remove(msg);
        delayedQueue.offer(msg, 3, TimeUnit.MINUTES);
    }

    @Async
    @EventListener(ApplicationReadyEvent.class)
    public void startTimerListener(){
        while (true) {
            try {
                String msg = queue.take();
                String robotId = msg.replace("robot:", "");
                log.info("로봇 " + robotId + " 의 3분 타이머 만료 신호 받음!");
            } catch (InterruptedException e) {
                log.warn("타이머 리스너 인터럽트.");
                Thread.currentThread().interrupt();
                break;
            }
        }
    }



}
