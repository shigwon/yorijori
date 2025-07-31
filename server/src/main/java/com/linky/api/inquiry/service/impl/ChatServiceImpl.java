package com.linky.api.inquiry.service.impl;

import com.linky.api.inquiry.dto.InquiryMessage;
import com.linky.api.inquiry.service.ChatService;
import lombok.RequiredArgsConstructor;
import org.springframework.data.redis.core.RedisTemplate;
import org.springframework.stereotype.Service;

import java.util.List;
import java.util.Set;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;

@Service
@RequiredArgsConstructor
public class ChatServiceImpl implements ChatService {

    private static final String CHAT_ROOMS_SET = "chat:rooms";
    private final RedisTemplate<String, Object> redisTemplate;

    @Override
    public void saveMessage(String orderCode, InquiryMessage message) {
        String key = getMessageKey(orderCode);

        redisTemplate.opsForSet().add(CHAT_ROOMS_SET, orderCode);

        double timestamp = message.timestamp().toEpochSecond(java.time.ZoneOffset.UTC);

        redisTemplate.opsForZSet().add(key, message, timestamp);
        redisTemplate.expire(key, 7, TimeUnit.DAYS);
    }

    @Override
    public List<InquiryMessage> getMessagesByOrderCode(String orderCode) {
        Set<Object> rawMessages = redisTemplate.opsForZSet().range(getMessageKey(orderCode), 0, -1);

        return rawMessages.stream()
                .map(msg -> (InquiryMessage) msg)
                .toList();
    }

    @Override
    public Set<String> getActiveChatRooms() {
        Set<Object> rawRooms = redisTemplate.opsForSet().members(CHAT_ROOMS_SET);

        return rawRooms.stream()
                .map(String::valueOf)
                .collect(Collectors.toSet());
    }

    private String getMessageKey(String orderCode) {
        return "chat:messages:" +  orderCode;
    }
}
