package com.linky.api.inquiry.service.impl;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.linky.api.inquiry.dto.InquiryMessage;
import com.linky.api.inquiry.service.ChatService;
import lombok.RequiredArgsConstructor;
import org.springframework.data.redis.core.StringRedisTemplate;
import org.springframework.stereotype.Service;

import java.time.Duration;
import java.time.ZoneOffset;
import java.util.List;
import java.util.Objects;
import java.util.Set;

@Service
@RequiredArgsConstructor
public class ChatServiceImpl implements ChatService {

    private static final String CHAT_ROOMS_SET = "chat:rooms";
    private final StringRedisTemplate stringRedisTemplate;
    private final ObjectMapper objectMapper;

    @Override
    public void saveMessage(String orderCode, InquiryMessage message) {
        try {
            String key = "chat:messages:" + orderCode;
            String messageJson = objectMapper.writeValueAsString(message);
            double timestamp = message.timestamp().toEpochSecond(ZoneOffset.UTC);

            stringRedisTemplate.opsForSet().add(CHAT_ROOMS_SET, orderCode);
            stringRedisTemplate.opsForZSet().add(key, messageJson, timestamp);
            stringRedisTemplate.expire(key, Duration.ofDays(7));
        } catch (Exception e) {
            throw new RuntimeException("Failed to save message", e);
        }
    }

    @Override
    public List<InquiryMessage> getMessagesByOrderCode(String orderCode) {
        try {
            String key = "chat:messages:" + orderCode;
            Set<String> messages = stringRedisTemplate.opsForZSet().range(key, 0, -1);

            if (messages == null) return List.of();

            return messages.stream()
                    .map(json -> {
                        try {
                            return objectMapper.readValue(json, InquiryMessage.class);
                        } catch (Exception e) {
                            return null;
                        }
                    })
                    .filter(Objects::nonNull)
                    .toList();
        } catch (Exception e) {
            return List.of();
        }
    }

    @Override
    public Set<String> getActiveChatRooms() {
        Set<String> rooms = stringRedisTemplate.opsForSet().members(CHAT_ROOMS_SET);
        return rooms != null ? rooms : Set.of();
    }
}
