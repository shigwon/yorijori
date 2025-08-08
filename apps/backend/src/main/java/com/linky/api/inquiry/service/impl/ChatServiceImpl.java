package com.linky.api.inquiry.service.impl;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.linky.api.inquiry.dto.ChatMessagePageRequestDto;
import com.linky.api.inquiry.dto.ChatMessagePageResponseDto;
import com.linky.api.inquiry.dto.ChatRoomSummaryDto;
import com.linky.api.inquiry.dto.InquiryMessage;
import com.linky.api.inquiry.service.ChatService;
import com.linky.api.order.repository.OrderRepository;
import com.linky.api.order.entity.Order;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.data.redis.core.StringRedisTemplate;
import org.springframework.stereotype.Service;

import java.time.Duration;
import java.time.ZoneOffset;
import java.util.List;
import java.util.Objects;
import java.util.Set;

@Service
@RequiredArgsConstructor
@Slf4j
public class ChatServiceImpl implements ChatService {

    private static final String CHAT_ROOMS_SET = "chat:rooms";
    private final StringRedisTemplate stringRedisTemplate;
    private final ObjectMapper objectMapper;
    private final OrderRepository orderRepository;

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
        // 내부적으로 페이징 메서드 사용 (전체 조회)
        return getMessagesByOrderCode(orderCode, null).messages();
    }

    @Override
    public ChatMessagePageResponseDto getMessagesByOrderCode(String orderCode, ChatMessagePageRequestDto request) {
        try {
            String key = "chat:messages:" + orderCode;

            // 전체 메시지 수
            Long totalCount = stringRedisTemplate.opsForZSet().zCard(key);
            if (totalCount == null || totalCount == 0) {
                return new ChatMessagePageResponseDto(List.of(), false, 0); // 수정된 부분
            }

            Set<String> messageJsons;
            boolean hasNext;

            if (request == null) {
                // 전체 조회 (기존 API 호환성)
                messageJsons = stringRedisTemplate.opsForZSet().reverseRange(key, 0, -1);
                hasNext = false;
            } else {
                // 페이징 조회
                int offset = (request.page() - 1) * request.size();
                messageJsons = stringRedisTemplate.opsForZSet()
                        .reverseRange(key, offset, offset + request.size() - 1);

                // hasNext 계산: 다음 페이지가 있는지 확인
                int totalPages = (int) Math.ceil((double) totalCount / request.size());
                hasNext = request.page() < totalPages;
            }

            List<InquiryMessage> messages = parseMessages(messageJsons);

            return new ChatMessagePageResponseDto(messages, hasNext, totalCount.intValue()); // 수정된 부분

        } catch (Exception e) {
            log.error("메시지 조회 실패: orderCode={}", orderCode, e);
            return new ChatMessagePageResponseDto(List.of(), false, 0); // 수정된 부분
        }
    }

    @Override
    public Set<String> getActiveChatRooms() {
        Set<String> rooms = stringRedisTemplate.opsForSet().members(CHAT_ROOMS_SET);
        return rooms != null ? rooms : Set.of();
    }

    @Override
    public List<ChatRoomSummaryDto> getActiveChatRoomsWithSummary() {
        try {
            Set<String> orderCodes = getActiveChatRooms();
            if (orderCodes.isEmpty()) {
                return List.of();
            }

            return orderCodes.stream()
                    .map(this::getChatRoomSummary)
                    .filter(Objects::nonNull)
                    .sorted((a, b) -> b.lastMessageTime().compareTo(a.lastMessageTime()))
                    .toList();

        } catch (Exception e) {
            log.error("채팅방 목록 조회 실패", e);
            return List.of();

        }

    }

    private List<InquiryMessage> parseMessages(Set<String> messageJsons) {
        if (messageJsons == null || messageJsons.isEmpty()) {
            return List.of();
        }

        return messageJsons.stream()
                .map(json -> {
                    try {
                        return objectMapper.readValue(json, InquiryMessage.class);
                    } catch (Exception e) {
                        log.warn("메시지 파싱 실패: {}", json, e);
                        return null;
                    }
                })
                .filter(Objects::nonNull)
                .toList();
    }

    private ChatRoomSummaryDto getChatRoomSummary(String orderCode) {
        try {
            String key = "chat:messages:" + orderCode;

            // 메시지 총 개수
            Long messageCount = stringRedisTemplate.opsForZSet().zCard(key);

            // 마지막 메시지 조회 (ZSet의 마지막 요소)
            Set<String> lastMessages = stringRedisTemplate.opsForZSet()
                    .reverseRange(key, 0, 0);

            if (lastMessages == null || lastMessages.isEmpty()) {
                return null;
            }

            String lastMessageJson = lastMessages.iterator().next();
            InquiryMessage lastMessage = objectMapper.readValue(lastMessageJson, InquiryMessage.class);

            // 고객 전화번호 조회
            String customerTel = getCustomerTelByOrderCode(orderCode);

            return new ChatRoomSummaryDto(
                    orderCode,
                    lastMessage.content(),
                    lastMessage.timestamp(), // 이게 lastMessageTime입니다
                    messageCount != null ? messageCount.intValue() : 0,
                    customerTel,
                    "ACTIVE"
            );

        } catch (Exception e) {
            log.error("채팅방 요약 정보 조회 실패: orderCode={}", orderCode, e);
            return null;
        }
    }

    private String getCustomerTelByOrderCode(String orderCode) {
        try {
            Order order = orderRepository.findByCode(orderCode);
            return order != null ? order.getTel() : "알 수 없음";
        } catch (Exception e) {
            log.warn("고객 전화번호 조회 실패: orderCode={}", orderCode, e);
            return "알 수 없음";
        }
    }
}