package com.linky.api.inquiry.dto;

import java.time.LocalDateTime;

public record ChatRoomSummaryDto(
        String orderCode,
        String lastMessage,
        LocalDateTime lastMessageTime,
        int messageCount,
        String customerTel,
        String status
) {
}