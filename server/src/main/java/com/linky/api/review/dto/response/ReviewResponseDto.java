package com.linky.api.review.dto.response;

import java.time.LocalDateTime;

public record ReviewResponseDto(
        int id,
        String orderCode,
        int rating,
        String content,
        LocalDateTime createdAt
) {
}
