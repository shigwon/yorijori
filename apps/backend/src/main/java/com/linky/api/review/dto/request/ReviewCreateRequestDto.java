package com.linky.api.review.dto.request;

public record ReviewCreateRequestDto(
        String orderCode,
        int rating,
        String content
) {
}
