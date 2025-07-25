package com.linky.api.message.dto;

public record MessageRequestDto(
        String to,
        String text
) {
}
