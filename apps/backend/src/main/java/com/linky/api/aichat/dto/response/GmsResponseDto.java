package com.linky.api.aichat.dto.response;

import java.util.List;

public record GmsResponseDto(
        String id,
        List<Object> choices
) {
}
