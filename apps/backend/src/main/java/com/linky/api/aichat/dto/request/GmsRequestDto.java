package com.linky.api.aichat.dto.request;

import java.util.List;
import java.util.Map;

public record GmsRequestDto(
        String model,
        List<Map<String, String>> messages,
        int max_tokens,
        double temperature
) {
}
