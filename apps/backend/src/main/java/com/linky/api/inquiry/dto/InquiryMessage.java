package com.linky.api.inquiry.dto;

import java.time.LocalDateTime;

public record InquiryMessage(
        String sender,
        String content,
        LocalDateTime timestamp
) {
}
