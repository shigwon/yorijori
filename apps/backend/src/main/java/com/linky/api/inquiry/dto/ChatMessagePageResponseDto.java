package com.linky.api.inquiry.dto;

import java.util.List;

public record ChatMessagePageResponseDto(
        List<InquiryMessage> messages,
        boolean hasNext,
        int totalCount
) {}