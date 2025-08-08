package com.linky.api.inquiry.dto;

public record ChatMessagePageRequestDto(
        int page,
        int size
) {}