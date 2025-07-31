package com.linky.api.order.dto.request;

public record CreateOrderRequestDto(
        int robotId,
        String code,
        String tel
) {
}
