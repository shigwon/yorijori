package com.linky.api.order.dto.request;

public record UpdateLocationRequestDto(
        int orderId,
        int robotId,
        double customerLatitude,
        double customerLongitude
) {
}
