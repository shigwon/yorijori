package com.linky.api.order.dto.response;

public record OrderListResponseDto(
        int orderId, String code, String tel, double customerLatitude, double customerLongitude, byte[] faceImage, int spaceNum
) {
}
