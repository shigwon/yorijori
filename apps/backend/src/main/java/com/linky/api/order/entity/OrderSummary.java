package com.linky.api.order.entity;

import lombok.*;

@Builder
@Data
@ToString
@AllArgsConstructor
@NoArgsConstructor
public class OrderSummary {
    int orderId;
    String code;
    String tel;
    double customerLatitude;
    double customerLongitude;
    String faceImageUrl;
    byte[] faceImage;
    int spaceNum;
}
