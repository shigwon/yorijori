package com.linky.api.order.entity;

import com.linky.api.order.enums.OrderState;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.time.LocalDateTime;

@Builder
@Data
@AllArgsConstructor
@NoArgsConstructor
public class Order {

    int id;
    int robotId;
    String code;
    String tel;
    double customerLatitude;
    double customerLongitude;
    LocalDateTime startTime;
    LocalDateTime endTime;
    OrderState state;
    String faceImageUrl;
    String foodImageUrl;
    int spaceNum;
    LocalDateTime createdAt;
}
