package com.linky.api.order.dto.request;

import lombok.Data;

@Data
public class UpdateDeliveryStateDto {
    int orderId;
    int robotId;
    String state;
}
