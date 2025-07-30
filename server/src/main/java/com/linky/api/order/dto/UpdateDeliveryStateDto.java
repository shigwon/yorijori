package com.linky.api.order.dto;

import lombok.Data;

@Data
public class UpdateDeliveryStateDto {
    int orderId;
    String state;
}
