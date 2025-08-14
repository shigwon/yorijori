package com.linky.api.order.mapper;

import com.linky.api.order.dto.request.CreateOrderRequestDto;
import com.linky.api.order.dto.request.UpdateLocationRequestDto;
import com.linky.api.order.entity.Order;
import org.springframework.stereotype.Component;

@Component
public class OrderMapper {

    public Order toEntity(CreateOrderRequestDto request){
        return Order.builder()
                .robotId(request.robotId())
                .code(request.code())
                .tel(request.tel())
                .build();
    }

    public Order toEntity(UpdateLocationRequestDto request) {
        return Order.builder()
                .orderId(request.orderId())
                .robotId(request.robotId())
                .customerLatitude(request.customerLatitude())
                .customerLongitude(request.customerLongitude())
                .build();
    }

}
