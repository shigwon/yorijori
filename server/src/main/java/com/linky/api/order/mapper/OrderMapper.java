package com.linky.api.order.mapper;

import com.linky.api.order.entity.Order;
import com.linky.order.grpc.OrderCreateRequest;
import org.springframework.stereotype.Component;

@Component
public class OrderMapper {

    public Order toEntity(OrderCreateRequest request){
        return Order.builder()
                .code(request.getCode())
                .tel(request.getTel())
                .robotId(request.getRobotId())
                .build();
    }
}
