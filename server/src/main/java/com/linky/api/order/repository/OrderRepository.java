package com.linky.api.order.repository;

import com.linky.order.grpc.OrderState;
import org.apache.ibatis.annotations.Mapper;

@Mapper
public interface OrderRepository {
    int updateDeliveryState(int id, String state);
}
