package com.linky.api.order.repository;

import com.linky.api.order.entity.Order;
import org.apache.ibatis.annotations.Mapper;
import org.springframework.data.repository.query.Param;

@Mapper
public interface OrderRepository {
    int createOrder(Order order);
    int searchOrderId(String code);
    int updateDeliveryState(int orderId, String state);
    int updateLocationAndFaceImageUrl(int orderId, double customerLatitude, double customerLongitude, String faceImageUrl);
}
