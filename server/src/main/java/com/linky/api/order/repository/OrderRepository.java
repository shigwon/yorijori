package com.linky.api.order.repository;

import com.linky.api.order.entity.Order;
import org.apache.ibatis.annotations.Mapper;

@Mapper
public interface OrderRepository {
    int createOrder(Order order);
    int searchOrderId(String code);
    Order findById(int id);
    int updateDeliveryState(int orderId, String state);
    int updateLocationAndFaceImageUrl(int orderId, double customerLatitude, double customerLongitude);
    int searchCapacity(int robotId);
}
