package com.linky.api.order.repository;

import com.linky.api.order.entity.Order;
import com.linky.api.order.entity.OrderSummary;
import org.apache.ibatis.annotations.Mapper;

import java.util.List;

@Mapper
public interface OrderRepository {
    int createOrder(Order order);
    int searchOrderId(String code);
    int updateDeliveryState(int orderId, String state);
    int updateLocationAndFaceImageUrl(int orderId, double customerLatitude, double customerLongitude);
    int searchCapacity(int robotId);

    List<OrderSummary> searchOrderList(int robotId);
}
