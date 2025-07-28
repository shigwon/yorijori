package com.linky.api.order.service;

import com.linky.order.grpc.*;

public interface OrderService {
    boolean createOrder(OrderCreateRequest request);
    boolean updateDeliveryState(int orderId, String state);
    boolean updateLocation(int orderId, double customerLatitude, double customerLongitude);
    int searchOrderId(String code);
}
