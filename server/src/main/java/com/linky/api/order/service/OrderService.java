package com.linky.api.order.service;

import com.linky.order.grpc.*;

public interface OrderService {
    boolean createOrder(OrderCreateRequest request);
    boolean updateDeliveryState(int orderId, String state);
    boolean updateLocationAndFaceImageUrl(int orderId, double customerLatitude, double customerLongitude, String faceImageUrl);
    int searchOrderId(String code);
}
