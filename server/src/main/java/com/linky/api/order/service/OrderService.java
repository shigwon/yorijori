package com.linky.api.order.service;

import com.linky.order.grpc.*;

public interface OrderService {

    OrderCreateResponse createOrder(OrderCreateRequest request);
    boolean updateDeliveryState(int id, String state);
}
