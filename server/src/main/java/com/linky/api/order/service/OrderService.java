package com.linky.api.order.service;

import com.linky.order.grpc.OrderCreateRequest;
import com.linky.order.grpc.OrderCreateResponse;

public interface OrderService {

    OrderCreateResponse createOrder(OrderCreateRequest request);
}
