package com.linky.api.order.service;

import com.linky.api.order.dto.request.CreateOrderRequestDto;
import com.linky.api.order.dto.request.UpdateLocationRequestDto;
import com.linky.order.grpc.OrderCreateRequest;

public interface OrderService {
    boolean createOrder(CreateOrderRequestDto createOrderRequest);
    boolean updateLocation(int orderId, UpdateLocationRequestDto updateLocationRequest);
    int searchOrderId(CreateOrderRequestDto createOrderRequest);
    int searchOrderUpdateCount(int robotId);
    boolean updateDeliveryState(int orderId, String state);
}
