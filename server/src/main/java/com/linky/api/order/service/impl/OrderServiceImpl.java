package com.linky.api.order.service.impl;

import com.linky.api.order.entity.Order;
import com.linky.api.order.mapper.OrderMapper;
import com.linky.api.order.repository.OrderRepository;
import com.linky.api.order.service.OrderService;
import com.linky.order.grpc.*;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;

@Service
@RequiredArgsConstructor
public class OrderServiceImpl implements OrderService {

    private final OrderMapper orderMapper;
    private final OrderRepository orderRepository;

    @Override
    public OrderCreateResponse createOrder(OrderCreateRequest request) {
        Order order = orderMapper.toEntity(request);
        return null;
    }

    @Override
    public boolean updateDeliveryState(int id, String state) {
        int result = orderRepository.updateDeliveryState(id, state);
        return result > 0;
    }

}
