package com.linky.api.order.service.impl;

import com.linky.api.order.entity.Order;
import com.linky.api.order.mapper.OrderMapper;
import com.linky.api.order.repository.OrderRepository;
import com.linky.api.order.service.OrderService;
import com.linky.order.grpc.OrderCreateRequest;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Service;

@Slf4j
@Service
@RequiredArgsConstructor
public class OrderServiceImpl implements OrderService {

    private final OrderMapper orderMapper;
    private final OrderRepository orderRepository;

    @Override
    public boolean createOrder(OrderCreateRequest request) {
        Order order = orderMapper.toEntity(request);
        int spaceNum = orderRepository.searchCapacity(order.getRobotId()) + 1;

        if(spaceNum > 3)
            return false;

        order.setSpaceNum(spaceNum);
        int result = orderRepository.createOrder(order);
        return result > 0;
    }

    @Override
    public int searchOrderId(String code) {
        return orderRepository.searchOrderId(code);
    }

    @Override
    public boolean updateDeliveryState(int orderId, String state) {
        int result = orderRepository.updateDeliveryState(orderId, state);
        return result > 0;
    }

    @Override
    public boolean updateLocation(int orderId, double customerLatitude, double customerLongitude) {
        int result = orderRepository.updateLocationAndFaceImageUrl(orderId, customerLatitude, customerLongitude);
        return result > 0;
    }

    @Override
    public int searchOrderUpdateCount(int robotId) {
        return orderRepository.searchOrderUpdateCount(robotId);
    }


}
