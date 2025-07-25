package com.linky.api.order.grpc;

import com.linky.api.order.entity.Order;
import com.linky.api.order.mapper.OrderMapper;
import com.linky.api.order.service.OrderService;
import com.linky.order.grpc.OrderCreateRequest;
import com.linky.order.grpc.OrderCreateResponse;
import com.linky.order.grpc.OrderServiceGrpc;
import io.grpc.stub.StreamObserver;
import lombok.RequiredArgsConstructor;
import net.devh.boot.grpc.server.service.GrpcService;

@RequiredArgsConstructor
@GrpcService
public class OrderGrpcService extends OrderServiceGrpc.OrderServiceImplBase {

    private final OrderService orderService;

    @Override
    public void createOrder(OrderCreateRequest request, StreamObserver<OrderCreateResponse> responseObserver) {
        OrderCreateResponse orderCreateResponse = orderService.createOrder(request);

        responseObserver.onNext(orderCreateResponse);
        responseObserver.onCompleted();
    }
}