package com.linky.api.order.grpc;

import com.linky.api.order.entity.Order;
import com.linky.api.order.mapper.OrderMapper;
import com.linky.api.order.service.OrderService;
import com.linky.order.grpc.*;
import io.grpc.Status;
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

    @Override
    public void updateDeliveryState(UpdateDeliveryStateRequest request, StreamObserver<UpdateDeliveryStateResponse> responseObserver) {

        UpdateDeliveryStateResponse response;

        try {
            int id = request.getId();
            OrderState state = request.getState();
            boolean updated = orderService.updateDeliveryState(id, state);

            if (updated) {
                response = UpdateDeliveryStateResponse.newBuilder()
                        .setSuccess(true)
                        .setMessage("배달 상태가 성공적으로 변경되었습니다.")
                        .build();
            } else {
                response = UpdateDeliveryStateResponse.newBuilder()
                        .setSuccess(false)
                        .setMessage("배달 상태 변경에 실패했습니다.")
                        .build();
            }
            responseObserver.onNext(response);
            responseObserver.onCompleted();

        } catch(Exception e) {
            responseObserver.onError(
                    Status.INTERNAL
                            .withDescription("서버 오류로 변경에 실패했습니다.")
                            .withCause(e)
                            .asRuntimeException()
            );
        }
    }
}