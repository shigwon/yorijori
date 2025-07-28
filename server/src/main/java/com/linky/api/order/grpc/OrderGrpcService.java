package com.linky.api.order.grpc;

import com.linky.api.message.service.MessageService;
import com.linky.api.order.entity.Order;
import com.linky.api.order.mapper.OrderMapper;
import com.linky.api.order.service.OrderService;
import com.linky.order.grpc.*;
import io.grpc.Status;
import io.grpc.stub.StreamObserver;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import net.devh.boot.grpc.server.service.GrpcService;

@Slf4j
@RequiredArgsConstructor
@GrpcService
public class OrderGrpcService extends OrderServiceGrpc.OrderServiceImplBase {

    private final OrderService orderService;
    private final MessageService messageService;

    @Override
    public void createOrder(OrderCreateRequest request, StreamObserver<OrderCreateResponse> responseObserver) {

        OrderCreateResponse response;

        try {
            boolean created = orderService.createOrder(request);

            if (created) {
                int orderId = orderService.searchOrderId(request.getCode());
                messageService.messageSend(request.getTel(), "https://naver.com?order_id=" + orderId +"&robot_id=" + request.getRobotId() + "&code=" + request.getCode());

                response = OrderCreateResponse.newBuilder()
                        .setSuccess(true)
                        .setMessage("주문 생성에 성공하였습니다.")
                        .setOrderId(orderId)
                        .build();
            } else {
                response = OrderCreateResponse.newBuilder()
                        .setSuccess(false)
                        .setMessage("주문 생성에 실패하였습니다.")
                        .build();
            }
            responseObserver.onNext(response);
            responseObserver.onCompleted();

        } catch (Exception e) {
            responseObserver.onError(
                    Status.INTERNAL
                            .withDescription(e.getMessage())
                            .withCause(e)
                            .asRuntimeException()
            );
        }
    }

    @Override
    public void updateLocationAndFaceImageUrl(UpdateLocationAndFaceImageUrlRequest request, StreamObserver<UpdateLocationAndFaceImageUrlResponse> responseObserver) {

        UpdateLocationAndFaceImageUrlResponse response;

        try {
            int orderId = request.getOrderId();
            double customerLatitude = request.getCustomerLatitude();
            double customerLongitude = request.getCustomerLongitude();
            String faceImageUrl = request.getFaceImageUrl();
            boolean updated = orderService.updateLocationAndFaceImageUrl(orderId, customerLatitude, customerLongitude, faceImageUrl);

            if (updated) {
                response = UpdateLocationAndFaceImageUrlResponse.newBuilder()
                        .setSuccess(true)
                        .setMessage("고객 위치가 성공적으로 업데이트 되었습니다.")
                        .build();
            } else {
                response = UpdateLocationAndFaceImageUrlResponse.newBuilder()
                        .setSuccess(false)
                        .setMessage("고객 위치 변경에 실패했습니다.")
                        .build();
            }
            responseObserver.onNext(response);
            responseObserver.onCompleted();

        } catch(Exception e) {
            responseObserver.onError(
                    Status.INTERNAL
                            .withDescription(e.getMessage())
                            .withCause(e).
                            asRuntimeException()
            );
        }
    }

    @Override
    public void updateDeliveryState(UpdateDeliveryStateRequest request, StreamObserver<UpdateDeliveryStateResponse> responseObserver) {

        UpdateDeliveryStateResponse response;

        try {
            int orderId = request.getOrderId();
            String state = request.getState().name();
            boolean updated = orderService.updateDeliveryState(orderId, state);

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
                            .withDescription(e.getMessage())
                            .withCause(e)
                            .asRuntimeException()
            );
        }
    }
}