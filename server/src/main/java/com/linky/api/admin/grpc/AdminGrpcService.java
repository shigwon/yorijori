package com.linky.api.admin.grpc;

import com.linky.api.order.entity.Order;
import com.linky.api.order.mapper.OrderMapper;
import com.linky.api.order.service.OrderService;
import com.linky.order.grpc.*;
import io.grpc.Status;
import io.grpc.stub.StreamObserver;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import net.devh.boot.grpc.server.service.GrpcService;

@RequiredArgsConstructor
@GrpcService
public class AdminGrpcService extends AdminServiceGrpc.AdminServiceImplBase {


}