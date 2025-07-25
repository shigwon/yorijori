package com.linky.api.test.grpc;

import com.linky.api.test.service.TestValidationService;
import io.grpc.stub.StreamObserver;
import lombok.RequiredArgsConstructor;
import net.devh.boot.grpc.server.service.GrpcService;

@RequiredArgsConstructor
@GrpcService
public class TestServiceImpl extends TestServiceGrpc.TestServiceImplBase {

    private final TestValidationService validationService;

    @Override
    public void testValidation(TestValidationRequest request, StreamObserver<TestValidationResponse> responseObserver) {
        String token = request.getAccessToken();

        TestValidationResponse response = validationService.validationToken(token);

        responseObserver.onNext(response);
        responseObserver.onCompleted();
    }
}
