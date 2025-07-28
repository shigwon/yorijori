package com.linky.api.test.service;

import com.linky.api.test.grpc.TestStreamingRequest;
import com.linky.api.test.grpc.TestStreamingResponse;
import io.grpc.stub.StreamObserver;
import org.springframework.stereotype.Service;

@Service
public class TestStreamingService {

    public void startStreaming(TestStreamingRequest request, StreamObserver<TestStreamingResponse> responseObserver) {
        // 예: 신호 받아서 일정 시간 동안 스트리밍 응답 보내기
        new Thread(() -> {
            try {
                for (int i = 0; i < 1000; i++) {
                    TestStreamingResponse response = TestStreamingResponse.newBuilder()
                            .setMessage(request.getSignal() + " - streaming message " + (i + 1))
                            .build();
                    responseObserver.onNext(response);
                    Thread.sleep(300);
                }
                responseObserver.onCompleted();
            } catch (InterruptedException e) {
                responseObserver.onError(e);
            }
        }).start();
    }
}
