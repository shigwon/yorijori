package com.linky.api.review.grpc;

import com.google.protobuf.Empty;
import com.linky.api.review.mapper.ReviewMapper;
import com.linky.api.review.service.ReviewService;
import com.linky.review.grpc.*;
import io.grpc.stub.StreamObserver;
import lombok.RequiredArgsConstructor;
import net.devh.boot.grpc.server.service.GrpcService;

@GrpcService
@RequiredArgsConstructor
public class ReviewGrpcService extends ReviewServiceGrpc.ReviewServiceImplBase {

    private final ReviewService reviewService;
    private final ReviewMapper reviewMapper;

    @Override
    public void createReview(ReviewCreateRequest request, StreamObserver<ReviewCreateResponse> responseObserver) {
        int result = reviewService.createReview(reviewMapper.toDto(request));

        ReviewCreateResponse response;
        if(result > 0) {
            response = ReviewCreateResponse.newBuilder()
                    .setId(result)
                    .build();
        } else {
            response = ReviewCreateResponse.newBuilder().build();
        }

        responseObserver.onNext(response);
        responseObserver.onCompleted();
    }

    @Override
    public void getReview(ReviewRequest request, StreamObserver<ReviewResponse> responseObserver) {
        ReviewResponse response = reviewMapper.toProto(reviewService.getReview(request.getId()));

        responseObserver.onNext(response);
        responseObserver.onCompleted();
    }

    @Override
    public void getPaginatedReviews(PaginatedReviewsRequest request, StreamObserver<PaginatedReviewsResponse> responseObserver) {
        PaginatedReviewsResponse response = reviewMapper.toProto(reviewService.getPaginatedReviews(reviewMapper.toDto(request)));

        responseObserver.onNext(response);
        responseObserver.onCompleted();
    }

    @Override
    public void getReviewStatistics(Empty request, StreamObserver<ReviewStatisticsResponse> responseObserver) {
        ReviewStatisticsResponse response = reviewMapper.toProto(reviewService.getReviewStatistics());

        responseObserver.onNext(response);
        responseObserver.onCompleted();
    }
}

