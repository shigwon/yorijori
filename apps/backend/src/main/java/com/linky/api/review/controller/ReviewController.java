package com.linky.api.review.controller;

import com.linky.api.common.response.entity.ApiResponseEntity;
import com.linky.api.review.dto.request.PaginatedReviewsRequestDto;
import com.linky.api.review.dto.request.ReviewCreateRequestDto;
import com.linky.api.review.dto.response.PaginatedReviewsResponseDto;
import com.linky.api.review.dto.response.ReviewResponseDto;
import com.linky.api.review.dto.response.ReviewStatisticsResponseDto;
import com.linky.api.review.service.ReviewService;
import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

@RestController
@RequestMapping("/api/v1/reviews")
@RequiredArgsConstructor
public class ReviewController {

    private final ReviewService reviewService;

    @PostMapping
    public ResponseEntity<ApiResponseEntity<Void>> createReview(@RequestBody ReviewCreateRequestDto request){
        reviewService.createReview(request);

        return ApiResponseEntity.successResponseEntity();
    }

    @GetMapping("/{reviewId}")
    public ResponseEntity<ApiResponseEntity<ReviewResponseDto>> getReview(@PathVariable("reviewId") Integer reviewId){
        return ApiResponseEntity.successResponseEntity(reviewService.getReview(reviewId));
    }

    @GetMapping
    public ResponseEntity<ApiResponseEntity<PaginatedReviewsResponseDto>> getPaginatedReviews(@RequestBody PaginatedReviewsRequestDto request){
        return ApiResponseEntity.successResponseEntity(reviewService.getPaginatedReviews(request));
    }

    @GetMapping("/statistics")
    public ResponseEntity<ApiResponseEntity<ReviewStatisticsResponseDto>> getReviewStatistics(){
        return ApiResponseEntity.successResponseEntity(reviewService.getReviewStatistics());
    }
}
