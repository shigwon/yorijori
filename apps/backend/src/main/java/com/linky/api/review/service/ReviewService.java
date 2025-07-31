package com.linky.api.review.service;

import com.linky.api.review.dto.request.PaginatedReviewsRequestDto;
import com.linky.api.review.dto.request.ReviewCreateRequestDto;
import com.linky.api.review.dto.response.PaginatedReviewsResponseDto;
import com.linky.api.review.dto.response.ReviewResponseDto;
import com.linky.api.review.dto.response.ReviewStatisticsResponseDto;

public interface ReviewService {

    void createReview(ReviewCreateRequestDto request);

    ReviewResponseDto getReview(int reviewId);

    PaginatedReviewsResponseDto getPaginatedReviews(PaginatedReviewsRequestDto request);

    ReviewStatisticsResponseDto getReviewStatistics();
}
