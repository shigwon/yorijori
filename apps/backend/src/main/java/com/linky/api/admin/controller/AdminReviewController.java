package com.linky.api.admin.controller;

import com.linky.api.common.response.entity.ApiResponseEntity;
import com.linky.api.review.dto.request.PaginatedReviewsRequestDto;
import com.linky.api.review.dto.response.PaginatedReviewsResponseDto;
import com.linky.api.review.dto.response.ReviewStatisticsResponseDto;
import com.linky.api.review.service.ReviewService;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

@Slf4j
@RestController
@RequestMapping("/api/admin/reviews")
@RequiredArgsConstructor
public class AdminReviewController {

    private final ReviewService reviewService;

    @GetMapping("/rating")
    public ResponseEntity<ApiResponseEntity<ReviewStatisticsResponseDto>> getReviewStatistics() {
        log.info("[Admin] 리뷰 통계 조회 요청");

        ReviewStatisticsResponseDto statistics = reviewService.getReviewStatistics();

        log.info("[Admin] 리뷰 통계 조회 완료 - 평균 평점: {}", statistics.averageRating());
        return ApiResponseEntity.successResponseEntity(statistics);
    }

    @GetMapping
    public ResponseEntity<ApiResponseEntity<PaginatedReviewsResponseDto>> getAllReviews(
            @RequestParam(defaultValue = "1") int page,
            @RequestParam(defaultValue = "20") int pageSize,
            @RequestParam(defaultValue = "0") int minRating,
            @RequestParam(defaultValue = "DESC") com.linky.api.review.enums.SortOption sortOption) {

        log.info("[Admin] 리뷰 목록 조회 요청 - page: {}, pageSize: {}, minRating: {}, sortOption: {}",
                page, pageSize, minRating, sortOption);

        PaginatedReviewsRequestDto request = new PaginatedReviewsRequestDto(
                page,
                pageSize,
                minRating,
                sortOption
        );

        PaginatedReviewsResponseDto reviews = reviewService.getPaginatedReviews(request);

        log.info("[Admin] 리뷰 목록 조회 완료 - 총 {}건, 현재 페이지: {}/{}",
                reviews.totalCount(), reviews.currentPage(), reviews.totalPages());

        return ApiResponseEntity.successResponseEntity(reviews);
    }

    @GetMapping("/{reviewId}")
    public ResponseEntity<ApiResponseEntity<com.linky.api.review.dto.response.ReviewResponseDto>> getReview(
            @PathVariable int reviewId) {

        log.info("[Admin] 리뷰 상세 조회 요청 - reviewId: {}", reviewId);

        com.linky.api.review.dto.response.ReviewResponseDto review = reviewService.getReview(reviewId);

        log.info("[Admin] 리뷰 상세 조회 완료 - reviewId: {}, rating: {}", reviewId, review.rating());
        return ApiResponseEntity.successResponseEntity(review);
    }
}