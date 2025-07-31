package com.linky.api.review.service.impl;

import com.linky.api.order.entity.Order;
import com.linky.api.order.repository.OrderRepository;
import com.linky.api.review.dto.request.PaginatedReviewsRequestDto;
import com.linky.api.review.dto.request.ReviewCreateRequestDto;
import com.linky.api.review.dto.response.PaginatedReviewsResponseDto;
import com.linky.api.review.dto.response.ReviewResponseDto;
import com.linky.api.review.dto.response.ReviewStatisticsResponseDto;
import com.linky.api.review.entity.Review;
import com.linky.api.review.entity.ReviewStatsRawData;
import com.linky.api.review.mapper.ReviewMapper;
import com.linky.api.review.repository.ReviewRepository;
import com.linky.api.review.service.ReviewService;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

@Service
@RequiredArgsConstructor
@Slf4j
public class ReviewServiceImpl implements ReviewService {

    private final ReviewRepository reviewRepository;
    private final OrderRepository orderRepository;
    private final ReviewMapper reviewMapper;

    @Transactional
    @Override
    public void createReview(ReviewCreateRequestDto request) {
        int orderId = orderRepository.searchOrderId(request.orderCode());

        if(orderId <= 0) {
            log.error("[ReviewServiceImpl] 주문 번호가 존재하지 않습니다.");
        }

        Review newReview = reviewMapper.toEntity(request);
        newReview.setOrderId(orderId);
    }

    @Override
    public ReviewResponseDto getReview(int reviewId) {
        if(reviewId <= 0) {
            log.error("[ReviewServiceImpl] 리뷰 번호가 존재하지 않습니다.");
        }

        Review review = reviewRepository.findById(reviewId);
        Order order = orderRepository.findById(review.getOrderId());

        return new ReviewResponseDto(
                review.getId(),
                order.getCode(),
                review.getRating(),
                review.getContent(),
                review.getCreatedAt());
    }

    @Transactional
    @Override
    public PaginatedReviewsResponseDto getPaginatedReviews(PaginatedReviewsRequestDto request) {
        long totalCount = reviewRepository.countByCriteria(request.minRating());
        int page = Math.max(1, request.page());
        int pageSize = request.pageSize();

        if(totalCount <= 0) {
            return new PaginatedReviewsResponseDto(Collections.emptyList(), 0, page, 0);
        }

        int offset = (page - 1) * pageSize;

        List<Review> reviews = reviewRepository.findWithPagination(
                request.minRating(),
                request.sortOption().name(),
                pageSize,
                offset
        );

        List<ReviewResponseDto> reviewResponseDtos = reviews.stream().map(reviewMapper::toDto)
                .toList();

        int totalPages = (int) Math.ceil((double) totalCount / pageSize);

        return new  PaginatedReviewsResponseDto(
                reviewResponseDtos,
                totalCount,
                request.page(),
                totalPages
        );
    }

    @Override
    public ReviewStatisticsResponseDto getReviewStatistics() {
        ReviewStatsRawData rawData = reviewRepository.findReviewStatistics();

        Map<String, Long> ratingRangeCounts = new HashMap<>();
        ratingRangeCounts.put("1점", rawData.getRating1Count());
        ratingRangeCounts.put("2점", rawData.getRating2Count());
        ratingRangeCounts.put("3점", rawData.getRating3Count());
        ratingRangeCounts.put("4점", rawData.getRating4Count());
        ratingRangeCounts.put("5점", rawData.getRating5Count());

        return new ReviewStatisticsResponseDto(
                rawData.getAverageRating(),
                ratingRangeCounts
        );
    }
}
