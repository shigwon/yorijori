package com.linky.api.review.mapper;

import com.linky.api.review.dto.request.PaginatedReviewsRequestDto;
import com.linky.api.review.dto.request.ReviewCreateRequestDto;
import com.linky.api.review.dto.response.PaginatedReviewsResponseDto;
import com.linky.api.review.dto.response.ReviewResponseDto;
import com.linky.api.review.dto.response.ReviewStatisticsResponseDto;
import com.linky.api.review.entity.Review;
import com.linky.api.review.enums.SortOption;
import com.linky.review.grpc.*;
import com.linky.util.TimestampConverter;
import org.springframework.stereotype.Component;

import java.time.LocalDateTime;

@Component
public class ReviewMapper {

    public Review toEntity(ReviewCreateRequestDto reviewCreateRequestDto) {
        return Review.builder()
                .rating(reviewCreateRequestDto.rating())
                .content(reviewCreateRequestDto.content())
                .createdAt(LocalDateTime.now())
                .build();
    }

    public ReviewResponseDto toDto(Review review) {
        return new ReviewResponseDto(
                review.getId(),
                String.valueOf(review.getOrderId()),
                review.getRating(),
                review.getContent(),
                review.getCreatedAt()
        );
    }

    public ReviewCreateRequestDto toDto(ReviewCreateRequest reviewCreateRequest) {
        return  new ReviewCreateRequestDto(
                reviewCreateRequest.getOrderCode(),
                reviewCreateRequest.getRating(),
                reviewCreateRequest.getContent()
        );
    }

    public PaginatedReviewsRequestDto toDto(PaginatedReviewsRequest paginatedReviewsRequest) {
        return new PaginatedReviewsRequestDto(
                paginatedReviewsRequest.getPage(),
                paginatedReviewsRequest.getPageSize(),
                paginatedReviewsRequest.getMinRating(),
                SortOption.valueOf(paginatedReviewsRequest.getSortOption().name())
        );
    }

    public ReviewResponse toProto(ReviewResponseDto reviewResponseDto) {
        return ReviewResponse.newBuilder()
                .setId(reviewResponseDto.id())
                .setOrderCode(reviewResponseDto.orderCode())
                .setRating(reviewResponseDto.rating())
                .setContent(reviewResponseDto.content())
                .setCreatedAt(TimestampConverter.toTimestamp(reviewResponseDto.createdAt()))
                .build();
    }

    public PaginatedReviewsResponse toProto(PaginatedReviewsResponseDto paginatedReviewsResponse) {
        return PaginatedReviewsResponse.newBuilder()
                .addAllData(paginatedReviewsResponse.reviews().stream().map(this::toProto).toList())
                .setTotalCount(paginatedReviewsResponse.totalCount())
                .setCurrentPage(paginatedReviewsResponse.currentPage())
                .setTotalPages(paginatedReviewsResponse.totalPages())
                .build();
    }

    public ReviewStatisticsResponse toProto(ReviewStatisticsResponseDto reviewStatisticsResponseDto) {
        return ReviewStatisticsResponse.newBuilder()
                .setAverageRating(reviewStatisticsResponseDto.averageRating())
                .putAllRatingRangeCounts(reviewStatisticsResponseDto.ratingRangeCounts())
                .build();
    }
}
