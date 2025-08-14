package com.linky.api.review.mapper;

import com.linky.api.review.dto.request.ReviewCreateRequestDto;
import com.linky.api.review.dto.response.ReviewResponseDto;
import com.linky.api.review.entity.Review;
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
}
