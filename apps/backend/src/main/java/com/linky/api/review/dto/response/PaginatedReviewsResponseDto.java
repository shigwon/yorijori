package com.linky.api.review.dto.response;

import java.util.List;

public record PaginatedReviewsResponseDto(
    List<ReviewResponseDto> reviews,
    long totalCount,
    int currentPage,
    int totalPages
) {
}
