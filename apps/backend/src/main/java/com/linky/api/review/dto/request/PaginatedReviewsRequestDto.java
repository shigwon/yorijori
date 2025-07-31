package com.linky.api.review.dto.request;

import com.linky.api.review.enums.SortOption;

public record PaginatedReviewsRequestDto(
    int page,
    int pageSize,
    int minRating,
    SortOption sortOption
) {
}
