package com.linky.api.review.dto.response;

import java.util.Map;

public record ReviewStatisticsResponseDto(
        double averageRating,
        Map<String, Long> ratingRangeCounts
) {
}
