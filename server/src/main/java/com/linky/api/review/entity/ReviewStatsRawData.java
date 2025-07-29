package com.linky.api.review.entity;

import lombok.Data;

@Data
public class ReviewStatsRawData {
    private double averageRating;
    private long rating1Count;
    private long rating2Count;
    private long rating3Count;
    private long rating4Count;
    private long rating5Count;
}
