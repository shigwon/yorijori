package com.linky.api.review.repository;

import com.linky.api.review.entity.Review;
import com.linky.api.review.entity.ReviewStatsRawData;
import org.apache.ibatis.annotations.Mapper;

import java.util.List;

@Mapper
public interface ReviewRepository {

    int insertReview(Review newReview);

    Review findById(int id);

    long countByCriteria(int minRating);

    List<Review> findWithPagination(int minRating, String sortOption, int limit, int offset);

    ReviewStatsRawData findReviewStatistics();
}
