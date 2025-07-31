package com.linky.api.review.entity;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.time.LocalDateTime;

@Data
@AllArgsConstructor
@NoArgsConstructor
@Builder
public class Review {

    int id;
    int orderId;
    int rating;
    String content;
    LocalDateTime createdAt;
}
