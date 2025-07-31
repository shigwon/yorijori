package com.linky.api.review.exception;

import com.linky.api.review.exception.enums.ReviewExceptionResult;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

@Getter
@RequiredArgsConstructor
public class ReviewException extends RuntimeException {

    private final ReviewExceptionResult reviewExceptionResult;
}
