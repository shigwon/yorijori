package com.linky.config.exception.review;

import com.linky.api.review.exception.ReviewException;
import com.linky.config.exception.ApiExceptionEntity;
import jakarta.servlet.http.HttpServletRequest;
import lombok.extern.slf4j.Slf4j;
import org.springframework.core.Ordered;
import org.springframework.core.annotation.Order;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.ExceptionHandler;
import org.springframework.web.bind.annotation.RestControllerAdvice;

@RestControllerAdvice
@Order(Ordered.HIGHEST_PRECEDENCE)
@Slf4j
public class ReviewApiExceptionAdvice {

    @ExceptionHandler({ReviewException.class})
    public ResponseEntity<ApiExceptionEntity> exceptionHandler(HttpServletRequest req, final ReviewException e) {
        log.warn("[ReviewApiExceptionAdvice] ReviewException :: {}", e.getReviewExceptionResult().getMessage());

        return ResponseEntity
                .status(e.getReviewExceptionResult().getStatus())
                .body(ApiExceptionEntity.builder()
                        .errorCode(e.getReviewExceptionResult().getCode())
                        .errorMsg(e.getReviewExceptionResult().getMessage())
                        .build());
    }
}
