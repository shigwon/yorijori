package com.linky.api.review.exception.enums;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.springframework.http.HttpStatus;

@Getter
@RequiredArgsConstructor
public enum ReviewExceptionResult {

    NOT_EXISTS(HttpStatus.BAD_REQUEST, "r0001", "존재하지 않는 리뷰입니다."),
    BAD_REQUEST(HttpStatus.BAD_REQUEST, "r0002", "매개 변수가 잘못 되었습니다.");

    private final HttpStatus status;
    private final String code;
    private final String message;
}
