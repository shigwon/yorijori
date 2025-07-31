package com.linky.api.common.response.enums;

import org.springframework.http.HttpStatus;

import lombok.Getter;
import lombok.RequiredArgsConstructor;

@Getter
@RequiredArgsConstructor
public enum ResponseEnum {

    OK(HttpStatus.OK, "success", "성공"),
    FAIL(HttpStatus.OK, "fail", "실패");

    private final HttpStatus status;
    private final String result;
    private final String message;
}
