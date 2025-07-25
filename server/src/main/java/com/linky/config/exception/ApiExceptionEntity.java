package com.linky.config.exception;

import lombok.Builder;

@Builder
public record ApiExceptionEntity(String errorCode, String errorMsg) {
}
