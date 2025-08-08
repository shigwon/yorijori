package com.linky.api.robot.exception;

import com.linky.config.exception.enums.ApiExceptionEnum;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

@Getter
@RequiredArgsConstructor
public class RobotLocationException extends RuntimeException {
    private final ApiExceptionEnum apiExceptionEnum;
    private final String customMessage;

    public RobotLocationException(ApiExceptionEnum apiExceptionEnum) {
        this.apiExceptionEnum = apiExceptionEnum;
        this.customMessage = null;
    }
}