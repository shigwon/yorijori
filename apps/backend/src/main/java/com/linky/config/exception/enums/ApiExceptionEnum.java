package com.linky.config.exception.enums;

import org.springframework.http.HttpStatus;

import lombok.Getter;

@Getter
public enum ApiExceptionEnum {

    RUNTIME_EXCEPTION(HttpStatus.INTERNAL_SERVER_ERROR, "E0001"),
    ACCESS_DENIED(HttpStatus.UNAUTHORIZED, "E0002", "인증되지 않은 사용자입니다."),
    BAD_REQUEST(HttpStatus.BAD_REQUEST, "E0003", "필수 요청 변수가 누락되었습니다."),
    METHOD_NOT_ALLOWED(HttpStatus.METHOD_NOT_ALLOWED, "E0004", "요청 메소드를 확인해주세요."),
    NOT_FOUND(HttpStatus.NOT_FOUND, "E0004", "존재하지 않는 API입니다."),
    INTERNAL_SERVER_ERROR(HttpStatus.INTERNAL_SERVER_ERROR, "E9999", "기타 오류"),
    FORBIDDEN(HttpStatus.FORBIDDEN, "S0001", "권한이 없습니다."),

    ROBOT_NOT_FOUND(HttpStatus.NOT_FOUND, "RL001", "해당 로봇을 찾을 수 없습니다."),
    LOCATION_NOT_FOUND(HttpStatus.NOT_FOUND, "RL002", "위치 정보를 찾을 수 없습니다."),
    INVALID_DATE_RANGE(HttpStatus.BAD_REQUEST, "RL003", "유효하지 않은 날짜 범위입니다."),
    INVALID_COORDINATES(HttpStatus.BAD_REQUEST, "RL004", "유효하지 않은 좌표입니다."),
    LOCATION_SAVE_FAILED(HttpStatus.INTERNAL_SERVER_ERROR, "RL005", "위치 저장에 실패했습니다."),
    REDIS_CONNECTION_FAILED(HttpStatus.INTERNAL_SERVER_ERROR, "RL006", "Redis 연결에 실패했습니다."),
    LOCATION_HISTORY_NOT_FOUND(HttpStatus.NOT_FOUND, "RL007", "위치 이력이 없습니다."),
    INVALID_ROBOT_ID(HttpStatus.BAD_REQUEST, "RL008", "유효하지 않은 로봇 ID입니다.");

    private final HttpStatus status;
    private final String code;
    private String message;

    ApiExceptionEnum(HttpStatus status, String code) {
        this.status = status;
        this.code = code;
    }

    ApiExceptionEnum(HttpStatus status, String code, String message) {
        this.status = status;
        this.code = code;
        this.message = message;
    }
}
