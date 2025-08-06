package com.linky.api.common.response.entity;

import java.util.Objects;

import com.linky.api.common.response.enums.ResponseEnum;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;

import lombok.Builder;
import lombok.extern.slf4j.Slf4j;

@Slf4j
public record ApiResponseEntity<T>(
        String result,
        String msg,
        T data
) {

    @Builder
    public ApiResponseEntity(String result, String msg, T data) {
        this.result = (result == null || result.isEmpty()) ? ResponseEnum.OK.getResult() : result;
        this.msg = (msg == null || msg.isEmpty()) ? ResponseEnum.OK.getMessage() : msg;
        this.data = data;
    }

    public static <T> ResponseEntity<ApiResponseEntity<T>> successResponseEntity() {
        ApiResponseEntity<T> body = ApiResponseEntity.<T>builder()
                .msg("요청 성공")
                .build();
        log.info("[API SUCCESS] result={}, msg={}", body.result, body.msg);
        return ResponseEntity.ok(body);
    }

    public static <T> ResponseEntity<ApiResponseEntity<T>> successResponseEntity(T data) {
        ApiResponseEntity<T> body = ApiResponseEntity.<T>builder()
                .data(data)
                .msg("요청 성공")
                .build();
        log.info("[API SUCCESS] result={}, msg={}, data={}", body.result, body.msg, data);
        return ResponseEntity.ok(body);
    }

    public static <T> ResponseEntity<ApiResponseEntity<T>> failResponseEntity(String msg) {
        ApiResponseEntity<T> body = ApiResponseEntity.<T>builder()
                .result(ResponseEnum.FAIL.getResult())
                .msg(!Objects.equals(msg, "요청 실패") ? msg : ResponseEnum.FAIL.getMessage())
                .build();
        log.warn("[API FAIL] result={}, msg={}", body.result, body.msg);
        return ResponseEntity.status(HttpStatus.BAD_REQUEST).body(body);
    }

    public static <T> ResponseEntity<ApiResponseEntity<T>> failResponseEntity(T data, String msg) {
        ApiResponseEntity<T> body = ApiResponseEntity.<T>builder()
                .result(ResponseEnum.FAIL.getResult())
                .msg(!Objects.equals(msg, "요청 실패") ? msg : ResponseEnum.FAIL.getMessage())
                .data(data)
                .build();
        log.warn("[API FAIL] result={}, msg={}, data={}", body.result, body.msg, data);
        return ResponseEntity.status(HttpStatus.BAD_REQUEST).body(body);
    }

    public static <T> ResponseEntity<ApiResponseEntity<T>> unauthorizedResponseEntity(String msg) {
        ApiResponseEntity<T> body = ApiResponseEntity.<T>builder()
                .result("unauthorized")
                .msg(msg != null ? msg : "인증이 필요합니다")
                .build();
        log.warn("[API UNAUTHORIZED] result={}, msg={}", body.result, body.msg);
        return ResponseEntity.status(HttpStatus.UNAUTHORIZED).body(body);
    }

    public static <T> ResponseEntity<ApiResponseEntity<T>> unauthorizedResponseEntity(T data, String msg) {
        ApiResponseEntity<T> body = ApiResponseEntity.<T>builder()
                .result("unauthorized")
                .msg(msg != null ? msg : "인증이 필요합니다")
                .data(data)
                .build();
        log.warn("[API UNAUTHORIZED] result={}, msg={}, data={}", body.result, body.msg, data);
        return ResponseEntity.status(HttpStatus.UNAUTHORIZED).body(body);
    }
}
