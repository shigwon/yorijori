package com.linky.api.file.exception.enums;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.springframework.http.HttpStatus;

@Getter
@RequiredArgsConstructor
public enum FileExceptionResult {

    NOT_EXISTS(HttpStatus.BAD_REQUEST, "f0001", "존재하지 않는 파일입니다."),
    BAD_REQUEST(HttpStatus.BAD_REQUEST, "f0002", "매개 변수가 잘못 되었습니다."),
    FAIL_UPLOAD(HttpStatus.INTERNAL_SERVER_ERROR, "f0003", "파일 업로드에 실패 했습니다."),
    FAIL_DOWNLOAD(HttpStatus.INTERNAL_SERVER_ERROR, "f0004", "파일 다운로드에 실패 했습니다.");

    private final HttpStatus status;
    private final String code;
    private final String message;
}