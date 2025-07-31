package com.linky.config.exception.file;

import com.linky.api.file.exception.FileException;
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
public class FileApiExceptionAdvice {

    @ExceptionHandler({FileException.class})
    public ResponseEntity<ApiExceptionEntity> exceptionHandler(HttpServletRequest req, final FileException e) {
        log.warn("[FileApiExceptionAdvice] FileException :: {}", e.getFileExceptionResult().getMessage());

        return ResponseEntity
                .status(e.getFileExceptionResult().getStatus())
                .body(ApiExceptionEntity.builder()
                        .errorCode(e.getFileExceptionResult().getCode())
                        .errorMsg(e.getFileExceptionResult().getMessage())
                        .build());
    }
}
