package com.linky.api.file.exception;

import com.linky.api.file.exception.enums.FileExceptionResult;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

@Getter
@RequiredArgsConstructor
public class FileException extends RuntimeException {

    private final FileExceptionResult fileExceptionResult;
}