package com.linky.api.file.dto.request;

import com.linky.api.file.enums.FileCategory;

public record FileDownloadRequestDto(
        String orderCode,
        String fileName,
        FileCategory fileCategory
) {
}
