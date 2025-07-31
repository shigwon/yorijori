package com.linky.api.file.dto.request;

import com.linky.api.file.enums.FileCategory;

public record FileUploadRequestDto(
        String orderCode,
        FileCategory fileCategory
) {
}
