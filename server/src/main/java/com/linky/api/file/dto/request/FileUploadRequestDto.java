package com.linky.api.file.dto.request;

import com.linky.api.file.enums.FileCategory;

public record FileUploadRequestDto(
        String orderCode,
        String fileName,
        String fileType,
        FileCategory fileCategory,
        byte[] fileData
) {
}
