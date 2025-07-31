package com.linky.api.file.mapper;

import com.linky.api.file.dto.request.FileDownloadRequestDto;
import com.linky.api.file.dto.request.FileUploadRequestDto;
import com.linky.api.file.enums.FileCategory;
import com.linky.file.grpc.FileDownloadRequest;
import com.linky.file.grpc.FileUploadRequest;
import org.springframework.stereotype.Component;

@Component
public class FileMapper {

    public FileUploadRequestDto toDto(FileUploadRequest fileUploadRequest) {
        return new FileUploadRequestDto(
                fileUploadRequest.getOrderCode(),
                FileCategory.valueOf(fileUploadRequest.getFileCategory().name())
        );
    }

    public FileDownloadRequestDto toDto(FileDownloadRequest fileDownloadRequest) {
        return new FileDownloadRequestDto(
                fileDownloadRequest.getOrderCode(),
                fileDownloadRequest.getFileName(),
                FileCategory.valueOf(fileDownloadRequest.getFileCategory().name())
        );
    }
}
