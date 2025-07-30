package com.linky.api.file.service;

import com.linky.api.file.dto.request.FileDownloadRequestDto;
import com.linky.api.file.dto.request.FileUploadRequestDto;

public interface FileService {

    String uploadFileToS3(FileUploadRequestDto request);
    byte[] downloadFileToS3(FileDownloadRequestDto request);
}
