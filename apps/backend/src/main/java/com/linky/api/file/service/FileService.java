package com.linky.api.file.service;

import com.linky.api.file.dto.request.FileDownloadRequestDto;
import com.linky.api.file.dto.request.FileUploadRequestDto;
import org.springframework.web.multipart.MultipartFile;

public interface FileService {

    String uploadFileToS3(FileUploadRequestDto request, MultipartFile file);
    byte[] downloadFileToS3(FileDownloadRequestDto request);
    byte[] downloadFileToS3ByUrl(String url);
}
