package com.linky.api.file.service;

import com.linky.file.grpc.FileDownloadRequest;
import com.linky.file.grpc.FileDownloadResponse;
import com.linky.file.grpc.FileUploadRequest;
import com.linky.file.grpc.FileUploadResponse;

public interface FileService {

    FileUploadResponse uploadFileToS3(FileUploadRequest request);
    FileDownloadResponse downloadFileToS3(FileDownloadRequest request);
}
