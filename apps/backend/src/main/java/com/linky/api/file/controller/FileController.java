package com.linky.api.file.controller;

import com.linky.api.common.response.entity.ApiResponseEntity;
import com.linky.api.file.dto.request.FileDownloadRequestDto;
import com.linky.api.file.dto.request.FileUploadRequestDto;
import com.linky.api.file.service.FileService;
import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;
import org.springframework.web.multipart.MultipartFile;

@RestController
@RequestMapping("/api/v1/files")
@RequiredArgsConstructor
public class FileController {

    private final FileService fileService;

    @PostMapping
    public ResponseEntity<ApiResponseEntity<String>> uploadFileToS3(@ModelAttribute FileUploadRequestDto request, @RequestPart MultipartFile file) {
        return ApiResponseEntity.successResponseEntity(fileService.uploadFileToS3(request, file));
    }

    @GetMapping
    public ResponseEntity<ApiResponseEntity<byte[]>> downloadFileToS3(@RequestBody FileDownloadRequestDto  request) {
        return ApiResponseEntity.successResponseEntity(fileService.downloadFileToS3(request));
    }
}
