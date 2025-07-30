package com.linky.api.file.service.impl;

import com.amazonaws.services.s3.AmazonS3;
import com.amazonaws.services.s3.model.S3Object;
import com.amazonaws.services.s3.model.S3ObjectInputStream;
import com.linky.api.file.dto.request.FileDownloadRequestDto;
import com.linky.api.file.dto.request.FileUploadRequestDto;
import com.linky.api.file.service.FileService;
import com.linky.api.order.service.OrderService;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;

@Service
@RequiredArgsConstructor
@Slf4j
public class FileServiceImpl implements FileService {

    @Value("${spring.cloud.aws.s3.bucket}")
    private String bucket;

    private final AmazonS3 amazonS3;
    private final OrderService orderService;

    @Override
    public String uploadFileToS3(FileUploadRequestDto request) {
        String orderCode = request.orderCode();
        String fileName = request.fileName();
        String fileType = request.fileType();
        String fileCategory = request.fileCategory().name();
        byte[] fileData = request.fileData();

        if(fileData.length == 0){
            throw new IllegalArgumentException("file data is empty");
        }

        String s3Key = getS3Key(orderCode, fileName, fileType, fileCategory);

        try {
            amazonS3.putObject(bucket, s3Key, new java.io.ByteArrayInputStream(fileData), null);

            return amazonS3.getUrl(bucket, s3Key).toString();
        } catch (Exception e) {
            log.error("uploadFileToS3 error", e.getMessage());
            return null;
        }
    }

    @Override
    public byte[] downloadFileToS3(FileDownloadRequestDto request) {
        String orderCode = request.orderCode();
        String fileName = request.fileName();
        String fileType = request.fileType();
        String fileCategory = request.fileCategory().name();

        String s3Key = getS3Key(orderCode, fileName, fileType, fileCategory);

        try {
            S3Object s3Object = amazonS3.getObject(bucket, s3Key);
            S3ObjectInputStream inputStream = s3Object.getObjectContent();
            byte[] bytes = com.amazonaws.util.IOUtils.toByteArray(inputStream);

            return bytes;
        } catch (Exception e) {
            log.error("downloadFileToS3 error {}", e.getMessage());
            return null;
        }
    }

    private static String getS3Key(String orderCode, String fileName, String fileType, String fileCategory) {
        if (orderCode.trim().isEmpty() || fileName.trim().isEmpty() || fileType.trim().isEmpty() || fileCategory.trim().isEmpty()) {
            throw new IllegalArgumentException("Request fields cannot be empty.");
        }

        if (!("face".equals(fileCategory) || "food".equals(fileCategory))) {
            throw new IllegalArgumentException("Invalid file category: " + fileCategory);
        }

        return "orders/" + orderCode + "/" + fileCategory + "/" + fileName + "." + fileType;
    }
}
