package com.linky.api.file.service.impl;

import com.amazonaws.services.s3.AmazonS3;
import com.amazonaws.services.s3.model.S3Object;
import com.amazonaws.services.s3.model.S3ObjectInputStream;
import com.linky.api.file.service.FileService;
import com.linky.api.order.service.OrderService;
import com.linky.file.grpc.FileDownloadRequest;
import com.linky.file.grpc.FileDownloadResponse;
import com.linky.file.grpc.FileUploadRequest;
import com.linky.file.grpc.FileUploadResponse;
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
    public FileUploadResponse uploadFileToS3(FileUploadRequest request) {
        String orderCode = request.getOrderCode();
        String fileName = request.getFileName();
        String fileType = request.getFileType();
        String fileCategory = request.getFileCategory();
        byte[] fileData = request.getFileData().toByteArray();

        if(fileData.length == 0){
            throw new IllegalArgumentException("file data is empty");
        }

        String s3Key = getS3Key(orderCode, fileName, fileType, fileCategory);

        try {
            amazonS3.putObject(bucket, s3Key, new java.io.ByteArrayInputStream(fileData), null);
            String fileUrl = amazonS3.getUrl(bucket, s3Key).toString();

            return FileUploadResponse.newBuilder().setFileUrl(fileUrl).build();
        } catch (Exception e) {
            log.error("uploadFileToS3 error", e.getMessage());
            e.printStackTrace();
            return FileUploadResponse.newBuilder().setFileUrl("").build();
        }
    }

    public FileDownloadResponse downloadFileToS3(FileDownloadRequest request) {
        String orderCode = request.getOrderCode();
        String fileName = request.getFileName();
        String fileType = request.getFileType();

        String fileCategory = request.getFileCategory();

        String s3Key = getS3Key(orderCode, fileName, fileType, fileCategory);

        try {
            S3Object s3Object = amazonS3.getObject(bucket, s3Key);
            S3ObjectInputStream inputStream = s3Object.getObjectContent();
            byte[] bytes = com.amazonaws.util.IOUtils.toByteArray(inputStream);
            return FileDownloadResponse.newBuilder().setFileData(com.google.protobuf.ByteString.copyFrom(bytes)).build();
        } catch (Exception e) {
            log.error("downloadFileToS3 error {}", e.getMessage());
            e.printStackTrace();
            return FileDownloadResponse.newBuilder().build();
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
