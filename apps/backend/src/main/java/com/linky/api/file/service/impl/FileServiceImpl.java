package com.linky.api.file.service.impl;

import com.amazonaws.services.s3.AmazonS3;
import com.amazonaws.services.s3.model.ObjectMetadata;
import com.amazonaws.services.s3.model.S3Object;
import com.amazonaws.services.s3.model.S3ObjectInputStream;
import com.amazonaws.util.IOUtils;
import com.linky.api.file.dto.request.FileDownloadRequestDto;
import com.linky.api.file.dto.request.FileUploadRequestDto;
import com.linky.api.file.exception.FileException;
import com.linky.api.file.exception.enums.FileExceptionResult;
import com.linky.api.file.service.FileService;
import com.linky.api.order.repository.OrderRepository;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import org.springframework.web.multipart.MultipartFile;

import java.net.URI;

@Service
@RequiredArgsConstructor
@Slf4j
public class FileServiceImpl implements FileService {

    @Value("${spring.cloud.aws.s3.bucket}")
    private String bucket;

    private final AmazonS3 amazonS3;
    private final OrderRepository orderRepository;

    @Transactional
    @Override
    public String uploadFileToS3(FileUploadRequestDto request, MultipartFile file) {
        String orderCode = request.orderCode();
        String fileName = file.getOriginalFilename();
        String fileCategory = request.fileCategory().name();

        String s3Key = getS3Key(orderCode, fileName, fileCategory);

        try {
            ObjectMetadata metadata = new ObjectMetadata();
            metadata.setContentLength(file.getSize());
            metadata.setContentType(file.getContentType());
            amazonS3.putObject(bucket, s3Key, file.getInputStream(), metadata);

            String url = amazonS3.getUrl(bucket, s3Key).toString();
            orderRepository.updateImageUrlByCode(orderCode, url, fileCategory);

            return url;
        } catch (Exception e) {
            throw new FileException(FileExceptionResult.FAIL_UPLOAD);
        }
    }

    @Override
    public byte[] downloadFileToS3(FileDownloadRequestDto request) {
        String orderCode = request.orderCode();
        String fileName = request.fileName();
        String fileCategory = request.fileCategory().name();

        String s3Key = getS3Key(orderCode, fileName, fileCategory);

        try(S3Object s3Object = amazonS3.getObject(bucket, s3Key)) {
            S3ObjectInputStream inputStream = s3Object.getObjectContent();

            return IOUtils.toByteArray(inputStream);
        } catch (Exception e) {
            throw new FileException(FileExceptionResult.FAIL_UPLOAD);
        }
    }

    @Override
    public byte[] downloadFileToS3ByUrl(String url) {
        if(url == null || url.isEmpty()){
            throw new FileException(FileExceptionResult.BAD_REQUEST);
        }

        try {
            URI uri = new URI(url);

            String bucket = extractBucketFromHost(uri.getHost());
            String key = uri.getPath().substring(1);

            try(S3Object s3Object = amazonS3.getObject(bucket, key)) {
                S3ObjectInputStream inputStream = s3Object.getObjectContent();

                return IOUtils.toByteArray(inputStream);
            }
        } catch (Exception e) {
            throw new FileException(FileExceptionResult.FAIL_DOWNLOAD);
        }
    }

    private String extractBucketFromHost(String host) {
        int firstDotIndex = host.indexOf('.');

        if(firstDotIndex == -1 || !host.contains(".s3.")){
            throw new FileException(FileExceptionResult.BAD_REQUEST);
        }

        return host.substring(0, firstDotIndex);
    }

    private static String getS3Key(String orderCode, String fileName, String fileCategory) {
        if (orderCode.trim().isEmpty() || fileName.trim().isEmpty() || fileCategory.trim().isEmpty()) {
            throw new FileException(FileExceptionResult.BAD_REQUEST);
        }

        return "orders/" + orderCode + "/" + fileCategory + "/" + fileName;
    }
}
