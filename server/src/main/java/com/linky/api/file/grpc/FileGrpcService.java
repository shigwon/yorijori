package com.linky.api.file.grpc;

import com.google.protobuf.ByteString;
import com.linky.api.file.mapper.FileMapper;
import com.linky.api.file.service.FileService;
import com.linky.file.grpc.*;
import io.grpc.stub.StreamObserver;
import lombok.RequiredArgsConstructor;
import net.devh.boot.grpc.server.service.GrpcService;

@GrpcService
@RequiredArgsConstructor
public class FileGrpcService extends FileServiceGrpc.FileServiceImplBase {

    private final FileService fileService;
    private final FileMapper fileMapper;

    @Override
    public void uploadFile(FileUploadRequest request, StreamObserver<FileUploadResponse> responseObserver) {
        String response = fileService.uploadFileToS3(fileMapper.toDto(request));

        responseObserver.onNext(FileUploadResponse.newBuilder().setFileUrl(response).build());
        responseObserver.onCompleted();
    }

    @Override
    public void downloadFile(FileDownloadRequest request, StreamObserver<FileDownloadResponse> responseObserver) {
        byte[] response = fileService.downloadFileToS3(fileMapper.toDto(request));

        responseObserver.onNext(FileDownloadResponse.newBuilder().setFileData(ByteString.copyFrom(response)).build());
        responseObserver.onCompleted();
    }
}
