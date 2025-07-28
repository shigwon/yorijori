package com.linky.api.file.grpc;

import com.linky.api.file.service.FileService;
import com.linky.file.grpc.*;
import io.grpc.stub.StreamObserver;
import lombok.RequiredArgsConstructor;
import net.devh.boot.grpc.server.service.GrpcService;

@GrpcService
@RequiredArgsConstructor
public class FileGrpcService extends FileServiceGrpc.FileServiceImplBase {

    private final FileService fileService;

    @Override
    public void uploadFile(FileUploadRequest request, StreamObserver<FileUploadResponse> responseObserver) {
        FileUploadResponse response = fileService.uploadFileToS3(request);

        responseObserver.onNext(response);
        responseObserver.onCompleted();
    }

    @Override
    public void downloadFile(FileDownloadRequest request, StreamObserver<FileDownloadResponse> responseObserver) {
        FileDownloadResponse response = fileService.downloadFileToS3(request);

        responseObserver.onNext(response);
        responseObserver.onCompleted();
    }
}
