package com.linky.api.admin.grpc;

import com.linky.api.admin.service.AdminService;
import com.linky.admin.grpc.*;
import io.grpc.Status;
import io.grpc.stub.StreamObserver;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import net.devh.boot.grpc.server.service.GrpcService;

import java.time.LocalDate;
import java.time.format.DateTimeFormatter;

@Slf4j
@RequiredArgsConstructor
@GrpcService
public class AdminGrpcService extends AdminServiceGrpc.AdminServiceImplBase {

    private final AdminService adminService;
    private static final DateTimeFormatter DATE_FORMATTER = DateTimeFormatter.ofPattern("yyyy-MM-dd");

    @Override
    public void getDailyOrderCount(DailyOrderCountRequest request,
                                   StreamObserver<OrderCountResponse> responseObserver) {
        try {
            LocalDate date = LocalDate.parse(request.getDate(), DATE_FORMATTER);
            int orderCount = adminService.getDailyOrderCount(date);

            OrderCountResponse response = OrderCountResponse.newBuilder()
                    .setOrderCount(orderCount)
                    .build();

            responseObserver.onNext(response);
            responseObserver.onCompleted();

        } catch (Exception e) {
            log.error("일별 주문 건수 조회 중 오류 발생", e);
            responseObserver.onError(
                    Status.INTERNAL
                            .withDescription("일별 주문 건수 조회 중 오류가 발생했습니다: " + e.getMessage())
                            .withCause(e)
                            .asRuntimeException()
            );
        }
    }

    @Override
    public void getWeeklyOrderCount(WeeklyOrderCountRequest request,
                                    StreamObserver<OrderCountResponse> responseObserver) {
        try {
            LocalDate startDate = LocalDate.parse(request.getStartDate(), DATE_FORMATTER);
            int orderCount = adminService.getWeeklyOrderCount(startDate);

            OrderCountResponse response = OrderCountResponse.newBuilder()
                    .setOrderCount(orderCount)
                    .build();

            responseObserver.onNext(response);
            responseObserver.onCompleted();

        } catch (Exception e) {
            log.error("주간별 주문 건수 조회 중 오류 발생", e);
            responseObserver.onError(
                    Status.INTERNAL
                            .withDescription("주간별 주문 건수 조회 중 오류가 발생했습니다: " + e.getMessage())
                            .withCause(e)
                            .asRuntimeException()
            );
        }
    }

    @Override
    public void getMonthlyOrderCount(MonthlyOrderCountRequest request,
                                     StreamObserver<OrderCountResponse> responseObserver) {
        try {
            int orderCount = adminService.getMonthlyOrderCount(
                    request.getYear(),
                    request.getMonth()
            );

            OrderCountResponse response = OrderCountResponse.newBuilder()
                    .setOrderCount(orderCount)
                    .build();

            responseObserver.onNext(response);
            responseObserver.onCompleted();

        } catch (Exception e) {
            log.error("월별 주문 건수 조회 중 오류 발생", e);
            responseObserver.onError(
                    Status.INTERNAL
                            .withDescription("월별 주문 건수 조회 중 오류가 발생했습니다: " + e.getMessage())
                            .withCause(e)
                            .asRuntimeException()
            );
        }
    }

    @Override
    public void getOrderCountByPeriod(PeriodOrderCountRequest request,
                                      StreamObserver<OrderCountResponse> responseObserver) {
        try {
            LocalDate startDate = LocalDate.parse(request.getStartDate(), DATE_FORMATTER);
            LocalDate endDate = LocalDate.parse(request.getEndDate(), DATE_FORMATTER);

            int orderCount = adminService.getOrderCountByPeriod(startDate, endDate);

            OrderCountResponse response = OrderCountResponse.newBuilder()
                    .setOrderCount(orderCount)
                    .build();

            responseObserver.onNext(response);
            responseObserver.onCompleted();

        } catch (Exception e) {
            log.error("기간별 주문 건수 조회 중 오류 발생", e);
            responseObserver.onError(
                    Status.INTERNAL
                            .withDescription("기간별 주문 건수 조회 중 오류가 발생했습니다: " + e.getMessage())
                            .withCause(e)
                            .asRuntimeException()
            );
        }
    }
}