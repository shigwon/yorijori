package com.linky.api.admin.controller;

import com.linky.api.admin.dto.request.*;
import com.linky.api.admin.dto.response.*;
import com.linky.api.admin.service.AdminService;
import com.linky.api.common.response.entity.ApiResponseEntity;
import com.linky.api.robot.dto.RobotListResponseDto;
import com.linky.api.robot.dto.RobotWithLocationDto;
import com.linky.api.robot.service.RobotService;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpSession;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.validation.annotation.Validated;
import org.springframework.web.bind.annotation.*;

import jakarta.validation.Valid;
import java.time.LocalDate;
import java.time.format.DateTimeFormatter;
import java.time.format.DateTimeParseException;
import java.util.List;

@Slf4j
@RestController
@RequestMapping("/api/admin")
@RequiredArgsConstructor
@Validated
public class AdminController {

    private final AdminService adminService;
    private final RobotService robotService;
    private static final DateTimeFormatter DATE_FORMATTER = DateTimeFormatter.ofPattern("yyyy-MM-dd");

    private boolean isAuthenticated(HttpServletRequest request) {
        HttpSession session = request.getSession(false);
        return adminService.isValidSession(session);
    }

    @PostMapping("/login")
    public ResponseEntity<ApiResponseEntity<LoginResponseDto>> login(
            @Valid @RequestBody LoginRequestDto request,
            HttpServletRequest httpRequest) {

        try {
            HttpSession session = httpRequest.getSession(true); // 세션 생성
            String adminName = adminService.loginWithSession(
                    request.getEmail(),
                    request.getPassword(),
                    session
            );

            LoginResponseDto response;
            if (adminName != null) {
                response = LoginResponseDto.builder()
                        .success(true)
                        .name(adminName)
                        .message("로그인에 성공했습니다.")
                        .sessionId(session.getId()) // 세션 ID 반환
                        .build();

                log.info("관리자 세션 로그인 성공: {} (Session: {})",
                        request.getEmail(), session.getId());
                return ApiResponseEntity.successResponseEntity(response);

            } else {
                // 로그인 실패 시 세션 무효화
                session.invalidate();

                response = LoginResponseDto.builder()
                        .success(false)
                        .message("이메일 또는 비밀번호가 올바르지 않습니다.")
                        .build();

                log.warn("관리자 세션 로그인 실패: {}", request.getEmail());
                return ApiResponseEntity.failResponseEntity(response, "로그인 실패");
            }

        } catch (Exception e) {
            log.error("세션 로그인 처리 중 오류 발생", e);
            return ApiResponseEntity.failResponseEntity("로그인 처리 중 오류가 발생했습니다: " + e.getMessage());
        }
    }

    @PostMapping("/logout")
    public ResponseEntity<ApiResponseEntity<LogoutResponseDto>> logout(
            HttpServletRequest httpRequest) {

        try {
            HttpSession session = httpRequest.getSession(false);
            boolean logoutSuccess = adminService.logoutWithSession(session);

            LogoutResponseDto response;
            if (logoutSuccess) {
                response = LogoutResponseDto.builder()
                        .success(true)
                        .message("로그아웃에 성공했습니다.")
                        .build();

                log.info("관리자 세션 로그아웃 성공");
                return ApiResponseEntity.successResponseEntity(response);

            } else {
                response = LogoutResponseDto.builder()
                        .success(false)
                        .message("유효하지 않은 세션입니다.")
                        .build();

                log.warn("관리자 세션 로그아웃 실패: 유효하지 않은 세션");
                return ApiResponseEntity.failResponseEntity(response, "로그아웃 실패");
            }

        } catch (Exception e) {
            log.error("세션 로그아웃 처리 중 오류 발생", e);
            return ApiResponseEntity.failResponseEntity("로그아웃 처리 중 오류가 발생했습니다: " + e.getMessage());
        }
    }

    @GetMapping("/orders/count/daily")
    public ResponseEntity<ApiResponseEntity<OrderCountResponseDto>> getDailyOrderCount(
            @Valid DailyOrderCountRequestDto request) {

        try {
            LocalDate date = LocalDate.parse(request.getDate(), DATE_FORMATTER);
            int orderCount = adminService.getDailyOrderCount(date);

            OrderCountResponseDto response = OrderCountResponseDto.builder()
                    .orderCount(orderCount)
                    .build();

            return ApiResponseEntity.successResponseEntity(response);

        } catch (DateTimeParseException e) {
            log.error("날짜 파싱 오류: {}", request.getDate(), e);
            return ApiResponseEntity.failResponseEntity("올바르지 않은 날짜 형식입니다");

        } catch (Exception e) {
            log.error("일별 주문 건수 조회 중 오류 발생", e);
            return ApiResponseEntity.failResponseEntity("일별 주문 건수 조회 중 오류가 발생했습니다: " + e.getMessage());
        }
    }

    @GetMapping("/orders/count/weekly")
    public ResponseEntity<ApiResponseEntity<OrderCountResponseDto>> getWeeklyOrderCount(
            @Valid WeeklyOrderCountRequestDto request) {

        try {
            LocalDate startDate = LocalDate.parse(request.getStartDate(), DATE_FORMATTER);
            int orderCount = adminService.getWeeklyOrderCount(startDate);

            OrderCountResponseDto response = OrderCountResponseDto.builder()
                    .orderCount(orderCount)
                    .build();

            return ApiResponseEntity.successResponseEntity(response);

        } catch (DateTimeParseException e) {
            log.error("날짜 파싱 오류: {}", request.getStartDate(), e);
            return ApiResponseEntity.failResponseEntity("올바르지 않은 날짜 형식입니다");

        } catch (Exception e) {
            log.error("주별 주문 건수 조회 중 오류 발생", e);
            return ApiResponseEntity.failResponseEntity("주별 주문 건수 조회 중 오류가 발생했습니다: " + e.getMessage());
        }
    }

    @GetMapping("/orders/count/monthly")
    public ResponseEntity<ApiResponseEntity<OrderCountResponseDto>> getMonthlyOrderCount(
            @Valid MonthlyOrderCountRequestDto request) {

        try {
            int orderCount = adminService.getMonthlyOrderCount(request.getYear(), request.getMonth());

            OrderCountResponseDto response = OrderCountResponseDto.builder()
                    .orderCount(orderCount)
                    .build();

            return ApiResponseEntity.successResponseEntity(response);

        } catch (Exception e) {
            log.error("월별 주문 건수 조회 중 오류 발생", e);
            return ApiResponseEntity.failResponseEntity("월별 주문 건수 조회 중 오류가 발생했습니다: " + e.getMessage());
        }
    }

    @GetMapping("/orders/count/period")
    public ResponseEntity<ApiResponseEntity<OrderCountResponseDto>> getOrderCountByPeriod(
            @Valid PeriodOrderCountRequestDto request) {

        try {
            LocalDate startDate = LocalDate.parse(request.getStartDate(), DATE_FORMATTER);
            LocalDate endDate = LocalDate.parse(request.getEndDate(), DATE_FORMATTER);

            // 날짜 순서 검증
            if (startDate.isAfter(endDate)) {
                return ApiResponseEntity.failResponseEntity("시작 날짜는 종료 날짜보다 이전이어야 합니다");
            }

            int orderCount = adminService.getOrderCountByPeriod(startDate, endDate);

            OrderCountResponseDto response = OrderCountResponseDto.builder()
                    .orderCount(orderCount)
                    .build();

            return ApiResponseEntity.successResponseEntity(response);

        } catch (DateTimeParseException e) {
            log.error("날짜 파싱 오류: startDate={}, endDate={}", request.getStartDate(), request.getEndDate(), e);
            return ApiResponseEntity.failResponseEntity("올바르지 않은 날짜 형식입니다");

        } catch (Exception e) {
            log.error("기간별 주문 건수 조회 중 오류 발생", e);
            return ApiResponseEntity.failResponseEntity("기간별 주문 건수 조회 중 오류가 발생했습니다: " + e.getMessage());
        }
    }

    @GetMapping("/orders/statistics/hourly")
    public ResponseEntity<ApiResponseEntity<HourlyOrderCountResponseDto>> getHourlyOrderCountInDay(
            @Valid DailyOrderCountRequestDto request) {

        try {
            LocalDate date = LocalDate.parse(request.getDate(), DATE_FORMATTER);

            // 수정된 서비스에서 Model 객체를 반환
            var hourlyData = adminService.getHourlyOrderCountInDay(date);

            // Model -> DTO 변환
            List<HourlyCountDto> hourlyCountDtos = hourlyData.stream()
                    .map(modelData -> HourlyCountDto.builder()
                            .hour(modelData.getHour())
                            .count(modelData.getCount())
                            .build())
                    .toList();

            HourlyOrderCountResponseDto response = HourlyOrderCountResponseDto.builder()
                    .hourlyData(hourlyCountDtos)
                    .build();

            return ApiResponseEntity.successResponseEntity(response);

        } catch (DateTimeParseException e) {
            log.error("날짜 파싱 오류: {}", request.getDate(), e);
            return ApiResponseEntity.failResponseEntity("올바르지 않은 날짜 형식입니다");

        } catch (Exception e) {
            log.error("시간대별 주문 통계 조회 중 오류 발생", e);
            return ApiResponseEntity.failResponseEntity("시간대별 주문 통계 조회 중 오류가 발생했습니다: " + e.getMessage());
        }
    }

    @GetMapping("/orders/statistics/weekly")
    public ResponseEntity<ApiResponseEntity<WeeklyOrderCountResponseDto>> getDailyOrderCountInWeek(
            @Valid WeeklyOrderCountRequestDto request) {

        try {
            LocalDate startDate = LocalDate.parse(request.getStartDate(), DATE_FORMATTER);

            // 수정된 서비스에서 Model 객체를 반환
            var dailyData = adminService.getDailyOrderCountInWeek(startDate);

            // Model -> DTO 변환
            List<DailyCountDto> dailyCountDtos = dailyData.stream()
                    .map(modelData -> DailyCountDto.builder()
                            .dayOfWeek(modelData.getDayOfWeek())
                            .dayName(modelData.getDayName())
                            .count(modelData.getCount())
                            .build())
                    .toList();

            WeeklyOrderCountResponseDto response = WeeklyOrderCountResponseDto.builder()
                    .dailyData(dailyCountDtos)
                    .build();

            return ApiResponseEntity.successResponseEntity(response);

        } catch (DateTimeParseException e) {
            log.error("날짜 파싱 오류: {}", request.getStartDate(), e);
            return ApiResponseEntity.failResponseEntity("올바르지 않은 날짜 형식입니다");

        } catch (Exception e) {
            log.error("주간 요일별 주문 통계 조회 중 오류 발생", e);
            return ApiResponseEntity.failResponseEntity("주간 요일별 주문 통계 조회 중 오류가 발생했습니다: " + e.getMessage());
        }
    }
}