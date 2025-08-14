package com.linky.api.admin.controller;

import com.linky.api.admin.dto.response.OrderLogDto;
import com.linky.api.admin.service.AdminService;
import com.linky.api.common.response.entity.ApiResponseEntity;
import jakarta.servlet.http.HttpServletRequest;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.format.annotation.DateTimeFormat;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.time.LocalDateTime;
import java.util.List;

@Slf4j
@RestController
@RequestMapping("/api/admin/logs")
@RequiredArgsConstructor
public class AdminLogController {

    private final AdminService adminService;

    private boolean isAuthenticated(HttpServletRequest request) {
        return adminService.isValidSession(request.getSession(false));
    }

    @GetMapping
    public ResponseEntity<ApiResponseEntity<List<OrderLogDto>>> getOrderLogs(
            @RequestParam(required = false) String state,
            @RequestParam(required = false)
            @DateTimeFormat(pattern = "yyyy-MM-dd HH:mm:ss") LocalDateTime startTime,
            @RequestParam(required = false)
            @DateTimeFormat(pattern = "yyyy-MM-dd HH:mm:ss") LocalDateTime endTime,
            @RequestParam(required = false) Integer robotId,
            HttpServletRequest httpRequest) {

        if (!isAuthenticated(httpRequest)) {
            return ApiResponseEntity.unauthorizedResponseEntity("관리자 인증이 필요합니다.");
        }

        log.info("[Admin] 주문 로그 조회 요청 - state: {}, startTime: {}, endTime: {}, robotId: {}",
                state, startTime, endTime, robotId);

        List<OrderLogDto> orderLogs = adminService.getOrderLogs(state, startTime, endTime, robotId);

        log.info("[Admin] 주문 로그 조회 완료 - 총 {}건", orderLogs.size());

        return ApiResponseEntity.successResponseEntity(orderLogs);
    }
}