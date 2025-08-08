package com.linky.api.admin.controller;

import com.linky.api.admin.service.AdminService;
import com.linky.api.common.response.entity.ApiResponseEntity;
import com.linky.api.robot.dto.RobotWithLocationDto;
import com.linky.api.robot.dto.request.LocationHistoryRequestDto;
import com.linky.api.robot.dto.response.LocationHistoryResponseDto;
import com.linky.api.robot.service.RobotService;
import com.linky.api.admin.dto.response.RobotLocationStatisticsDto;
import jakarta.servlet.http.HttpServletRequest;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import jakarta.validation.Valid;
import java.util.List;

@Slf4j
@RestController
@RequestMapping("/api/admin/robots")
@RequiredArgsConstructor
public class AdminRobotLocationController {

    private final RobotService robotService;
    private final AdminService adminService;

    private boolean isAuthenticated(HttpServletRequest request) {
        return adminService.isValidSession(request.getSession(false));
    }

    @GetMapping("/location/monitor")
    public ResponseEntity<ApiResponseEntity<List<RobotWithLocationDto>>> monitorAllRobotLocations(
            HttpServletRequest httpRequest) {

        if (!isAuthenticated(httpRequest)) {
            return ApiResponseEntity.unauthorizedResponseEntity("관리자 인증이 필요합니다.");
        }

        log.info("[Admin] 전체 로봇 위치 모니터링 요청");

        List<RobotWithLocationDto> locations = robotService.getAllRobotsWithLocation();

        long onlineCount = locations.stream().mapToLong(loc -> loc.isOnline() ? 1 : 0).sum();
        long offlineCount = locations.size() - onlineCount;

        log.info("[Admin] 전체 로봇 위치 모니터링 완료: 총 {}대 (온라인: {}대, 오프라인: {}대)",
                locations.size(), onlineCount, offlineCount);

        return ApiResponseEntity.successResponseEntity(locations);
    }

    @PostMapping("/{robotId}/location/history")
    public ResponseEntity<ApiResponseEntity<LocationHistoryResponseDto>> getDetailedLocationHistory(
            @PathVariable("robotId") Integer robotId,
            @Valid @RequestBody LocationHistoryRequestDto request,
            HttpServletRequest httpRequest) {

        if (!isAuthenticated(httpRequest)) {
            return ApiResponseEntity.unauthorizedResponseEntity("관리자 인증이 필요합니다.");
        }

        request.setRobotId(robotId);

        log.info("[Admin] 로봇 위치 이력 상세 조회 요청: robotId={}, request={}", robotId, request);

        LocationHistoryResponseDto response = robotService.getLocationHistory(request);

        log.info("[Admin] 로봇 위치 이력 상세 조회 완료: robotId={}, 총 {}건",
                robotId, response.getTotalCount());

        return ApiResponseEntity.successResponseEntity(response);
    }

    @GetMapping("/location/statistics")
    public ResponseEntity<ApiResponseEntity<RobotLocationStatisticsDto>> getLocationStatistics(
            HttpServletRequest httpRequest) {

        if (!isAuthenticated(httpRequest)) {
            return ApiResponseEntity.unauthorizedResponseEntity("관리자 인증이 필요합니다.");
        }

        log.info("[Admin] 로봇 위치 데이터 통계 조회 요청");

        List<RobotWithLocationDto> currentLocations = robotService.getAllRobotsWithLocation();

        long totalRobots = currentLocations.size();
        long onlineRobots = currentLocations.stream().mapToLong(loc -> loc.isOnline() ? 1 : 0).sum();
        long offlineRobots = totalRobots - onlineRobots;
        double onlineRate = totalRobots > 0 ? (double) onlineRobots / totalRobots * 100 : 0;

        long movingRobots = currentLocations.stream()
                .mapToLong(loc -> "MOVING".equals(loc.getMovementStatus()) ? 1 : 0).sum();
        long stationaryRobots = currentLocations.stream()
                .mapToLong(loc -> "STATIONARY".equals(loc.getMovementStatus()) ? 1 : 0).sum();

        RobotLocationStatisticsDto statistics = RobotLocationStatisticsDto.builder()
                .totalRobots(totalRobots)
                .onlineRobots(onlineRobots)
                .offlineRobots(offlineRobots)
                .onlineRate(Math.round(onlineRate * 100.0) / 100.0)
                .movingRobots(movingRobots)
                .stationaryRobots(stationaryRobots)
                .lastUpdated(java.time.LocalDateTime.now())
                .build();

        log.info("[Admin] 로봇 위치 데이터 통계 조회 완료: {}", statistics);

        return ApiResponseEntity.successResponseEntity(statistics);
    }
}