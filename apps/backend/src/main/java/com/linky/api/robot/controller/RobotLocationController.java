package com.linky.api.robot.controller;

import com.linky.api.common.response.entity.ApiResponseEntity;
import com.linky.api.robot.dto.RobotWithLocationDto;
import com.linky.api.robot.dto.LocationHistoryDto;
import com.linky.api.robot.dto.request.LocationHistoryRequestDto;
import com.linky.api.robot.dto.response.LocationHistoryResponseDto;
import com.linky.api.robot.service.RobotService;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.http.ResponseEntity;
import org.springframework.validation.annotation.Validated;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@Slf4j
@RestController
@RequestMapping("/api/v1/robots")
@RequiredArgsConstructor
@Validated
public class RobotLocationController {

    private final RobotService robotService;

    @GetMapping("/{robotId}/location/current")
    public ResponseEntity<ApiResponseEntity<RobotWithLocationDto>> getCurrentLocation(
            @PathVariable("robotId") Integer robotId) {

        log.info("로봇 실시간 위치 조회 요청: robotId={}", robotId);

        RobotWithLocationDto currentLocation = robotService.getRobotWithLocation(robotId);

        log.info("로봇 실시간 위치 조회 완료: robotId={}, location={}",
                robotId, currentLocation.getLocationDisplay());

        return ApiResponseEntity.successResponseEntity(currentLocation);
    }

    @GetMapping("/location/current")
    public ResponseEntity<ApiResponseEntity<List<RobotWithLocationDto>>> getAllCurrentLocations() {

        log.info("전체 로봇 실시간 위치 조회 요청");

        List<RobotWithLocationDto> currentLocations = robotService.getAllRobotsWithLocation();

        long onlineCount = currentLocations.stream().mapToLong(loc -> loc.isOnline() ? 1 : 0).sum();
        log.info("전체 로봇 실시간 위치 조회 완료: 총 {}대 (온라인: {}대)",
                currentLocations.size(), onlineCount);

        return ApiResponseEntity.successResponseEntity(currentLocations);
    }

    @GetMapping("/{robotId}/location/history")
    public ResponseEntity<ApiResponseEntity<LocationHistoryResponseDto>> getLocationHistory(
            @PathVariable("robotId") Integer robotId,
            @RequestParam(defaultValue = "1") int page,
            @RequestParam(defaultValue = "20") int pageSize,
            @RequestParam(required = false) String startDate,
            @RequestParam(required = false) String endDate) {

        log.info("로봇 위치 이력 조회 요청: robotId={}, page={}, pageSize={}, startDate={}, endDate={}",
                robotId, page, pageSize, startDate, endDate);

        LocationHistoryRequestDto request = LocationHistoryRequestDto.builder()
                .robotId(robotId)
                .page(page)
                .pageSize(pageSize)
                .startDate(startDate)
                .endDate(endDate)
                .build();

        LocationHistoryResponseDto response = robotService.getLocationHistory(request);

        log.info("로봇 위치 이력 조회 완료: robotId={}, 총 {}건, 현재 페이지: {}/{}",
                robotId, response.getTotalCount(), response.getCurrentPage(), response.getTotalPages());

        return ApiResponseEntity.successResponseEntity(response);
    }

    @GetMapping("/{robotId}/location/latest")
    public ResponseEntity<ApiResponseEntity<LocationHistoryDto>> getLatestStoredLocation(
            @PathVariable("robotId") Integer robotId) {

        log.info("로봇 최근 저장 위치 조회 요청: robotId={}", robotId);

        LocationHistoryDto latestLocation = robotService.getLatestStoredLocation(robotId);

        log.info("로봇 최근 저장 위치 조회 완료: robotId={}, recordedAt={}",
                robotId, latestLocation.getRecordedAt());

        return ApiResponseEntity.successResponseEntity(latestLocation);
    }
}