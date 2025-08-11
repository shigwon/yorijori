package com.linky.api.robot.service.impl;

import com.linky.api.robot.dto.RobotLocationDto;
import com.linky.api.robot.dto.UpdateRobotStatusDto;
import com.linky.api.robot.dto.*;
import com.linky.api.robot.dto.request.LocationHistoryRequestDto;
import com.linky.api.robot.dto.response.LocationHistoryResponseDto;
import com.linky.api.robot.entity.Robot;
import com.linky.api.robot.entity.RobotLocation;
import com.linky.api.robot.entity.RobotLocationHistory;
import com.linky.api.robot.entity.RobotStatus;
import com.linky.api.robot.exception.RobotLocationException;
import com.linky.api.robot.mapper.RobotMapper;
import com.linky.api.robot.repository.RobotRedisRepository;
import com.linky.api.robot.repository.mybatis.RobotRepository;
import com.linky.api.robot.repository.jpa.RobotLocationHistoryRepository;
import com.linky.api.robot.service.RobotService;
import com.linky.config.exception.enums.ApiExceptionEnum;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.data.domain.Page;
import org.springframework.data.domain.PageRequest;
import org.springframework.data.domain.Pageable;
import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.time.format.DateTimeParseException;
import java.util.List;
import java.util.Optional;
import java.util.stream.StreamSupport;

@Slf4j
@Service
@RequiredArgsConstructor
public class RobotServiceImpl implements RobotService {

    private final RobotRedisRepository robotRedisRepository;
    private final RobotRepository robotRepository;
    private final RobotLocationHistoryRepository locationHistoryRepository; // 추가
    private final RobotMapper robotMapper;

    private static final DateTimeFormatter DATE_TIME_FORMATTER =
            DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss");

    @Override
    public void saveLocationToRedis(RobotLocationDto robotLocationDto) {
        RobotLocation robotLocation = robotMapper.toEntity(robotLocationDto);
        robotRedisRepository.save(robotLocation);
    }

    @Override
    public boolean updateRobotStatus(UpdateRobotStatusDto updateRobotStatusDto) {
        RobotStatus robotStatus = robotMapper.toEntity(updateRobotStatusDto);
        return robotRepository.updateRobotStatus(robotStatus) > 0;
    }

    public RobotListResponseDto getAllRobots() {
        try {
            List<Robot> robots = robotRepository.findAllRobots();

            List<RobotDto> robotDtos = robots.stream()
                    .map(this::convertToDto)
                    .toList();

            return RobotListResponseDto.builder()
                    .robots(robotDtos)
                    .totalCount(robotDtos.size())
                    .build();

        } catch (Exception e) {
            log.error("로봇 목록 조회 중 오류 발생", e);
            throw new RuntimeException("로봇 목록 조회에 실패했습니다.", e);
        }
    }

    @Override
    public RobotDto getRobotDetail(int robotId) {
        try {
            Optional<Robot> robotOpt = robotRepository.findRobotById(robotId);

            if (robotOpt.isEmpty()) {
                throw new RuntimeException("로봇 ID " + robotId + "를 찾을 수 없습니다.");
            }

            Robot robot = robotOpt.get();
            return convertToDto(robot);

        } catch (Exception e) {
            log.error("로봇 상세 정보 조회 중 오류 발생: robotId={}", robotId, e);
            throw new RuntimeException("로봇 상세 정보 조회에 실패했습니다.", e);
        }
    }

    @Override
    public RobotListResponseDto getRobotsByStatus(String status) {
        try {
            List<Robot> robots = robotRepository.findRobotsByStatus(status);

            List<RobotDto> robotDtos = robots.stream()
                    .map(this::convertToDto)
                    .toList();

            return RobotListResponseDto.builder()
                    .robots(robotDtos)
                    .totalCount(robotDtos.size())
                    .build();

        } catch (Exception e) {
            log.error("로봇 상태별 조회 중 오류 발생: status={}", status, e);
            throw new RuntimeException("로봇 상태별 조회에 실패했습니다.", e);
        }
    }

    @Override
    public RobotWithLocationDto getRobotWithLocation(int robotId) {
        try {
            // 1. MySQL에서 기본 정보 조회
            Optional<Robot> robotOpt = robotRepository.findRobotById(robotId);
            if (robotOpt.isEmpty()) {
                throw new RobotLocationException(ApiExceptionEnum.ROBOT_NOT_FOUND);
            }

            Robot robot = robotOpt.get();

            // 2. Redis에서 위치 정보 조회
            Optional<RobotLocation> locationOpt = robotRedisRepository.findById(String.valueOf(robotId));

            // 3. 통합 DTO 생성 (기존 로직 + 실시간 필드 추가)
            RobotWithLocationDto.RobotWithLocationDtoBuilder builder = RobotWithLocationDto.builder()
                    .id(robot.getId())
                    .code(robot.getCode())
                    .status(robot.getStatus())
                    .statusDisplayName(getStatusDisplayName(robot.getStatus()))
                    .lastChargeTime(robot.getLastChargeTime());

            if (locationOpt.isPresent()) {
                RobotLocation location = locationOpt.get();
                builder
                        .latitude(location.getLatitude())
                        .longitude(location.getLongitude())
                        .locationAvailable(true)
                        .locationStatus("실시간")
                        .locationDisplay(formatLocation(location.getLatitude(), location.getLongitude()))
                        .lastUpdated(LocalDateTime.now()) // 추가
                        .isOnline(true) // 추가
                        .movementStatus("UNKNOWN"); // 추가
            } else {
                builder
                        .latitude(null)
                        .longitude(null)
                        .locationAvailable(false)
                        .locationStatus("오프라인")
                        .locationDisplay("위치 정보 없음")
                        .lastUpdated(null) // 추가
                        .isOnline(false) // 추가
                        .movementStatus("OFFLINE"); // 추가
            }

            return builder.build();

        } catch (Exception e) {
            log.error("로봇 통합 정보 조회 중 오류 발생: robotId={}", robotId, e);
            throw new RuntimeException("로봇 통합 정보 조회에 실패했습니다.", e);
        }
    }

    // 기존 getAllRobotsWithLocation 메서드 확장
    @Override
    public List<RobotWithLocationDto> getAllRobotsWithLocation() {
        try {
            List<Robot> robots = robotRepository.findAllRobots();

            return robots.stream()
                    .map(robot -> {
                        Optional<RobotLocation> locationOpt = robotRedisRepository.findById(String.valueOf(robot.getId()));

                        RobotWithLocationDto.RobotWithLocationDtoBuilder builder = RobotWithLocationDto.builder()
                                .id(robot.getId())
                                .code(robot.getCode())
                                .status(robot.getStatus())
                                .statusDisplayName(getStatusDisplayName(robot.getStatus()))
                                .lastChargeTime(robot.getLastChargeTime());

                        if (locationOpt.isPresent()) {
                            RobotLocation location = locationOpt.get();
                            builder
                                    .latitude(location.getLatitude())
                                    .longitude(location.getLongitude())
                                    .locationAvailable(true)
                                    .locationStatus("실시간")
                                    .locationDisplay(formatLocation(location.getLatitude(), location.getLongitude()))
                                    .lastUpdated(LocalDateTime.now()) // 추가
                                    .isOnline(true) // 추가
                                    .movementStatus("UNKNOWN"); // 추가
                        } else {
                            builder
                                    .latitude(null)
                                    .longitude(null)
                                    .locationAvailable(false)
                                    .locationStatus("오프라인")
                                    .locationDisplay("위치 정보 없음")
                                    .lastUpdated(null) // 추가
                                    .isOnline(false) // 추가
                                    .movementStatus("OFFLINE"); // 추가
                        }

                        return builder.build();
                    })
                    .toList();

        } catch (Exception e) {
            log.error("전체 로봇 통합 정보 조회 중 오류 발생", e);
            throw new RuntimeException("전체 로봇 통합 정보 조회에 실패했습니다.", e);
        }
    }

    // === 새로 추가되는 위치 이력 관련 메서드들 ===

    @Override
    public LocationHistoryResponseDto getLocationHistory(LocationHistoryRequestDto request) {
        // 1. 로봇 존재 확인
        Optional<Robot> robotOpt = robotRepository.findRobotById(request.getRobotId());
        if (robotOpt.isEmpty()) {
            throw new RobotLocationException(ApiExceptionEnum.ROBOT_NOT_FOUND);
        }

        Robot robot = robotOpt.get();

        // 2. 페이징 설정
        Pageable pageable = PageRequest.of(request.getPage() - 1, request.getPageSize());

        // 3. 날짜 범위에 따른 조회
        Page<RobotLocationHistory> locationPage;
        LocalDateTime startDate = null;
        LocalDateTime endDate = null;

        if (request.getStartDate() != null && request.getEndDate() != null) {
            try {
                startDate = LocalDateTime.parse(request.getStartDate(), DATE_TIME_FORMATTER);
                endDate = LocalDateTime.parse(request.getEndDate(), DATE_TIME_FORMATTER);

                if (startDate.isAfter(endDate)) {
                    throw new RobotLocationException(ApiExceptionEnum.INVALID_DATE_RANGE);
                }

                locationPage = locationHistoryRepository.findByRobotIdAndRecordedAtBetweenOrderByRecordedAtDesc(
                        request.getRobotId(), startDate, endDate, pageable);
            } catch (DateTimeParseException e) {
                throw new RobotLocationException(ApiExceptionEnum.INVALID_DATE_RANGE);
            }
        } else {
            locationPage = locationHistoryRepository.findByRobotIdOrderByRecordedAtDesc(
                    request.getRobotId(), pageable);
        }

        // 4. DTO 변환
        List<LocationHistoryDto> locationDtos = locationPage.getContent().stream()
                .map(history -> convertToHistoryDto(history, robot.getCode()))
                .toList();

        return LocationHistoryResponseDto.builder()
                .locations(locationDtos)
                .totalCount(locationPage.getTotalElements())
                .currentPage(request.getPage())
                .totalPages(locationPage.getTotalPages())
                .hasNext(locationPage.hasNext())
                .hasPrevious(locationPage.hasPrevious())
                .robotId(request.getRobotId())
                .robotCode(robot.getCode())
                .periodStart(startDate)
                .periodEnd(endDate)
                .build();
    }

    @Override
    public LocationHistoryDto getLatestStoredLocation(Integer robotId) {
        Optional<Robot> robotOpt = robotRepository.findRobotById(robotId);
        if (robotOpt.isEmpty()) {
            throw new RobotLocationException(ApiExceptionEnum.ROBOT_NOT_FOUND);
        }

        Optional<RobotLocationHistory> latestLocation =
                locationHistoryRepository.findFirstByRobotIdOrderByRecordedAtDesc(robotId);

        if (latestLocation.isEmpty()) {
            throw new RobotLocationException(ApiExceptionEnum.LOCATION_HISTORY_NOT_FOUND);
        }

        return convertToHistoryDto(latestLocation.get(), robotOpt.get().getCode());
    }

    @Scheduled(fixedRate = 1000) // 1초마다 실행
    @Transactional("postgresTransactionManager")
    @Override
    public void saveLocationsToPostgreSQL() {
//        try {
//            log.debug("Redis → PostgreSQL 위치 데이터 배치 저장 시작");
//
//            // 1. Redis에서 모든 로봇 위치 조회
//            Iterable<RobotLocation> redisLocations = robotRedisRepository.findAll();
//            System.out.println(redisLocations.toString());
//
//            // 2. PostgreSQL에 저장
//            List<RobotLocationHistory> historyList = StreamSupport.stream(redisLocations.spliterator(), false)
//                    .map(redisLocation -> RobotLocationHistory.builder()
//                            .robotId(redisLocation.getRobotId())
//                            .latitude(redisLocation.getLatitude())
//                            .longitude(redisLocation.getLongitude())
//                            .recordedAt(LocalDateTime.now())
//                            .build())
//                    .toList();
//
//            if (!historyList.isEmpty()) {
//                locationHistoryRepository.saveAll(historyList);
//                log.debug("위치 데이터 {}건 PostgreSQL에 저장 완료", historyList.size());
//            }
//
//        } catch (Exception e) {
//            log.error("Redis → PostgreSQL 위치 데이터 배치 저장 실패", e);
//        }
    }

    @Scheduled(cron = "0 0 2 * * ?") // 매일 새벽 2시 실행
    @Transactional("postgresTransactionManager")
    @Override
    public void cleanupOldLocationData() {
        try {
            log.info("오래된 위치 데이터 정리 시작");

            // 7일 이전 데이터 삭제
            LocalDateTime cutoffDate = LocalDateTime.now().minusDays(7);
            locationHistoryRepository.deleteByRecordedAtBefore(cutoffDate);

            log.info("오래된 위치 데이터 정리 완료 ({}일 이전 데이터)", 7);

        } catch (Exception e) {
            log.error("오래된 위치 데이터 정리 실패", e);
        }
    }

    // === 기존 유틸리티 메서드들 ===

    private RobotDto convertToDto(Robot robot) {
        return RobotDto.builder()
                .id(robot.getId())
                .code(robot.getCode())
                .status(robot.getStatus())
                .statusDisplayName(getStatusDisplayName(robot.getStatus()))
                .lastChargeTime(robot.getLastChargeTime())
                .build();
    }

    private String formatLocation(double latitude, double longitude) {
        return String.format("%.6f, %.6f", latitude, longitude);
    }

    private String getStatusDisplayName(String status) {
        return switch (status) {
            case "WORKING" -> "주행중";
            case "WAITING" -> "대기중";
            default -> "알 수 없음";
        };
    }

    // 새로 추가되는 유틸리티 메서드
    private LocationHistoryDto convertToHistoryDto(RobotLocationHistory history, String robotCode) {
        return LocationHistoryDto.builder()
                .id(history.getId())
                .robotId(history.getRobotId())
                .robotCode(robotCode)
                .latitude(history.getLatitude())
                .longitude(history.getLongitude())
                .recordedAt(history.getRecordedAt())
                .locationDisplay(formatLocation(history.getLatitude(), history.getLongitude()))
                .build();
    }
}


