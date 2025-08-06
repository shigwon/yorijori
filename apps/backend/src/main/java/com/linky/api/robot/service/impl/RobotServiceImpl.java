// com.linky.api.robot.service.impl.RobotServiceImpl (확장)
package com.linky.api.robot.service.impl;

import com.linky.api.robot.dto.RobotLocationDto;
import com.linky.api.robot.dto.UpdateRobotStatusDto;
import com.linky.api.robot.dto.*;
import com.linky.api.robot.entity.Robot;
import com.linky.api.robot.entity.RobotLocation;
import com.linky.api.robot.entity.RobotStatus;
import com.linky.api.robot.mapper.RobotMapper;
import com.linky.api.robot.repository.RobotRedisRepository;
import com.linky.api.robot.repository.RobotRepository;
import com.linky.api.robot.service.RobotService;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Service;

import java.util.List;
import java.util.Optional;

@Slf4j
@Service
@RequiredArgsConstructor
public class RobotServiceImpl implements RobotService {

    private final RobotRedisRepository robotRedisRepository;
    private final RobotRepository robotRepository;
    private final RobotMapper robotMapper;

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
                throw new RuntimeException("로봇 ID " + robotId + "를 찾을 수 없습니다.");
            }

            Robot robot = robotOpt.get();

            // 2. Redis에서 위치 정보 조회
            Optional<RobotLocation> locationOpt = robotRedisRepository.findById(String.valueOf(robotId));

            // 3. 통합 DTO 생성
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
                        .locationDisplay(formatLocation(location.getLatitude(), location.getLongitude()));
            } else {
                builder
                        .latitude(null)
                        .longitude(null)
                        .locationAvailable(false)
                        .locationStatus("오프라인")
                        .locationDisplay("위치 정보 없음");
            }

            return builder.build();

        } catch (Exception e) {
            log.error("로봇 통합 정보 조회 중 오류 발생: robotId={}", robotId, e);
            throw new RuntimeException("로봇 통합 정보 조회에 실패했습니다.", e);
        }
    }

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
                                    .locationDisplay(formatLocation(location.getLatitude(), location.getLongitude()));
                        } else {
                            builder
                                    .latitude(null)
                                    .longitude(null)
                                    .locationAvailable(false)
                                    .locationStatus("오프라인")
                                    .locationDisplay("위치 정보 없음");
                        }

                        return builder.build();
                    })
                    .toList();

        } catch (Exception e) {
            log.error("전체 로봇 통합 정보 조회 중 오류 발생", e);
            throw new RuntimeException("전체 로봇 통합 정보 조회에 실패했습니다.", e);
        }
    }

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
}

