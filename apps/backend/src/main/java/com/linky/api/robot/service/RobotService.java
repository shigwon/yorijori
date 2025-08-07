package com.linky.api.robot.service;

import com.linky.api.robot.dto.RobotDto;
import com.linky.api.robot.dto.RobotListResponseDto;
import com.linky.api.robot.dto.RobotLocationDto;
import com.linky.api.robot.dto.UpdateRobotStatusDto;
import com.linky.api.robot.dto.RobotWithLocationDto;
import com.linky.api.robot.dto.LocationHistoryDto;
import com.linky.api.robot.dto.request.LocationHistoryRequestDto;
import com.linky.api.robot.dto.response.LocationHistoryResponseDto;

import java.util.List;

public interface RobotService {
    // 기존 메서드들
    void saveLocationToRedis(RobotLocationDto robotLocationDto);
    RobotListResponseDto getAllRobots();
    RobotDto getRobotDetail(int robotId);
    RobotListResponseDto getRobotsByStatus(String status);
    RobotWithLocationDto getRobotWithLocation(int robotId);
    List<RobotWithLocationDto> getAllRobotsWithLocation();
    boolean updateRobotStatus(UpdateRobotStatusDto updateRobotStatusDto);

    LocationHistoryResponseDto getLocationHistory(LocationHistoryRequestDto request);

    LocationHistoryDto getLatestStoredLocation(Integer robotId);

    void saveLocationsToPostgreSQL();

    void cleanupOldLocationData();
}