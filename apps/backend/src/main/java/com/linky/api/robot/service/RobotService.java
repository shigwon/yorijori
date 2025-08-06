package com.linky.api.robot.service;

import com.linky.api.robot.dto.RobotDto;
import com.linky.api.robot.dto.RobotListResponseDto;
import com.linky.api.robot.dto.RobotLocationDto;
import com.linky.api.robot.dto.UpdateRobotStatusDto;
import com.linky.api.robot.dto.RobotWithLocationDto;

import java.util.List;

public interface RobotService {
    void saveLocationToRedis(RobotLocationDto robotLocationDto);

    RobotListResponseDto getAllRobots();

    RobotDto getRobotDetail(int robotId);

    RobotListResponseDto getRobotsByStatus(String status);

    RobotWithLocationDto getRobotWithLocation(int robotId);

    List<RobotWithLocationDto> getAllRobotsWithLocation();

    void saveLocationToRedis(RobotLocationDto robotLocationDto);

    boolean updateRobotStatus(UpdateRobotStatusDto updateRobotStatusDto);
}
