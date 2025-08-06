package com.linky.api.robot.service;

import com.linky.api.robot.dto.RobotLocationDto;
import com.linky.api.robot.dto.UpdateRobotStatusDto;

public interface RobotService {
    void saveLocationToRedis(RobotLocationDto robotLocationDto);
    boolean updateRobotStatus(UpdateRobotStatusDto updateRobotStatusDto);
}
