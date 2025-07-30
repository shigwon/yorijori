package com.linky.api.robot.service;

import com.linky.api.robot.dto.RobotLocationDto;

public interface RobotService {
    void saveLocationToRedis(RobotLocationDto robotLocationDto);
}
