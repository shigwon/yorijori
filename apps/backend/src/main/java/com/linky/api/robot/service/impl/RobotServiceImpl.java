package com.linky.api.robot.service.impl;

import com.linky.api.robot.dto.RobotLocationDto;
import com.linky.api.robot.entity.RobotLocation;
import com.linky.api.robot.mapper.RobotLocationMapper;
import com.linky.api.robot.repository.RobotRedisRepository;
import com.linky.api.robot.service.RobotService;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;

@Service
@RequiredArgsConstructor
public class RobotServiceImpl implements RobotService {

    private final RobotRedisRepository robotRedisRepository;
    private final RobotLocationMapper robotLocationMapper;

    public void saveLocationToRedis(RobotLocationDto robotLocationDto) {
        RobotLocation robotLocation = robotLocationMapper.toEntity(robotLocationDto);
        robotRedisRepository.save(robotLocation);
    }
}
