package com.linky.api.robot.service.impl;

import com.linky.api.robot.dto.RobotLocationDto;
import com.linky.api.robot.dto.UpdateRobotStatusDto;
import com.linky.api.robot.entity.RobotLocation;
import com.linky.api.robot.entity.RobotStatus;
import com.linky.api.robot.mapper.RobotMapper;
import com.linky.api.robot.repository.RobotRedisRepository;
import com.linky.api.robot.repository.RobotRepository;
import com.linky.api.robot.service.RobotService;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Service;

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
}
