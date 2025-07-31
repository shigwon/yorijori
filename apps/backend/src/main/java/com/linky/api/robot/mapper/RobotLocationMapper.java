package com.linky.api.robot.mapper;

import com.linky.api.robot.dto.RobotLocationDto;
import com.linky.api.robot.entity.RobotLocation;
import org.springframework.stereotype.Component;

@Component
public class RobotLocationMapper {

    public RobotLocation toEntity(RobotLocationDto robotLocationDto) {
        return RobotLocation.builder()
                .robotId(robotLocationDto.getRobotId())
                .latitude(robotLocationDto.getLatitude())
                .longitude(robotLocationDto.getLongitude())
                .build();
    }
}
