package com.linky.api.robot.mapper;

import com.linky.api.robot.dto.RobotLocationDto;
import com.linky.api.robot.dto.UpdateRobotStatusDto;
import com.linky.api.robot.entity.RobotLocation;
import com.linky.api.robot.entity.RobotStatus;
import org.springframework.stereotype.Component;

@Component
public class RobotMapper {

    public RobotLocation toEntity(RobotLocationDto robotLocationDto) {
        return RobotLocation.builder()
                .robotId(robotLocationDto.getRobotId())
                .latitude(robotLocationDto.getLatitude())
                .longitude(robotLocationDto.getLongitude())
                .build();
    }

    public RobotStatus toEntity(UpdateRobotStatusDto updateRobotStatusDto) {
        return RobotStatus.builder()
                .robotId(updateRobotStatusDto.getRobotId())
                .status(updateRobotStatusDto.getStatus())
                .build();
    }
}
