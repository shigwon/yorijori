package com.linky.api.robot.repository;

import com.linky.api.robot.dto.UpdateRobotStatusDto;
import com.linky.api.robot.entity.RobotStatus;
import org.apache.ibatis.annotations.Mapper;

@Mapper
public interface RobotRepository {
    int updateRobotStatus(RobotStatus robotStatus);
}
