package com.linky.api.robot.repository;

import com.linky.api.robot.entity.Robot;
import com.linky.api.robot.entity.RobotStatus;
import org.apache.ibatis.annotations.Mapper;
import org.apache.ibatis.annotations.Param;

import java.util.List;
import java.util.Optional;

@Mapper
public interface RobotRepository {

    List<Robot> findAllRobots();

    Optional<Robot> findRobotById(@Param("id") int id);

    List<Robot> findRobotsByStatus(@Param("status") String status);

    int countAllRobots();

    int updateRobotStatus(RobotStatus robotStatus);
}

