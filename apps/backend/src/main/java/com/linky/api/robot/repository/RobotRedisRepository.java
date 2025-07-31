package com.linky.api.robot.repository;

import com.linky.api.robot.entity.RobotLocation;
import org.springframework.data.repository.CrudRepository;

public interface RobotRedisRepository extends CrudRepository<RobotLocation, String> {

}
