package com.linky.api.robot.entity;

import lombok.Builder;
import lombok.Data;
import org.springframework.data.annotation.Id;
import org.springframework.data.redis.core.RedisHash;

@Data
@Builder
@RedisHash("robotLocation")
public class RobotLocation {
    @Id
    int robotId;
    double latitude;
    double longitude;
}
