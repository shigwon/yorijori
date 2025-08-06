package com.linky.api.robot.entity;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class RobotStatus {
    int robotId;
    String status;
}
